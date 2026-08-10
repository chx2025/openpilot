#!/usr/bin/env python3
"""
Lane Centering Corrector (LCC) - 多點曲線擬合與動態融合版 (高速優化 + Rate Limiter + E2E Yield)

將 10~50m 範圍內的左右車道線計算出多個中心點，並利用 np.polyfit 擬合出 
y = ax^2 + bx + c 的二次曲線。藉此精準分離「道路曲率(a)」、「航向誤差(b)」與「橫向偏移(c)」。
針對高速公路行駛優化：引入動態前視距離 (車速 x 1.5秒)，車速越快看得越遠。
隨後算出目標居中曲率，並依據標線信心度與 Model 原始輸出的 desiredCurvature 進行按比例融合，
提供極度平滑且不會因座標系旋轉而自激振盪的居中修正。

本版變更：
1. Rate limiter：新增 MAX_CORRECTION_RATE，修正量每幀變化被限制在固定變化率內，
   不論目標修正量多大，都是「盡量朝目標靠近」而不是「瞬間跳過去」。所有會更新
   self.correction 的路徑（ACTIVE / YIELDING / LOW_CONFIDENCE / INTERP_FAIL）都
   先經過 _rate_limit()，再丟進 FirstOrderFilter，避免任何分支瞬間跳變。

2. E2E 路徑信心度 yield 機制（參考 lane_centering(1).py 的做法）：
   用 model_v2.position (E2E 路徑本身，不是車道線) 在前視距離 L 處的 y 座標，
   跟車道中心線在同一位置的 y 值算出差距 pos_error。同時取 model_v2.position.yStd
   在 L 處的值當作模型對這條路徑的「信心度」。
   - 如果 yStd 很低（模型對這條路徑很有把握）且 pos_error 夠大（明顯偏離車道中心），
     判定為「刻意閃避」，用 break_in 比例把 raw_correction 依 e2e_authority 往下壓，
     讓 LCC 適度讓位給 model。
   - 如果 yStd 偏高（模型自己對這條路徑也不確定，比較像雜訊而非刻意決策），
     不啟動 yield，維持原本的車道置中修正力道。
   - 若 position/yStd 資料缺失或內插失敗，安全 fallback 為 yield_factor=1.0
     （不啟動 yield，維持既有行為）。
"""

import os
import time
import numpy as np

from openpilot.common.params import Params
from openpilot.common.filter_simple import FirstOrderFilter

PARAM_REFRESH_SEC = 2.0

# --- Debug log 設定（獨立 CSV，供路測後回傳分析用） ---
LOG_PATH = "/data/media/0/realdata/lcc_debug.csv"
LOG_INTERVAL_SEC = 0.1  
LOG_COLUMNS = [
  "t", "dt", "v_kph", "state",
  "lat_active", "speed_gate", "sharp_turn", "model_curvature",
  "yield_hold_timer", "engage_ramp_timer", "ramp_factor",
  "weight", "poly_a", "poly_b", "poly_c", "lane_target_curv",
  "path_std", "pos_error", "yield_factor",
  "raw_correction", "rate_limited_correction", "correction",
]

# --- 系統內建參數 ---
SPEED_ON_KPH = 40.0
SPEED_OFF_KPH = 30.0
KPH_TO_MS = 1000.0 / 3600.0

# 高速優化：將採樣點向遠處延伸 (10m ~ 50m)
FIT_X = np.array([10.0, 20.0, 30.0, 40.0, 50.0])

# 動態前視距離參數
MIN_LOOKAHEAD_M = 15.0      # 低速時最低保底前視距離
LOOKAHEAD_TIME_SEC = 1.5    # 依據車速計算前視距離的秒數 (110km/h 約等於看 45m)

FILTER_RC_SEC = 0.5      
SHARP_TURN_CURVATURE = 0.06 

PROB_MIN = 0.3
PROB_FULL = 0.6

# 融合權重上限：當車道線極度清晰時，LCC 修正量最高佔最終軌跡的比例
# 例如 0.25 代表 25% 依賴車道線幾何，75% 依賴 E2E 模型預測
LANE_WEIGHT_MAX = 0.90 
MAX_CORRECTION = 0.012  # 修正量曲率上限 (1/m)

# 修正量變化率限制：不論目標修正量多大，每秒最多只能朝該方向移動這麼多曲率，
# 用來抑制單幀雜訊點或權重/yield 切換造成的方向盤震盪
MAX_CORRECTION_RATE = 0.004  # 1/m per sec

YIELD_CONFIRM_SEC = 0.15
ENGAGE_RAMP_SEC = 1.5

# --- E2E 路徑信心度 yield 參數（借用 lane_centering(1).py 的邏輯與預設值） ---
# model_v2.position.yStd 在前視距離處的值 <= 此門檻，才視為「模型對這條路徑有把握」，
# 才有資格啟動 yield；yStd 太大代表模型自己都不確定，視為雜訊，不啟動 yield。
YIELD_MAX_PATH_STD = 0.35
# pos_error（E2E 路徑與車道中心線在前視距離處的差距，單位公尺）小於此值時，
# 視為正常誤差雜訊，不啟動 yield。
YIELD_BREAK_IN_START = 0.15
# pos_error 大於等於此值時，break_in 比例封頂在 1.0（等於用滿 e2e_authority）。
YIELD_BREAK_IN_FULL = 0.50
# 預設 e2e_authority：yield 機制最多能把 raw_correction 壓低的比例上限。
# 1.0 代表信心度與偏移量都拉滿時，可以把 LCC 修正完全讓給 model。
DEFAULT_E2E_AUTHORITY = 1.0


def _clip_interp(x, xp, fp):
  """對 modelV2 XYZTData 的 x/y 陣列做前視距離內插，資料異常時回傳 None"""
  if len(xp) < 2:
    return None
  if xp[-1] <= xp[0]:
    return None
  if x < xp[0] or x > xp[-1]:
    return None
  return float(np.interp(x, xp, fp))


class LaneCenteringCorrector:
  def __init__(self) -> None:
    self._params = Params()
    self._last_params_read = 0.0
    self._enabled = False

    # --- 狀態 ---
    self._filter = FirstOrderFilter(0.0, FILTER_RC_SEC, 0.01)
    self.correction = 0.0
    self.weight = 0.0
    self._active = False
    self._speed_gate = False  
    self._yield_hold_timer = 0.0  
    self._engage_ramp_timer = 0.0  

    # --- debug log 狀態 ---
    self._last_log_time = 0.0
    self._last_logged_state = None
    self._log_header_written = os.path.exists(LOG_PATH)

  def _read_params(self) -> None:
    now = time.monotonic()
    if now - self._last_params_read < PARAM_REFRESH_SEC:
      return
    self._last_params_read = now
    self._enabled = self._params.get_bool("dp_lcc_enabled")

  def reset(self) -> None:
    self._filter.x = 0.0
    self.correction = 0.0
    self.weight = 0.0
    self._active = False
    self._speed_gate = False
    self._yield_hold_timer = 0.0
    self._engage_ramp_timer = 0.0

  def _rate_limit(self, target: float, dt: float) -> float:
    """將 self.correction 往 target 方向移動，但單幀移動量不超過 MAX_CORRECTION_RATE * dt。
    在濾波之前套用，擋住單幀跳動；FirstOrderFilter 只是在此基礎上做額外平滑，
    不會反過來放大 rate limiter 已經限制過的變化量。"""
    max_delta = MAX_CORRECTION_RATE * dt
    return float(np.clip(target, self.correction - max_delta, self.correction + max_delta))

  def _yield_factor(self, model_v2, center_y_l: float, l: float, e2e_authority: float) -> tuple[float, float, float]:
    """依據 model_v2.position 與 position.yStd 判斷 model 是否「刻意」偏離車道中心。
    回傳 (yield_factor, path_std, pos_error)，資料缺失或無效時安全 fallback 為
    yield_factor=1.0（不啟動 yield，維持原本車道置中力道），path_std/pos_error 回傳 -1.0 方便 log 辨識。"""
    try:
      pos_x = np.asarray(model_v2.position.x, dtype=float)
      pos_y = np.asarray(model_v2.position.y, dtype=float)
      pos_y_std = np.asarray(model_v2.position.yStd, dtype=float)

      if pos_x.size < 2 or pos_x.size != pos_y.size or pos_x.size != pos_y_std.size:
        return 1.0, -1.0, -1.0
      if not (np.isfinite(pos_x).all() and np.isfinite(pos_y).all() and np.isfinite(pos_y_std).all()):
        return 1.0, -1.0, -1.0
      if not np.all(np.diff(pos_x) > 0):
        return 1.0, -1.0, -1.0
      if l < pos_x[0] or l > pos_x[-1]:
        return 1.0, -1.0, -1.0

      model_y = float(np.interp(l, pos_x, pos_y))
      path_std = float(np.interp(l, pos_x, pos_y_std))

      pos_error = center_y_l - model_y
      error_abs = abs(pos_error)

      if not (0.0 <= path_std <= YIELD_MAX_PATH_STD):
        # model 自己對這條路徑也不確定，比較像雜訊，不啟動 yield
        return 1.0, path_std, pos_error

      break_in = float(np.clip(
        (error_abs - YIELD_BREAK_IN_START) / (YIELD_BREAK_IN_FULL - YIELD_BREAK_IN_START),
        0.0, 1.0,
      ))
      yield_factor = 1.0 - float(np.clip(e2e_authority, 0.0, 1.0)) * break_in
      return yield_factor, path_std, pos_error
    except (AttributeError, TypeError, ValueError, IndexError):
      return 1.0, -1.0, -1.0

  def _log_row(self, state: str, **fields) -> None:
    now = time.monotonic()
    state_changed = state != self._last_logged_state
    if not state_changed and (now - self._last_log_time) < LOG_INTERVAL_SEC:
      return
    self._last_log_time = now
    self._last_logged_state = state
    try:
      row = {"t": f"{time.time():.3f}", "state": state}
      for col in LOG_COLUMNS:
        if col in ("t", "state"):
          continue
        val = fields.get(col, "")
        row[col] = f"{val:.5f}" if isinstance(val, float) else str(val)
      os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)
      write_header = not self._log_header_written
      with open(LOG_PATH, "a", encoding="utf-8") as f:
        if write_header:
          f.write(",".join(LOG_COLUMNS) + "\n")
          self._log_header_written = True
        f.write(",".join(row.get(col, "") for col in LOG_COLUMNS) + "\n")
    except Exception:
      pass

  def _log_fallback(self, state: str, dt, v_ego_kph, lat_active, is_sharp_turn, model_curvature, ramp_factor):
    """簡化提早 return 時的預設 log 記錄"""
    self._log_row(state, dt=dt, v_kph=v_ego_kph, lat_active=lat_active,
                  speed_gate=self._speed_gate, sharp_turn=is_sharp_turn,
                  model_curvature=model_curvature, yield_hold_timer=self._yield_hold_timer,
                  engage_ramp_timer=self._engage_ramp_timer, ramp_factor=ramp_factor,
                  weight=0.0, poly_a=0.0, poly_b=0.0, poly_c=0.0, 
                  lane_target_curv=0.0, path_std=-1.0, pos_error=0.0, yield_factor=1.0,
                  raw_correction=0.0, rate_limited_correction=0.0, correction=self.correction)

  def update(self, model_v2, v_ego: float, lat_active: bool, dt: float,
             left_blinker: bool = False, right_blinker: bool = False,
             steering_pressed: bool = False, e2e_authority: float = DEFAULT_E2E_AUTHORITY) -> float:
    self._read_params()
    self._filter.dt = dt

    v_ego_kph = v_ego / KPH_TO_MS
    if v_ego_kph >= SPEED_ON_KPH:
      self._speed_gate = True
    elif v_ego_kph <= SPEED_OFF_KPH:
      self._speed_gate = False

    model_curvature = getattr(model_v2.action, "desiredCurvature", 0.0) if lat_active else 0.0
    is_sharp_turn = abs(model_curvature) > SHARP_TURN_CURVATURE

    hard_invalid = (not self._enabled or not lat_active or not self._speed_gate or is_sharp_turn)
    if hard_invalid:
      _state = "DISABLED" if not self._enabled else \
               "NOT_LAT_ACTIVE" if not lat_active else \
               "SPEED_GATE_OFF" if not self._speed_gate else "SHARP_TURN"
      self.reset()
      self._log_fallback(_state, dt, v_ego_kph, lat_active, is_sharp_turn, model_curvature, 0.0)
      return 0.0

    self._engage_ramp_timer += dt
    ramp_factor = float(np.clip(self._engage_ramp_timer / ENGAGE_RAMP_SEC, 0.0, 1.0))

    raw_yield_condition = (left_blinker or right_blinker) and steering_pressed
    if raw_yield_condition:
      self._yield_hold_timer += dt
    else:
      self._yield_hold_timer = 0.0
    is_yielding = self._yield_hold_timer >= YIELD_CONFIRM_SEC

    if is_yielding:
      self._active = False
      self.weight = 0.0
      limited = self._rate_limit(0.0, dt)
      self.correction = self._filter.update(limited)
      self._log_fallback("YIELDING", dt, v_ego_kph, lat_active, is_sharp_turn, model_curvature, ramp_factor)
      return self.correction

    lane_lines = model_v2.laneLines
    lane_line_probs = model_v2.laneLineProbs

    if len(lane_lines) < 3 or len(lane_line_probs) < 3:
      self.reset()
      self._log_fallback("NO_LANE_DATA", dt, v_ego_kph, lat_active, is_sharp_turn, model_curvature, ramp_factor)
      return 0.0

    lll_prob = lane_line_probs[1]
    rll_prob = lane_line_probs[2]
    min_prob = min(lll_prob, rll_prob)

    # 動態權重：信心度決定融合比例 (最高 LANE_WEIGHT_MAX)
    confidence = float(np.clip((min_prob - PROB_MIN) / (PROB_FULL - PROB_MIN), 0.0, 1.0))
    self.weight = confidence * LANE_WEIGHT_MAX

    if self.weight <= 0.0:
      self._active = False
      limited = self._rate_limit(0.0, dt)
      self.correction = self._filter.update(limited)
      self._log_fallback("LOW_CONFIDENCE", dt, v_ego_kph, lat_active, is_sharp_turn, model_curvature, ramp_factor)
      return self.correction

    lll = lane_lines[1]
    rll = lane_lines[2]
    
    # 取樣 10~50m 多個座標點計算中心
    valid_x = []
    center_y = []
    for x in FIT_X:
      l_y = _clip_interp(x, lll.x, lll.y)
      r_y = _clip_interp(x, rll.x, rll.y)
      if l_y is not None and r_y is not None:
        valid_x.append(x)
        center_y.append((l_y + r_y) / 2.0)

    # 二次曲線擬合需要至少 3 個有效點
    if len(valid_x) < 3:
      self._active = False
      limited = self._rate_limit(0.0, dt)
      self.correction = self._filter.update(limited)
      self._log_fallback("INTERP_FAIL", dt, v_ego_kph, lat_active, is_sharp_turn, model_curvature, ramp_factor)
      return self.correction

    # 擬合二次曲線方程式 y = ax^2 + bx + c
    coeffs = np.polyfit(valid_x, center_y, 2)
    a, b, c = coeffs[0], coeffs[1], coeffs[2]

    # 高速優化：依據車速計算動態前視距離 (L)
    L = max(MIN_LOOKAHEAD_M, v_ego * LOOKAHEAD_TIME_SEC)
    
    # 利用公式：曲率 = 2a + 2b/L + 2c/L^2，算出最符合車道中心的理論曲率
    lane_target_curvature = (2.0 * a) + (2.0 * b / L) + (2.0 * c / (L ** 2))

    # 車道中心線在前視距離 L 處的 y 值，供 yield 機制跟 E2E 路徑做比較
    center_y_l = a * (L ** 2) + b * L + c

    # E2E 路徑信心度 yield：model 對自己路徑有把握 (yStd 低) 且明顯偏離車道中心時，
    # 判定為刻意閃避，把修正量依 e2e_authority 往下壓
    yield_factor, path_std, pos_error = self._yield_factor(model_v2, center_y_l, L, e2e_authority)

    # 動態路徑融合 (Path Fusion):
    raw_correction = self.weight * ramp_factor * yield_factor * (lane_target_curvature - model_curvature)
    raw_correction = float(np.clip(raw_correction, -MAX_CORRECTION, MAX_CORRECTION))

    # Rate limiter：抑制單幀雜訊點或 yield 切換瞬間造成的方向盤震盪
    rate_limited_correction = self._rate_limit(raw_correction, dt)

    self.correction = self._filter.update(rate_limited_correction)
    self._active = True

    self._log_row("ACTIVE", dt=dt, v_kph=v_ego_kph, lat_active=lat_active,
                  speed_gate=self._speed_gate, sharp_turn=is_sharp_turn,
                  model_curvature=model_curvature, yield_hold_timer=self._yield_hold_timer,
                  engage_ramp_timer=self._engage_ramp_timer, ramp_factor=ramp_factor,
                  weight=self.weight, poly_a=a, poly_b=b, poly_c=c,
                  lane_target_curv=lane_target_curvature,
                  path_std=path_std, pos_error=pos_error, yield_factor=yield_factor,
                  raw_correction=raw_correction, rate_limited_correction=rate_limited_correction,
                  correction=self.correction)
    
    return self.correction
