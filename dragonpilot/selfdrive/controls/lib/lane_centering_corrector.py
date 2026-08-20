#!/usr/bin/env python3
"""
Lane Centering Corrector (LCC) v2.6 - 台灣道路在地化終極特調版 
(混合信心度 + Soft Decay + 動態遲滯 + 漸進式曲率權重 + 控制路徑濾波)

將 10~50m 範圍內的左右車道線計算出多個中心點，並利用 np.polyfit 擬合出 
y = ax^2 + bx + c 的二次曲線。
v2.5 Pro 優化重點：
- 捨棄急彎硬切斷 (Hard Cutoff)，改為「漸進式曲率權重 (Progressive Curvature Weight)」。
- 彎道越急，LCC 修正權重越低，實現與 E2E 模型的無縫融合，徹底消滅門檻抖動，出彎秒接管！

v2.6 修正重點 (解決方向盤左右晃動)：
- 【核心 Bug】原本 lane_target_curvature / center_y_l 是用「每幀原始」的 polyfit
  係數 a,b,c 計算，UI_SMOOTH_TAU 平滑只套用在 self.poly_a/b/c 這組僅供繪圖用的
  屬性上，完全沒有真正進入控制路徑，導致車道線偵測雜訊直接放大成方向盤修正量。
  現在新增獨立的「控制路徑」平滑係數 self._ctrl_a/b/c，曲率與中心點計算改用這組。
- weight (車道線信心度) 與 curvature_weight (漸進式曲率權重) 原本逐幀重算、
  沒有時間濾波，機率/曲率在門檻附近抖動會直接反映到修正量大小，現在都各自加上
  低通濾波，行為更平滑，UI 繪圖用的平滑邏輯 (self.poly_a/b/c, UI_SMOOTH_TAU)
  維持不變。

v2.7 修正重點 (異常值限幅，補強低通濾波擋不住的單幀爆量雜訊)：
- 低通濾波 (CTRL_SMOOTH_TAU) 只能拖慢單幀離群值進入修正量的速度，沒辦法真正
  擋掉它；實測 log 曾出現單幀 poly_c 跳動超過 1m 的離群值，被濾波器拖慢後
  變成長達數秒的緩慢橫向拉扯，反而更明顯。
- 新增 _slew_limit()：在把當幀 a/b/c 餵進低通濾波器之前，先依照
  CTRL_MAX_STEP_*_PER_SEC 限制單幀最大允許變化量，離群值會被直接削掉，
  只有「持續存在」的真實變化才能在接下來幾幀逐步累積進來。
"""

import os
import time
import numpy as np

from openpilot.common.params import Params
from openpilot.common.filter_simple import FirstOrderFilter

PARAM_REFRESH_SEC = 2.0

# --- Debug log 設定 ---
ENABLE_CSV_LOG = True
LOG_PATH = "/data/media/0/realdata/lcc_debug.csv"
LOG_INTERVAL_SEC = 0.1  
LOG_COLUMNS = [
  "t", "dt", "v_kph", "state",
  "lat_active", "speed_gate", "model_curvature",
  "yield_hold_timer", "engage_ramp_timer", "ramp_factor", "curvature_weight",
  "weight", "poly_a", "poly_b", "poly_c", "lane_target_curv",
  "path_std", "pos_error", "yield_persist_timer", "yield_factor", "yield_suppression_pct",
  "raw_correction", "rate_limited_correction", "correction",
]

# --- 系統內建參數 ---
SPEED_ON_KPH = 20.0
SPEED_OFF_KPH = 10.0
KPH_TO_MS = 1000.0 / 3600.0

FIT_X = np.array([10.0, 20.0, 30.0, 40.0, 50.0])

# 恢復市區靈敏度，縮短前視距離與預測時間
MIN_LOOKAHEAD_M = 12.0
LOOKAHEAD_TIME_SEC = 0.9

# 兼顧快速啟動與過濾直線碎震，消除神經質晃動
FILTER_RC_SEC = 0.45

# --- 漸進式曲率權重 (取代舊版急彎硬切斷) ---
CURVATURE_WEIGHT_FULL = 0.020  # 曲率低於此值 (一般彎道/直線)：100% 修正
CURVATURE_WEIGHT_ZERO = 0.060  # 曲率高於此值 (大急彎)：0% 修正 (完全交給 E2E 模型)

PROB_MIN = 0.4
PROB_FULL = 0.6

LANE_WEIGHT_MAX = 0.90 
# 保護 EPS 馬達，堅守最大修正極限
MAX_CORRECTION = 0.012  
MAX_CORRECTION_RATE = 0.011

YIELD_CONFIRM_SEC = 0.15
ENGAGE_RAMP_SEC = 0.8  

SOFT_DISABLE_HOLD_SEC = 0.4

YIELD_MAX_PATH_STD = 0.35
YIELD_BREAK_IN_START = 0.35
YIELD_BREAK_IN_FULL = 0.85
DEFAULT_E2E_AUTHORITY = 1.0

# --- UI 視覺優化參數 ---
UI_SMOOTH_TAU = 0.2
UI_MIN_DRAW_WEIGHT = 0.4

# --- v2.6 控制路徑濾波參數 (實際餵進修正量計算，防止晃動) ---
CTRL_SMOOTH_TAU = 0.15          # 曲線係數 a,b,c 的控制路徑平滑
WEIGHT_SMOOTH_TAU = 0.15        # 車道線信心度 weight 的平滑
CURVATURE_WEIGHT_SMOOTH_TAU = 0.15  # 漸進式曲率權重的平滑
WEIGHT_ACTIVE_EPS = 1e-4        # 平滑後判斷 weight 是否視為 0 的容差

# --- v2.7 異常值限幅參數 (擋掉單幀離群值，避免低通濾波器把離群值拖成長晃動) ---
# 數值代表「每秒」最大允許變化量，實際限幅量 = 值 * dt (通常 dt=0.01s)
# 抓得比正常道路幾何變化率寬鬆一些，只用來擋真正的離群值，不影響正常追線反應
CTRL_MAX_STEP_A_PER_SEC = 0.01    # 曲率項 a 的最大變化率
CTRL_MAX_STEP_B_PER_SEC = 0.5     # 斜率項 b 的最大變化率
CTRL_MAX_STEP_C_PER_SEC = 3.0     # 偏移項 c 的最大變化率 (公尺/秒)


def _clip_interp(x, xp, fp):
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

    self._filter = FirstOrderFilter(0.0, FILTER_RC_SEC, 0.01)
    self.correction = 0.0
    
    self.poly_a = 0.0
    self.poly_b = 0.0
    self.poly_c = 0.0

    # v2.6: 控制路徑專用的平滑係數 (與 self.poly_a/b/c 的 UI 平滑分開)
    self._ctrl_a = 0.0
    self._ctrl_b = 0.0
    self._ctrl_c = 0.0

    self.weight = 0.0
    self._active = False
    self._speed_gate = False  
    self._yield_hold_timer = 0.0  
    self._engage_ramp_timer = 0.0  
    self._inactive_timer = 0.0
    self._yield_persist_timer = 0.0
    self.curvature_weight = 0.0

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
    self.poly_a = 0.0
    self.poly_b = 0.0
    self.poly_c = 0.0
    self._ctrl_a = 0.0
    self._ctrl_b = 0.0
    self._ctrl_c = 0.0
    self.weight = 0.0
    self._active = False
    self._speed_gate = False
    self._yield_hold_timer = 0.0
    self._engage_ramp_timer = 0.0
    self._inactive_timer = 0.0
    self._yield_persist_timer = 0.0
    self.curvature_weight = 0.0

  def _rate_limit(self, target: float, dt: float) -> float:
    max_delta = MAX_CORRECTION_RATE * dt
    return float(np.clip(target, self.correction - max_delta, self.correction + max_delta))

  def _smooth(self, target: float, current: float, tau: float, dt: float) -> float:
    alpha = dt / max(tau + dt, 1e-5)
    return (1.0 - alpha) * current + alpha * target

  @staticmethod
  def _slew_limit(target: float, current: float, max_rate_per_sec: float, dt: float) -> float:
    """限制單幀最大變化量，把離群值削到合理範圍內，再交給低通濾波器處理。"""
    max_delta = max_rate_per_sec * dt
    return float(np.clip(target, current - max_delta, current + max_delta))

  @staticmethod
  def _calc_curvature_weight(model_curvature: float) -> float:
    """計算漸進式曲率權重：彎道越急，LCC 介入比例越低"""
    c = abs(float(model_curvature))
    if c <= CURVATURE_WEIGHT_FULL:
      return 1.0
    if c >= CURVATURE_WEIGHT_ZERO:
      return 0.0
    return float(np.clip(
      1.0 - ((c - CURVATURE_WEIGHT_FULL) / (CURVATURE_WEIGHT_ZERO - CURVATURE_WEIGHT_FULL)),
      0.0, 1.0
    ))

  def _decay(self, dt: float) -> float:
    limited = self._rate_limit(0.0, dt)
    return self._filter.update(limited)

  def _lookup_path_std_and_error(self, model_v2, center_y_l: float, l: float) -> tuple[float, float]:
    try:
      pos_x = np.asarray(model_v2.position.x, dtype=float)
      pos_y = np.asarray(model_v2.position.y, dtype=float)
      pos_y_std = np.asarray(model_v2.position.yStd, dtype=float)

      if pos_x.size < 2 or pos_x.size != pos_y.size or pos_x.size != pos_y_std.size:
        return -1.0, 0.0
      if not (np.isfinite(pos_x).all() and np.isfinite(pos_y).all() and np.isfinite(pos_y_std).all()):
        return -1.0, 0.0
      if not np.all(np.diff(pos_x) > 0):
        return -1.0, 0.0
      if l < pos_x[0] or l > pos_x[-1]:
        return -1.0, 0.0

      model_y = float(np.interp(l, pos_x, pos_y))
      path_std = float(np.interp(l, pos_x, pos_y_std))
      pos_error = center_y_l - model_y
      return path_std, pos_error
    except (AttributeError, TypeError, ValueError, IndexError):
      return -1.0, 0.0

  def _yield_factor(self, model_v2, center_y_l: float, l: float, e2e_authority: float, dt: float, v_ego: float) -> tuple[float, float, float, float]:
    path_std, pos_error = self._lookup_path_std_and_error(model_v2, center_y_l, l)
    error_abs = abs(pos_error)
    std_valid = 0.0 <= path_std <= YIELD_MAX_PATH_STD

    if std_valid:
      if error_abs > YIELD_BREAK_IN_START:
        self._yield_persist_timer += dt
      elif error_abs < (YIELD_BREAK_IN_START - 0.15):
        self._yield_persist_timer = 0.0
    else:
      self._yield_persist_timer = 0.0

    dynamic_persist_sec = max(0.15, 0.5 - (v_ego / 30.0) * 0.35)

    if not std_valid or self._yield_persist_timer < dynamic_persist_sec:
      return 1.0, path_std, pos_error, self._yield_persist_timer

    break_in = float(np.clip(
      (error_abs - YIELD_BREAK_IN_START) / (YIELD_BREAK_IN_FULL - YIELD_BREAK_IN_START),
      0.0, 1.0,
    ))
    yield_factor = 1.0 - float(np.clip(e2e_authority, 0.0, 1.0)) * break_in
    return yield_factor, path_std, pos_error, self._yield_persist_timer

  def _log_row(self, state: str, **fields) -> None:
    if not ENABLE_CSV_LOG:
      return
    
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

  def _log_fallback(self, state: str, dt, v_ego_kph, lat_active, model_curvature, ramp_factor):
    if not ENABLE_CSV_LOG:
      return
    self._log_row(state, dt=dt, v_kph=v_ego_kph, lat_active=lat_active,
                  speed_gate=self._speed_gate, model_curvature=model_curvature, 
                  yield_hold_timer=self._yield_hold_timer, engage_ramp_timer=self._engage_ramp_timer, 
                  ramp_factor=ramp_factor, curvature_weight=self.curvature_weight,
                  weight=0.0, poly_a=0.0, poly_b=0.0, poly_c=0.0, 
                  lane_target_curv=0.0, path_std=-1.0, pos_error=0.0,
                  yield_persist_timer=self._yield_persist_timer, yield_factor=1.0, yield_suppression_pct=0.0,
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
    raw_curvature_weight = self._calc_curvature_weight(model_curvature)
    # v2.6: 對曲率權重做低通濾波，避免在 FULL/ZERO 門檻附近抖動時修正量忽大忽小
    self.curvature_weight = self._smooth(raw_curvature_weight, self.curvature_weight, CURVATURE_WEIGHT_SMOOTH_TAU, dt)

    # --- 硬性停用 ---
    hard_invalid = (not self._enabled or not lat_active)
    if hard_invalid:
      _state = "DISABLED" if not self._enabled else "NOT_LAT_ACTIVE"
      self.reset()
      self._log_fallback(_state, dt, v_ego_kph, lat_active, model_curvature, 0.0)
      return 0.0

    # --- 暫時性放手 (Soft Invalid)：現在只受車速控制 ---
    soft_invalid = not self._speed_gate
    if soft_invalid:
      self._inactive_timer += dt
      if self._inactive_timer >= SOFT_DISABLE_HOLD_SEC:
        self._engage_ramp_timer = 0.0
      
      self._active = False
      self.weight = 0.0
      self.poly_a = self.poly_b = self.poly_c = 0.0
      self.correction = self._decay(dt)
      self._log_fallback("SPEED_GATE_OFF", dt, v_ego_kph, lat_active, model_curvature,
                         float(np.clip(self._engage_ramp_timer / ENGAGE_RAMP_SEC, 0.0, 1.0)))
      return self.correction

    self._inactive_timer = 0.0
    self._engage_ramp_timer += dt
    ramp_factor = float(np.clip(self._engage_ramp_timer / ENGAGE_RAMP_SEC, 0.0, 1.0))

    # 必須有方向燈且轉動方向盤才退讓
    raw_yield_condition = (left_blinker or right_blinker) and steering_pressed
    if raw_yield_condition:
      self._yield_hold_timer += dt
    else:
      self._yield_hold_timer = 0.0
    is_yielding = self._yield_hold_timer >= YIELD_CONFIRM_SEC

    if is_yielding:
      self._active = False
      self.weight = 0.0
      self.poly_a = self.poly_b = self.poly_c = 0.0
      self.correction = self._decay(dt)
      self._log_fallback("YIELDING", dt, v_ego_kph, lat_active, model_curvature, ramp_factor)
      return self.correction

    lane_lines = model_v2.laneLines
    lane_line_probs = model_v2.laneLineProbs

    # --- NO_LANE_DATA 改為 Soft Decay 平滑退場 ---
    if len(lane_lines) < 3 or len(lane_line_probs) < 3:
      self._active = False
      self.weight = 0.0
      self.poly_a = self.poly_b = self.poly_c = 0.0
      self.correction = self._decay(dt)
      self._log_fallback("NO_LANE_DATA", dt, v_ego_kph, lat_active, model_curvature, ramp_factor)
      return self.correction

    # --- 混合信心度算法 (平均值 + 單邊可靠性限制) ---
    lll_prob = float(np.clip(lane_line_probs[1], 0.0, 1.0))
    rll_prob = float(np.clip(lane_line_probs[2], 0.0, 1.0))
    
    mean_prob = 0.5 * (lll_prob + rll_prob)
    min_prob = min(lll_prob, rll_prob)
    
    confidence_mean = float(np.clip((mean_prob - PROB_MIN) / (PROB_FULL - PROB_MIN), 0.0, 1.0))
    single_side_factor = float(np.clip(min_prob / PROB_MIN, 0.0, 1.0))
    
    confidence = confidence_mean * (0.65 + 0.35 * single_side_factor)
    raw_weight = confidence * LANE_WEIGHT_MAX
    # v2.6: 對信心度做低通濾波，避免車道線機率在門檻附近抖動時修正量忽大忽小
    self.weight = self._smooth(raw_weight, self.weight, WEIGHT_SMOOTH_TAU, dt)

    if self.weight <= WEIGHT_ACTIVE_EPS:
      self._active = False
      self.poly_a = self.poly_b = self.poly_c = 0.0
      self.correction = self._decay(dt)
      self._log_fallback("LOW_CONFIDENCE", dt, v_ego_kph, lat_active, model_curvature, ramp_factor)
      return self.correction

    lll = lane_lines[1]
    rll = lane_lines[2]
    
    valid_x = []
    center_y = []
    for x in FIT_X:
      l_y = _clip_interp(x, lll.x, lll.y)
      r_y = _clip_interp(x, rll.x, rll.y)
      if l_y is not None and r_y is not None:
        valid_x.append(x)
        center_y.append((l_y + r_y) / 2.0)

    if len(valid_x) < 3:
      self._active = False
      self.poly_a = self.poly_b = self.poly_c = 0.0
      self.correction = self._decay(dt)
      self._log_fallback("INTERP_FAIL", dt, v_ego_kph, lat_active, model_curvature, ramp_factor)
      return self.correction

    try:
      coeffs = np.polyfit(valid_x, center_y, 2)
      if len(coeffs) != 3 or not np.isfinite(coeffs).all():
        raise ValueError("invalid lane polynomial")
      a, b, c = (float(coeffs[0]), float(coeffs[1]), float(coeffs[2]))
    except (TypeError, ValueError, np.linalg.LinAlgError):
      self._active = False
      self.weight = 0.0
      self.poly_a = self.poly_b = self.poly_c = 0.0
      self.correction = self._decay(dt)
      self._log_fallback("POLYFIT_FAIL", dt, v_ego_kph, lat_active, model_curvature, ramp_factor)
      return self.correction
    
    # --- UI 平滑過濾 (僅供畫面顯示用，不影響實際修正量計算) ---
    if self.weight >= UI_MIN_DRAW_WEIGHT:
      if abs(self.poly_c) < 1e-7:  
        self.poly_a = float(a)
        self.poly_b = float(b)
        self.poly_c = float(c)
      else:
        self.poly_a = self._smooth(float(a), self.poly_a, UI_SMOOTH_TAU, dt)
        self.poly_b = self._smooth(float(b), self.poly_b, UI_SMOOTH_TAU, dt)
        self.poly_c = self._smooth(float(c), self.poly_c, UI_SMOOTH_TAU, dt)
    else:
      self.poly_a = 0.0
      self.poly_b = 0.0
      self.poly_c = 0.0

    # --- v2.6/v2.7 控制路徑平滑：這組才是真正拿去算修正量的係數 ---
    # 每次有效擬合都會更新，不受 UI_MIN_DRAW_WEIGHT 門檻影響，避免車道線雜訊
    # 直接放大成方向盤修正量。冷啟動 (係數全為 0) 時直接採用當幀值，之後才平滑。
    if self._ctrl_a == 0.0 and self._ctrl_b == 0.0 and abs(self._ctrl_c) < 1e-7:
      self._ctrl_a, self._ctrl_b, self._ctrl_c = float(a), float(b), float(c)
    else:
      # v2.7: 先限幅擋掉單幀離群值 (例如車道線暫時鎖錯造成的瞬間 1m+ 跳動)，
      # 再交給低通濾波器處理正常雜訊，兩層濾波各司其職。
      a_limited = self._slew_limit(float(a), self._ctrl_a, CTRL_MAX_STEP_A_PER_SEC, dt)
      b_limited = self._slew_limit(float(b), self._ctrl_b, CTRL_MAX_STEP_B_PER_SEC, dt)
      c_limited = self._slew_limit(float(c), self._ctrl_c, CTRL_MAX_STEP_C_PER_SEC, dt)
      self._ctrl_a = self._smooth(a_limited, self._ctrl_a, CTRL_SMOOTH_TAU, dt)
      self._ctrl_b = self._smooth(b_limited, self._ctrl_b, CTRL_SMOOTH_TAU, dt)
      self._ctrl_c = self._smooth(c_limited, self._ctrl_c, CTRL_SMOOTH_TAU, dt)

    L = max(MIN_LOOKAHEAD_M, v_ego * LOOKAHEAD_TIME_SEC)
    lane_target_curvature = (2.0 * self._ctrl_a) + (2.0 * self._ctrl_b / L) + (2.0 * self._ctrl_c / (L ** 2))
    center_y_l = self._ctrl_a * (L ** 2) + self._ctrl_b * L + self._ctrl_c

    if not (np.isfinite(L) and np.isfinite(lane_target_curvature) and np.isfinite(center_y_l)):
      self._active = False
      self.weight = 0.0
      self.poly_a = self.poly_b = self.poly_c = 0.0
      self.correction = self._decay(dt)
      self._log_fallback("CURVATURE_FAIL", dt, v_ego_kph, lat_active, model_curvature, ramp_factor)
      return self.correction

    yield_factor, path_std, pos_error, yield_persist_timer = self._yield_factor(model_v2, center_y_l, L, e2e_authority, dt, v_ego)

    curvature_error = lane_target_curvature - model_curvature
    if not np.isfinite(curvature_error):
      self._active = False
      self.weight = 0.0
      self.poly_a = self.poly_b = self.poly_c = 0.0
      self.correction = self._decay(dt)
      self._log_fallback("CURVATURE_ERROR_FAIL", dt, v_ego_kph, lat_active, model_curvature, ramp_factor)
      return self.correction

    # --- v2.5 最終連乘公式：加入 curvature_weight 漸進控制 ---
    raw_correction = self.weight * ramp_factor * yield_factor * self.curvature_weight * curvature_error
    raw_correction = float(np.clip(raw_correction, -MAX_CORRECTION, MAX_CORRECTION))

    rate_limited_correction = self._rate_limit(raw_correction, dt)
    self.correction = self._filter.update(rate_limited_correction)
    self._active = True

    if ENABLE_CSV_LOG:
      self._log_row("ACTIVE", dt=dt, v_kph=v_ego_kph, lat_active=lat_active,
                    speed_gate=self._speed_gate, model_curvature=model_curvature,
                    yield_hold_timer=self._yield_hold_timer,
                    engage_ramp_timer=self._engage_ramp_timer, ramp_factor=ramp_factor,
                    curvature_weight=self.curvature_weight,
                    weight=self.weight, poly_a=a, poly_b=b, poly_c=c,
                    lane_target_curv=lane_target_curvature,
                    path_std=path_std, pos_error=pos_error,
                    yield_persist_timer=yield_persist_timer, yield_factor=yield_factor,
                    yield_suppression_pct=(1.0 - yield_factor) * 100.0,
                    raw_correction=raw_correction, rate_limited_correction=rate_limited_correction,
                    correction=self.correction)
    
    return self.correction
