#!/usr/bin/env python3
"""
Lane Centering Corrector (LCC) v2.3 - 台灣道路在地化特調版 
(混合信心度演算法 + 快速重啟 + Soft Decay + E2E Yield + 收斂延遲修正)

將 10~50m 範圍內的左右車道線計算出多個中心點，並利用 np.polyfit 擬合出 
y = ax^2 + bx + c 的二次曲線。藉此精準分離「道路曲率(a)」、「航向誤差(b)」與「橫向偏移(c)」。
針對台灣複雜路況優化：
1. 引入「平均信心度 + 單邊可靠性限制」，處理單側標線不清或斑駁的問題。
2. NO_LANE_DATA 改為平滑退場 (Soft Decay) 並保留 Ramp，適應無標線大路口。
3. 縮短 ENGAGE_RAMP_SEC 至 0.8s，提升閃避機車/障礙物後的恢復敏捷度。
4. v2.3: 修正「回中 7-8 秒」問題 —— yield 機制過去在偏移量最大、最需要修正的瞬間
   反而壓制修正力道，形成自我拖延迴圈。改為加入 persistence gate (需持續偏移
   一段時間才啟動 yield)，並放寬 break-in 區間；同時放寬 rate limiter 與濾波
   時間常數，縮短「軟體造成」的啟動延遲。
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
  "lat_active", "speed_gate", "sharp_turn", "model_curvature",
  "yield_hold_timer", "engage_ramp_timer", "ramp_factor",
  "weight", "poly_a", "poly_b", "poly_c", "lane_target_curv",
  "path_std", "pos_error", "yield_persist_timer", "yield_factor", "yield_suppression_pct",
  "raw_correction", "rate_limited_correction", "correction",
]

# --- 系統內建參數 ---
SPEED_ON_KPH = 20.0
SPEED_OFF_KPH = 10.0
KPH_TO_MS = 1000.0 / 3600.0

FIT_X = np.array([10.0, 20.0, 30.0, 40.0, 50.0])

MIN_LOOKAHEAD_M = 15.0
LOOKAHEAD_TIME_SEC = 1.2

# v2.3: RC 0.5s -> 0.35s，縮短低通濾波造成的啟動延遲 (約省 0.5~0.7s)。
# 仍保留一定平滑量以避免座標系旋轉造成的自激振盪，如實測有抖動請調回 0.4~0.5。
FILTER_RC_SEC = 0.35

# --- 急彎判定：Hysteresis (遲滯) 機制 ---
SHARP_TURN_CURVATURE_ENTER = 0.06
SHARP_TURN_CURVATURE_EXIT = 0.05

PROB_MIN = 0.4
PROB_FULL = 0.6

LANE_WEIGHT_MAX = 0.90 
MAX_CORRECTION = 0.012  
# v2.3: 0.008 -> 0.011，rate limiter 從 0 爬到 MAX_CORRECTION 的時間從 1.5s 縮短到 ~1.1s。
MAX_CORRECTION_RATE = 0.011

YIELD_CONFIRM_SEC = 0.15
ENGAGE_RAMP_SEC = 0.8  # 縮短至 0.8 秒，提升重新介入的敏捷度

SOFT_DISABLE_HOLD_SEC = 0.4

YIELD_MAX_PATH_STD = 0.35
# v2.3: 放寬 break-in 區間 (0.20~0.60 -> 0.35~0.85)，一般車道內小偏移不再被 yield 壓制，
# 只有真正跟模型路徑分歧很大時才退讓。
YIELD_BREAK_IN_START = 0.35
YIELD_BREAK_IN_FULL = 0.85
# v2.3: 新增 persistence gate —— 偏移量要持續超過 YIELD_BREAK_IN_START 這麼久，
# 才真正啟動 yield 壓制。避免在偏移剛發生、最需要修正力道的瞬間反而被壓制，
# 這是造成「回中 7-8 秒」的主因（yield 在偏移最大時抑制修正 -> 收斂變慢 ->
# 偏移持續偏大 -> 繼續被壓制，形成自我拖延迴圈）。
YIELD_PERSIST_SEC = 0.5
DEFAULT_E2E_AUTHORITY = 1.0

# --- UI 視覺優化參數 ---
UI_SMOOTH_TAU = 0.2
UI_MIN_DRAW_WEIGHT = 0.4


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
    
    self.weight = 0.0
    self._active = False
    self._speed_gate = False  
    self._yield_hold_timer = 0.0  
    self._engage_ramp_timer = 0.0  
    self._sharp_turn_latched = False
    self._inactive_timer = 0.0
    self._yield_persist_timer = 0.0

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
    self.weight = 0.0
    self._active = False
    self._speed_gate = False
    self._yield_hold_timer = 0.0
    self._engage_ramp_timer = 0.0
    self._sharp_turn_latched = False
    self._inactive_timer = 0.0
    self._yield_persist_timer = 0.0

  def _rate_limit(self, target: float, dt: float) -> float:
    max_delta = MAX_CORRECTION_RATE * dt
    return float(np.clip(target, self.correction - max_delta, self.correction + max_delta))

  def _smooth(self, target: float, current: float, tau: float, dt: float) -> float:
    alpha = dt / max(tau + dt, 1e-5)
    return (1.0 - alpha) * current + alpha * target

  def _update_sharp_turn(self, model_curvature: float) -> bool:
    c = abs(model_curvature)
    if self._sharp_turn_latched:
      if c < SHARP_TURN_CURVATURE_EXIT:
        self._sharp_turn_latched = False
    else:
      if c > SHARP_TURN_CURVATURE_ENTER:
        self._sharp_turn_latched = True
    return self._sharp_turn_latched

  def _decay(self, dt: float) -> float:
    limited = self._rate_limit(0.0, dt)
    return self._filter.update(limited)

  def _lookup_path_std_and_error(self, model_v2, center_y_l: float, l: float) -> tuple[float, float]:
    """僅負責從 model_v2.position 內插出 path_std 與 pos_error，不含 yield 判斷邏輯。"""
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

  def _yield_factor(self, model_v2, center_y_l: float, l: float, e2e_authority: float, dt: float) -> tuple[float, float, float, float]:
    """
    v2.3: 加入 persistence gate。偏移量必須持續超過 YIELD_BREAK_IN_START 達
    YIELD_PERSIST_SEC 秒，才真正啟動 yield 壓制。這避免了偏移剛發生、最需要
    修正力道的當下就被 yield 壓制，導致收斂緩慢的自我拖延迴圈。
    回傳: (yield_factor, path_std, pos_error, yield_persist_timer)
    """
    path_std, pos_error = self._lookup_path_std_and_error(model_v2, center_y_l, l)
    error_abs = abs(pos_error)
    std_valid = 0.0 <= path_std <= YIELD_MAX_PATH_STD

    if std_valid and error_abs > YIELD_BREAK_IN_START:
      self._yield_persist_timer += dt
    else:
      self._yield_persist_timer = 0.0

    if not std_valid or self._yield_persist_timer < YIELD_PERSIST_SEC:
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

  def _log_fallback(self, state: str, dt, v_ego_kph, lat_active, is_sharp_turn, model_curvature, ramp_factor):
    if not ENABLE_CSV_LOG:
      return
    self._log_row(state, dt=dt, v_kph=v_ego_kph, lat_active=lat_active,
                  speed_gate=self._speed_gate, sharp_turn=is_sharp_turn,
                  model_curvature=model_curvature, yield_hold_timer=self._yield_hold_timer,
                  engage_ramp_timer=self._engage_ramp_timer, ramp_factor=ramp_factor,
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

    # --- 硬性停用 ---
    hard_invalid = (not self._enabled or not lat_active)
    if hard_invalid:
      _state = "DISABLED" if not self._enabled else "NOT_LAT_ACTIVE"
      self.reset()
      self._log_fallback(_state, dt, v_ego_kph, lat_active, False, model_curvature, 0.0)
      return 0.0

    is_sharp_turn = self._update_sharp_turn(model_curvature)

    # --- 暫時性放手 (Soft Invalid) ---
    soft_invalid = (not self._speed_gate) or is_sharp_turn
    if soft_invalid:
      _state = "SPEED_GATE_OFF" if not self._speed_gate else "SHARP_TURN"

      if not self._speed_gate:
        self._inactive_timer += dt
        if self._inactive_timer >= SOFT_DISABLE_HOLD_SEC:
          self._engage_ramp_timer = 0.0
      else:
        self._inactive_timer = 0.0

      self._active = False
      self.weight = 0.0
      self.poly_a = self.poly_b = self.poly_c = 0.0
      self.correction = self._decay(dt)
      self._log_fallback(_state, dt, v_ego_kph, lat_active, is_sharp_turn, model_curvature,
                         float(np.clip(self._engage_ramp_timer / ENGAGE_RAMP_SEC, 0.0, 1.0)))
      return self.correction

    self._inactive_timer = 0.0
    self._engage_ramp_timer += dt
    ramp_factor = float(np.clip(self._engage_ramp_timer / ENGAGE_RAMP_SEC, 0.0, 1.0))

    # 嚴格還原：必須有方向燈且轉動方向盤才退讓
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
      self._log_fallback("YIELDING", dt, v_ego_kph, lat_active, is_sharp_turn, model_curvature, ramp_factor)
      return self.correction

    lane_lines = model_v2.laneLines
    lane_line_probs = model_v2.laneLineProbs

    # --- NO_LANE_DATA 改為 Soft Decay 平滑退場 ---
    if len(lane_lines) < 3 or len(lane_line_probs) < 3:
      self._active = False
      self.weight = 0.0
      self.poly_a = self.poly_b = self.poly_c = 0.0
      self.correction = self._decay(dt)
      self._log_fallback("NO_LANE_DATA", dt, v_ego_kph, lat_active, is_sharp_turn, model_curvature, ramp_factor)
      return self.correction

    # --- 混合信心度算法 (平均值 + 單邊可靠性限制) ---
    lll_prob = float(np.clip(lane_line_probs[1], 0.0, 1.0))
    rll_prob = float(np.clip(lane_line_probs[2], 0.0, 1.0))
    
    mean_prob = 0.5 * (lll_prob + rll_prob)
    min_prob = min(lll_prob, rll_prob)
    
    confidence_mean = float(np.clip((mean_prob - PROB_MIN) / (PROB_FULL - PROB_MIN), 0.0, 1.0))
    single_side_factor = float(np.clip(min_prob / PROB_MIN, 0.0, 1.0))
    
    confidence = confidence_mean * (0.65 + 0.35 * single_side_factor)
    self.weight = confidence * LANE_WEIGHT_MAX

    if self.weight <= 0.0:
      self._active = False
      self.poly_a = self.poly_b = self.poly_c = 0.0
      self.correction = self._decay(dt)
      self._log_fallback("LOW_CONFIDENCE", dt, v_ego_kph, lat_active, is_sharp_turn, model_curvature, ramp_factor)
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
      self._log_fallback("INTERP_FAIL", dt, v_ego_kph, lat_active, is_sharp_turn, model_curvature, ramp_factor)
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
      self._log_fallback("POLYFIT_FAIL", dt, v_ego_kph, lat_active, is_sharp_turn, model_curvature, ramp_factor)
      return self.correction
    
    # --- UI 平滑過濾 ---
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

    L = max(MIN_LOOKAHEAD_M, v_ego * LOOKAHEAD_TIME_SEC)
    lane_target_curvature = (2.0 * a) + (2.0 * b / L) + (2.0 * c / (L ** 2))
    center_y_l = a * (L ** 2) + b * L + c

    if not (np.isfinite(L) and np.isfinite(lane_target_curvature) and np.isfinite(center_y_l)):
      self._active = False
      self.weight = 0.0
      self.poly_a = self.poly_b = self.poly_c = 0.0
      self.correction = self._decay(dt)
      self._log_fallback("CURVATURE_FAIL", dt, v_ego_kph, lat_active, is_sharp_turn, model_curvature, ramp_factor)
      return self.correction

    yield_factor, path_std, pos_error, yield_persist_timer = self._yield_factor(model_v2, center_y_l, L, e2e_authority, dt)

    curvature_error = lane_target_curvature - model_curvature
    if not np.isfinite(curvature_error):
      self._active = False
      self.weight = 0.0
      self.poly_a = self.poly_b = self.poly_c = 0.0
      self.correction = self._decay(dt)
      self._log_fallback("CURVATURE_ERROR_FAIL", dt, v_ego_kph, lat_active, is_sharp_turn, model_curvature, ramp_factor)
      return self.correction

    raw_correction = self.weight * ramp_factor * yield_factor * curvature_error
    raw_correction = float(np.clip(raw_correction, -MAX_CORRECTION, MAX_CORRECTION))

    rate_limited_correction = self._rate_limit(raw_correction, dt)
    self.correction = self._filter.update(rate_limited_correction)
    self._active = True

    if ENABLE_CSV_LOG:
      self._log_row("ACTIVE", dt=dt, v_kph=v_ego_kph, lat_active=lat_active,
                    speed_gate=self._speed_gate, sharp_turn=is_sharp_turn,
                    model_curvature=model_curvature, yield_hold_timer=self._yield_hold_timer,
                    engage_ramp_timer=self._engage_ramp_timer, ramp_factor=ramp_factor,
                    weight=self.weight, poly_a=a, poly_b=b, poly_c=c,
                    lane_target_curv=lane_target_curvature,
                    path_std=path_std, pos_error=pos_error,
                    yield_persist_timer=yield_persist_timer, yield_factor=yield_factor,
                    yield_suppression_pct=(1.0 - yield_factor) * 100.0,
                    raw_correction=raw_correction, rate_limited_correction=rate_limited_correction,
                    correction=self.correction)
    
    return self.correction
