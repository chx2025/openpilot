#!/usr/bin/env python3
"""
Lane Centering Corrector (LCC) v2.11 - 台灣道路在地化終極特調版 
(混合信心度 + Soft Decay + 動態遲滯 + 漸進式曲率權重 + 控制路徑濾波 + 動態前視 + 死區過濾 + 晃動偵測)

將 10~50m 範圍內的左右車道線計算出多個中心點，並利用 np.polyfit 擬合出 
y = ax^2 + bx + c 的二次曲線。

v2.10 新增晃動偵測 (Wobble Detection) 欄位：
- 新增 `curvature_error` (原始曲率誤差，未經死區處理)
- 新增 `delta_correction` (與前一幀的修正量差值)
- 新增 `wobble_rate` (每秒修正量變化率，數值越高代表方向盤抽動越劇烈)

v2.11 修改 LOG 記錄頻率：
- 將 LOG_INTERVAL_SEC 從 0.1 秒改為 0.5 秒，減少檔案大小。

v2.12 新增車道寬度合理性檢查 (解決市區持續性單向偏移)：
- 市區路口的停止線、斑馬線、路邊停車常常讓多點插值抓到不該抓的點，
  擬合出的中心線會在好幾秒內持續偏向同一邊——這不是雜訊，是真實資料被污染，
  之前的濾波器/限幅/死區都沒辦法擋（因為它們只擋瞬間離群值，這種是持續性訊號）。
- 新增 LANE_WIDTH_MIN/MAX：每個插值點都額外檢查左右線距離是否落在合理車道寬度
  範圍內，不合理的點直接捨棄，不納入 polyfit。捨棄後有效點 < 3 才會判定失敗，
  並新增 LANE_WIDTH_FAIL 狀態方便從 log 分辨是插值失敗還是寬度不合理被擋。
- ACTIVE 時額外記錄 avg_lane_width 到 log，方便之後依實際道路寬度分布微調門檻。
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
# v2.11: 將記錄間隔改為 0.1 秒
LOG_INTERVAL_SEC = 0.1  
LOG_COLUMNS = [
  "t", "dt", "v_kph", "state",
  "lat_active", "speed_gate", "model_curvature",
  "yield_hold_timer", "engage_ramp_timer", "ramp_factor", "curvature_weight",
  "weight", "poly_a", "poly_b", "poly_c", "lane_target_curv",
  "path_std", "pos_error", "yield_persist_timer", "yield_factor", "yield_suppression_pct",
  "curvature_error", "raw_correction", "rate_limited_correction", "correction",
  "delta_correction", "wobble_rate", "avg_lane_width"
]

# --- 系統內建參數 ---
SPEED_ON_KPH = 20.0
SPEED_OFF_KPH = 10.0
KPH_TO_MS = 1000.0 / 3600.0

MIN_LOOKAHEAD_M = 12.0
LOOKAHEAD_TIME_SEC = 0.9
FILTER_RC_SEC = 0.45

CURVATURE_WEIGHT_FULL = 0.020  
CURVATURE_WEIGHT_ZERO = 0.060  

PROB_MIN = 0.4
PROB_FULL = 0.6

LANE_WEIGHT_MAX = 0.90 
MAX_CORRECTION = 0.012  
MAX_CORRECTION_RATE = 0.011

# --- v2.9 新增：曲率誤差死區參數 ---
CURVATURE_DEADBAND = 0.0004

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

# --- v2.6 控制路徑濾波參數 ---
CTRL_SMOOTH_TAU = 0.15          
WEIGHT_SMOOTH_TAU = 0.15        
CURVATURE_WEIGHT_SMOOTH_TAU = 0.15  
WEIGHT_ACTIVE_EPS = 1e-4        

# --- v2.7 異常值限幅參數 ---
CTRL_MAX_STEP_A_PER_SEC = 0.01    
CTRL_MAX_STEP_B_PER_SEC = 0.5     
CTRL_MAX_STEP_C_PER_SEC = 3.0     

# --- v2.12 車道寬度合理性檢查 (公尺)：擋掉被斑馬線/停止線/路邊停車污染的插值點 ---
# 台灣道路實際寬度落差較大，這組數字是保守起點，建議依 log 的 avg_lane_width
# 實際分布再微調，不要一開始就抓太緊，避免正常窄巷道路被誤判失敗
LANE_WIDTH_MIN = 2.5
LANE_WIDTH_MAX = 4.2


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
    self._last_correction = 0.0  
    
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
    self._last_correction = 0.0
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
    max_delta = max_rate_per_sec * dt
    return float(np.clip(target, current - max_delta, current + max_delta))

  @staticmethod
  def _calc_curvature_weight(model_curvature: float) -> float:
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
    
    # 狀態改變或時間間隔大於 0.5 秒才記錄
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
    
    delta_correction = self.correction - self._last_correction
    wobble_rate = abs(delta_correction) / max(dt, 1e-5)
    self._last_correction = self.correction

    self._log_row(state, dt=dt, v_kph=v_ego_kph, lat_active=lat_active,
                  speed_gate=self._speed_gate, model_curvature=model_curvature, 
                  yield_hold_timer=self._yield_hold_timer, engage_ramp_timer=self._engage_ramp_timer, 
                  ramp_factor=ramp_factor, curvature_weight=self.curvature_weight,
                  weight=0.0, poly_a=0.0, poly_b=0.0, poly_c=0.0, 
                  lane_target_curv=0.0, path_std=-1.0, pos_error=0.0,
                  yield_persist_timer=self._yield_persist_timer, yield_factor=1.0, yield_suppression_pct=0.0,
                  curvature_error=0.0, raw_correction=0.0, rate_limited_correction=0.0, correction=self.correction,
                  delta_correction=delta_correction, wobble_rate=wobble_rate)

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
    self.curvature_weight = self._smooth(raw_curvature_weight, self.curvature_weight, CURVATURE_WEIGHT_SMOOTH_TAU, dt)

    hard_invalid = (not self._enabled or not lat_active)
    if hard_invalid:
      _state = "DISABLED" if not self._enabled else "NOT_LAT_ACTIVE"
      self.reset()
      self._log_fallback(_state, dt, v_ego_kph, lat_active, model_curvature, 0.0)
      return 0.0

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

    if len(lane_lines) < 3 or len(lane_line_probs) < 3:
      self._active = False
      self.weight = 0.0
      self.poly_a = self.poly_b = self.poly_c = 0.0
      self.correction = self._decay(dt)
      self._log_fallback("NO_LANE_DATA", dt, v_ego_kph, lat_active, model_curvature, ramp_factor)
      return self.correction

    lll_prob = float(np.clip(lane_line_probs[1], 0.0, 1.0))
    rll_prob = float(np.clip(lane_line_probs[2], 0.0, 1.0))
    
    mean_prob = 0.5 * (lll_prob + rll_prob)
    min_prob = min(lll_prob, rll_prob)
    
    confidence_mean = float(np.clip((mean_prob - PROB_MIN) / (PROB_FULL - PROB_MIN), 0.0, 1.0))
    single_side_factor = float(np.clip(min_prob / PROB_MIN, 0.0, 1.0))
    
    confidence = confidence_mean * (0.65 + 0.35 * single_side_factor)
    raw_weight = confidence * LANE_WEIGHT_MAX
    self.weight = self._smooth(raw_weight, self.weight, WEIGHT_SMOOTH_TAU, dt)

    if self.weight <= WEIGHT_ACTIVE_EPS:
      self._active = False
      self.poly_a = self.poly_b = self.poly_c = 0.0
      self.correction = self._decay(dt)
      self._log_fallback("LOW_CONFIDENCE", dt, v_ego_kph, lat_active, model_curvature, ramp_factor)
      return self.correction

    lll = lane_lines[1]
    rll = lane_lines[2]
    
    max_fit_dist = float(np.clip(v_ego * 2.5, 20.0, 60.0))
    dynamic_fit_x = np.linspace(10.0, max_fit_dist, 5)

    valid_x = []
    center_y = []
    widths = []
    width_reject_count = 0
    for x in dynamic_fit_x:
      l_y = _clip_interp(x, lll.x, lll.y)
      r_y = _clip_interp(x, rll.x, rll.y)
      if l_y is None or r_y is None:
        continue
      width = abs(l_y - r_y)
      if width < LANE_WIDTH_MIN or width > LANE_WIDTH_MAX:
        # 寬度不合理，通常是插到斑馬線/停止線/路邊停車，直接捨棄這個點
        width_reject_count += 1
        continue
      valid_x.append(x)
      widths.append(width)
      center_y.append((l_y + r_y) / 2.0)

    avg_lane_width = float(np.mean(widths)) if widths else -1.0

    if len(valid_x) < 3:
      self._active = False
      self.poly_a = self.poly_b = self.poly_c = 0.0
      self.correction = self._decay(dt)
      _fail_state = "LANE_WIDTH_FAIL" if width_reject_count > 0 else "INTERP_FAIL"
      self._log_fallback(_fail_state, dt, v_ego_kph, lat_active, model_curvature, ramp_factor)
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

    if self._ctrl_a == 0.0 and self._ctrl_b == 0.0 and abs(self._ctrl_c) < 1e-7:
      self._ctrl_a, self._ctrl_b, self._ctrl_c = float(a), float(b), float(c)
    else:
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

    raw_curvature_error = lane_target_curvature - model_curvature
    if not np.isfinite(raw_curvature_error):
      self._active = False
      self.weight = 0.0
      self.poly_a = self.poly_b = self.poly_c = 0.0
      self.correction = self._decay(dt)
      self._log_fallback("CURVATURE_ERROR_FAIL", dt, v_ego_kph, lat_active, model_curvature, ramp_factor)
      return self.correction

    curvature_error_abs = abs(raw_curvature_error)
    if curvature_error_abs <= CURVATURE_DEADBAND:
      curvature_error = 0.0
    else:
      curvature_error = np.copysign(curvature_error_abs - CURVATURE_DEADBAND, raw_curvature_error)

    raw_correction = self.weight * ramp_factor * yield_factor * self.curvature_weight * curvature_error
    raw_correction = float(np.clip(raw_correction, -MAX_CORRECTION, MAX_CORRECTION))

    rate_limited_correction = self._rate_limit(raw_correction, dt)
    self.correction = self._filter.update(rate_limited_correction)
    self._active = True

    delta_correction = self.correction - self._last_correction
    wobble_rate = abs(delta_correction) / max(dt, 1e-5)
    self._last_correction = self.correction

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
                    curvature_error=raw_curvature_error,  
                    raw_correction=raw_correction, rate_limited_correction=rate_limited_correction,
                    correction=self.correction,
                    delta_correction=delta_correction, wobble_rate=wobble_rate,
                    avg_lane_width=avg_lane_width)
    
    return self.correction
