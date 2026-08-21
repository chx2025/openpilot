#!/usr/bin/env python3
"""
Lane Centering Corrector (LCC) v3.0 (DP x Starpilot 終極混血版)
- 外殼與控制邏輯：DP v2.12 (漸進式曲率權重、讓車避險、信心度濾波、晃動偵測)
- 核心演算法：Starpilot 純跟隨 (Pure Pursuit) 幾何算法 (捨棄 polyfit 二次微分)
- 適用車型：高度推薦豐田車系 (Toyota CC 等)，有效消滅直線碎震，保護 EPS 馬達。
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
  "weight", "lookahead_m", "lane_width", "pos_error", "path_std",
  "yield_persist_timer", "yield_factor", "yield_suppression_pct",
  "curvature_error", "raw_correction", "rate_limited_correction", "correction",
  "delta_correction", "wobble_rate"
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

# ==========================================
# --- 🌟 Starpilot 核心可選參數 (在此設定) ---
# ==========================================
# 車道偏移微調 (單位：公尺)。正值偏右，負值偏左。例如 -0.05 代表整車靠左 5 公分。
SP_LANE_OFFSET = 0.0  

# 車道誤差死區 (單位：公尺)。預設 0.08 (8公分)。這範圍內的微小跳動不會引發方向盤修正，是消滅晃動的關鍵。
SP_CENTER_DEADBAND = 0.08  

# 允許介入的最小與最大車道寬度 (單位：公尺)
SP_LANE_WIDTH_MIN = 2.6
SP_LANE_WIDTH_MAX = 4.8

# 最大允許使用者設定的安全偏移量限制，避免不小心設太大
SP_MAX_OFFSET = 0.3
SP_MIN_CENTER_TO_LINE = 1.1
# ==========================================

YIELD_CONFIRM_SEC = 0.15
ENGAGE_RAMP_SEC = 0.8  
SOFT_DISABLE_HOLD_SEC = 0.4

# --- 避讓 (Yield) 邏輯參數 ---
YIELD_MAX_PATH_STD = 0.35
YIELD_BREAK_IN_START = 0.35
YIELD_BREAK_IN_FULL = 0.85
DEFAULT_E2E_AUTHORITY = 1.0

# --- 控制路徑濾波參數 ---
WEIGHT_SMOOTH_TAU = 0.15        
CURVATURE_WEIGHT_SMOOTH_TAU = 0.15  
WEIGHT_ACTIVE_EPS = 1e-4        


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

  def _lookup_path_std_and_error(self, model_v2, target_y_l: float, l: float) -> tuple[float, float]:
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
      pos_error = target_y_l - model_y
      return path_std, pos_error
    except (AttributeError, TypeError, ValueError, IndexError):
      return -1.0, 0.0

  def _yield_factor(self, model_v2, target_y_l: float, l: float, e2e_authority: float, dt: float, v_ego: float) -> tuple[float, float, float, float]:
    path_std, pos_error = self._lookup_path_std_and_error(model_v2, target_y_l, l)
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
    
    delta_correction = self.correction - self._last_correction
    wobble_rate = abs(delta_correction) / max(dt, 1e-5)
    self._last_correction = self.correction

    self._log_row(state, dt=dt, v_kph=v_ego_kph, lat_active=lat_active,
                  speed_gate=self._speed_gate, model_curvature=model_curvature, 
                  yield_hold_timer=self._yield_hold_timer, engage_ramp_timer=self._engage_ramp_timer, 
                  ramp_factor=ramp_factor, curvature_weight=self.curvature_weight,
                  weight=0.0, lookahead_m=0.0, lane_width=0.0, pos_error=0.0, path_std=-1.0, 
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
      self.correction = self._decay(dt)
      self._log_fallback("YIELDING", dt, v_ego_kph, lat_active, model_curvature, ramp_factor)
      return self.correction

    lane_lines = model_v2.laneLines
    lane_line_probs = model_v2.laneLineProbs

    if len(lane_lines) < 3 or len(lane_line_probs) < 3:
      self._active = False
      self.weight = 0.0
      self.correction = self._decay(dt)
      self._log_fallback("NO_LANE_DATA", dt, v_ego_kph, lat_active, model_curvature, ramp_factor)
      return self.correction

    # --- 信心度計算 (DP 原版邏輯) ---
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
      self.correction = self._decay(dt)
      self._log_fallback("LOW_CONFIDENCE", dt, v_ego_kph, lat_active, model_curvature, ramp_factor)
      return self.correction

    # ==============================================================
    # 🌟 核心替換：改用 Starpilot 純跟隨 (Pure Pursuit) 空間位置演算法 🌟
    # ==============================================================
    lll = lane_lines[1]
    rll = lane_lines[2]
    
    # 動態前視距離 (結合 Starpilot 與 DP 的優點，依照車速預測)
    lookahead = float(np.clip(v_ego * LOOKAHEAD_TIME_SEC, MIN_LOOKAHEAD_M, 40.0))

    l_x = np.asarray(lll.x, dtype=float)
    l_y = np.asarray(lll.y, dtype=float)
    r_x = np.asarray(rll.x, dtype=float)
    r_y = np.asarray(rll.y, dtype=float)
    pos_x = np.asarray(model_v2.position.x, dtype=float)
    pos_y = np.asarray(model_v2.position.y, dtype=float)

    # 取得前視距離當下的左、右車道線 Y 座標，與模型預測車輛的 Y 座標
    left_y_l = _clip_interp(lookahead, l_x, l_y)
    right_y_l = _clip_interp(lookahead, r_x, r_y)
    model_y_l = _clip_interp(lookahead, pos_x, pos_y)

    if left_y_l is None or right_y_l is None or model_y_l is None:
      self._active = False
      self.weight = 0.0
      self.correction = self._decay(dt)
      self._log_fallback("INTERP_FAIL", dt, v_ego_kph, lat_active, model_curvature, ramp_factor)
      return self.correction

    # 車道寬度驗證
    lane_width = right_y_l - left_y_l
    if not (SP_LANE_WIDTH_MIN <= lane_width <= SP_LANE_WIDTH_MAX):
      self._active = False
      self.weight = 0.0
      self.correction = self._decay(dt)
      self._log_fallback("LANE_WIDTH_INVALID", dt, v_ego_kph, lat_active, model_curvature, ramp_factor)
      return self.correction

    # 安全偏移量計算
    max_safe_offset = min(SP_MAX_OFFSET, max(0.0, lane_width * 0.5 - SP_MIN_CENTER_TO_LINE))
    applied_offset = float(np.clip(SP_LANE_OFFSET, -max_safe_offset, max_safe_offset))

    # 計算目標中心點與實際位置誤差
    target_y_l = 0.5 * (left_y_l + right_y_l) + applied_offset
    pos_error_raw = target_y_l - model_y_l

    # Starpilot 靈魂：位置誤差死區 (Deadband)，過濾直線碎震
    pos_error_abs = abs(pos_error_raw)
    if pos_error_abs <= SP_CENTER_DEADBAND:
      pos_error = 0.0
    else:
      pos_error = np.copysign(pos_error_abs - SP_CENTER_DEADBAND, pos_error_raw)

    # 純跟隨幾何公式轉換：將橫向位置誤差轉換為目標曲率誤差
    curvature_error = float(2.0 * pos_error / (lookahead ** 2))
    
    if not np.isfinite(curvature_error):
      self._active = False
      self.weight = 0.0
      self.correction = self._decay(dt)
      self._log_fallback("MATH_ERROR_FAIL", dt, v_ego_kph, lat_active, model_curvature, ramp_factor)
      return self.correction

    # ==============================================================

    # 保留 DP 強大的退讓 (Yield) 避險機制
    yield_factor, path_std, log_pos_error, yield_persist_timer = self._yield_factor(model_v2, target_y_l, lookahead, e2e_authority, dt, v_ego)

    # 最終連乘公式：權重 * 緩起動 * 避險讓車係數 * 彎道漸進權重 * 誤差
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
                    weight=self.weight, lookahead_m=lookahead, lane_width=lane_width, pos_error=pos_error_raw,
                    path_std=path_std,
                    yield_persist_timer=yield_persist_timer, yield_factor=yield_factor,
                    yield_suppression_pct=(1.0 - yield_factor) * 100.0,
                    curvature_error=curvature_error,  
                    raw_correction=raw_correction, rate_limited_correction=rate_limited_correction,
                    correction=self.correction,
                    delta_correction=delta_correction, wobble_rate=wobble_rate)
    
    return self.correction
