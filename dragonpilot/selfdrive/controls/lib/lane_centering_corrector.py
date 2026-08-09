#!/usr/bin/env python3
"""
Lane Centering Corrector (LCC) - 多點曲線擬合與動態融合版

將 5~30m 範圍內的左右車道線計算出多個中心點，並利用 np.polyfit 擬合出 
y = ax^2 + bx + c 的二次曲線。藉此精準分離「道路曲率(a)」、「航向誤差(b)」與「橫向偏移(c)」。
隨後算出目標居中曲率，並依據標線信心度與 Model 原始輸出的 desiredCurvature 進行按比例融合，
提供極度平滑且不會因座標系旋轉而自激振盪的居中修正。
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
  "raw_correction", "correction",
]

# --- 系統內建參數 ---
SPEED_ON_KPH = 40.0
SPEED_OFF_KPH = 30.0
KPH_TO_MS = 1000.0 / 3600.0

# 用於擬合車道中心曲線的採樣點 (公尺)
FIT_X = np.array([5.0, 10.0, 15.0, 20.0, 25.0, 30.0])
LOOKAHEAD_DIST_M = 15.0  # 作為基準的目標前視距離
FILTER_RC_SEC = 0.5      

SHARP_TURN_CURVATURE = 0.06 

PROB_MIN = 0.3
PROB_FULL = 0.6

# 融合權重上限：當車道線極度清晰時，LCC 修正量最高佔最終軌跡的比例
# 例如 0.25 代表 25% 依賴車道線幾何，75% 依賴 E2E 模型預測
LANE_WEIGHT_MAX = 0.25 
MAX_CORRECTION = 0.006  # 修正量曲率上限 (1/m)

YIELD_CONFIRM_SEC = 0.15
ENGAGE_RAMP_SEC = 1.5


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
                  lane_target_curv=0.0, raw_correction=0.0, correction=self.correction)

  def update(self, model_v2, v_ego: float, lat_active: bool, dt: float,
             left_blinker: bool = False, right_blinker: bool = False,
             steering_pressed: bool = False) -> float:
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
      self.correction = self._filter.update(0.0)
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
      self.correction = self._filter.update(0.0)
      self._log_fallback("LOW_CONFIDENCE", dt, v_ego_kph, lat_active, is_sharp_turn, model_curvature, ramp_factor)
      return self.correction

    lll = lane_lines[1]
    rll = lane_lines[2]
    
    # 取樣 5~30m 多個座標點計算中心
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
      self.correction = self._filter.update(0.0)
      self._log_fallback("INTERP_FAIL", dt, v_ego_kph, lat_active, is_sharp_turn, model_curvature, ramp_factor)
      return self.correction

    # 擬合二次曲線方程式 y = ax^2 + bx + c
    coeffs = np.polyfit(valid_x, center_y, 2)
    a, b, c = coeffs[0], coeffs[1], coeffs[2]

    # 利用公式：曲率 = 2a + 2b/L + 2c/L^2，算出最符合車道中心的理論曲率
    L = LOOKAHEAD_DIST_M
    lane_target_curvature = (2.0 * a) + (2.0 * b / L) + (2.0 * c / (L ** 2))

    # 動態路徑融合 (Path Fusion):
    # final_curvature = model_curvature * (1 - weight) + lane_target_curvature * weight
    # 因為在 controlsd.py 裡，最終是 new_desired_curvature = model_curvature + correction
    # 所以我們的修正量 = final_curvature - model_curvature = weight * (lane_target_curvature - model_curvature)
    raw_correction = self.weight * ramp_factor * (lane_target_curvature - model_curvature)
    raw_correction = float(np.clip(raw_correction, -MAX_CORRECTION, MAX_CORRECTION))

    self.correction = self._filter.update(raw_correction)
    self._active = True

    self._log_row("ACTIVE", dt=dt, v_kph=v_ego_kph, lat_active=lat_active,
                  speed_gate=self._speed_gate, sharp_turn=is_sharp_turn,
                  model_curvature=model_curvature, yield_hold_timer=self._yield_hold_timer,
                  engage_ramp_timer=self._engage_ramp_timer, ramp_factor=ramp_factor,
                  weight=self.weight, poly_a=a, poly_b=b, poly_c=c,
                  lane_target_curv=lane_target_curvature, 
                  raw_correction=raw_correction, correction=self.correction)
    
    return self.correction
