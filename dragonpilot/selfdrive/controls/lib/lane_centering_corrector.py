#!/usr/bin/env python3
"""
Lane Centering Corrector (LCC) - 基於 Starpilot 演算法精簡版

1. 誤差計算基準改為 target_y (車道中心) 與 model_y (E2E模型預測路徑) 的差值，自然解耦航向角。
2. 引入 Deadband (8cm 死區)，過濾微小視覺雜訊。
3. 嚴格的車道寬度與放寬後的標準差 (Std) 過濾。
* 已移除 E2E Authority 避讓機制，全程嚴格置中。
* 內建獨立的低通濾波器，徹底避免 API 相容性問題。
"""

import os
import time
import numpy as np

from cereal import log
from openpilot.common.params import Params

PARAM_REFRESH_SEC = 2.0

# --- Debug log 設定 ---
LOG_PATH = "/data/media/0/realdata/lcc_debug.csv"
LOG_INTERVAL_SEC = 0.1  
LOG_COLUMNS = [
  "t", "dt", "v_kph", "state",
  "lat_active", "speed_gate", "lane_width", "target_y",
  "model_y", "error", 
  "raw_correction", "correction",
]

# --- Starpilot 演算法核心常數 ---
_MIN_V_EGO = 5.0             # 最低作動速度 (約 18 km/h)
_MIN_LANE_PROB = 0.5         # 最低標線信心度 (已放寬為 0.5)
_MAX_LANE_STD = 0.4          # 最大標線標準差 (已放寬為 0.4)
_MIN_LANE_WIDTH = 2.6        # 最小合理車道寬度
_MAX_LANE_WIDTH = 4.8        # 最大合理車道寬度
_MAX_OFFSET = 0.3            # 允許的最大自訂偏移量
_MIN_CENTER_TO_LINE = 1.1    # 車道中心到邊線的最短安全距離
_MAX_RAW_CORRECTION = 0.004  # 原始修正曲率上限
_MAX_GAIN = 0.30             # 最終輸出的增益比例
_SMOOTH_TAU = 0.4            # 修正量的低通濾波時間常數 (秒)
_SIGNAL_RELEASE_TAU = 0.20   # 打方向燈時淡出的時間常數
_CONFIDENCE_RELEASE_TAU = 0.20 # 信心不足時淡出的時間常數
_CENTER_ERROR_DEADBAND = 0.08  # 誤差死區 (8公分內不修正，避免抖動)

# 系統自訂常數
SPEED_ON_KPH = 40.0
SPEED_OFF_KPH = 30.0
KPH_TO_MS = 1000.0 / 3600.0
YIELD_CONFIRM_SEC = 0.15


class LaneCenteringCorrector:
  def __init__(self) -> None:
    self._params = Params()
    self._last_params_read = 0.0
    self._enabled = False

    self.correction = 0.0  # 供 controlsd.py 讀取的公開變數
    self._speed_gate = False  
    self._yield_hold_timer = 0.0  

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
    self.correction = 0.0
    self._speed_gate = False
    self._yield_hold_timer = 0.0

  def _smooth(self, target: float, current: float, tau: float, dt: float) -> float:
    """獨立計算的低通濾波，避免依賴外部 Filter 元件造成當機"""
    alpha = dt / max(tau + dt, 1e-5)
    return (1.0 - alpha) * current + alpha * target

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

  def _log_fallback(self, state: str, dt: float, v_ego_kph: float, lat_active: bool):
    self._log_row(state, dt=dt, v_kph=v_ego_kph, lat_active=lat_active,
                  speed_gate=self._speed_gate, lane_width=0, target_y=0,
                  model_y=0, error=0,
                  raw_correction=0, correction=self.correction)

  @staticmethod
  def _valid_path(x, y) -> bool:
    return x.size >= 2 and x.size == y.size and np.isfinite(x).all() and np.isfinite(y).all() and np.all(np.diff(x) > 0)

  @staticmethod
  def _covers(x, distance: float) -> bool:
    return bool(x[0] <= distance <= x[-1])

  def update(self, model_v2, v_ego: float, lat_active: bool, dt: float,
             left_blinker: bool = False, right_blinker: bool = False,
             steering_pressed: bool = False) -> float:
    self._read_params()

    v_ego_kph = v_ego / KPH_TO_MS
    if v_ego_kph >= SPEED_ON_KPH:
      self._speed_gate = True
    elif v_ego_kph <= SPEED_OFF_KPH:
      self._speed_gate = False

    if not self._enabled or not lat_active or not self._speed_gate or v_ego < _MIN_V_EGO:
      self.reset()
      self._log_fallback("DISABLED_OR_SLOW", dt, v_ego_kph, lat_active)
      return 0.0

    raw_yield_condition = (left_blinker or right_blinker) and steering_pressed
    if raw_yield_condition:
      self._yield_hold_timer += dt
    else:
      self._yield_hold_timer = 0.0
    
    # 方向燈與駕駛出力介入時，快速淡出修正量
    if self._yield_hold_timer >= YIELD_CONFIRM_SEC:
      self.correction = self._smooth(0.0, self.correction, _SIGNAL_RELEASE_TAU, dt)
      self._log_fallback("YIELDING_OR_SIGNAL", dt, v_ego_kph, lat_active)
      return self.correction

    try:
      if model_v2.meta.laneChangeState != log.LaneChangeState.off:
        self.reset()
        self._log_fallback("LANE_CHANGE", dt, v_ego_kph, lat_active)
        return 0.0
    except (AttributeError, TypeError, ValueError):
      self.reset()
      return 0.0

    # 執行 Starpilot 核心誤差計算
    valid, raw_correction, log_data = self._calculate_raw_correction(model_v2, v_ego)
    
    if not valid:
      # 當信心不足或無法計算時，平滑淡出
      self.correction = self._smooth(0.0, self.correction, _CONFIDENCE_RELEASE_TAU, dt)
      self._log_row("INVALID_LANE", dt=dt, v_kph=v_ego_kph, lat_active=lat_active, speed_gate=self._speed_gate, **log_data)
      return self.correction

    # 計算最終修正量並進行平滑處理
    target = float(np.clip(raw_correction, -_MAX_RAW_CORRECTION, _MAX_RAW_CORRECTION)) * _MAX_GAIN
    self.correction = self._smooth(target, self.correction, _SMOOTH_TAU, dt)

    self._log_row("ACTIVE", dt=dt, v_kph=v_ego_kph, lat_active=lat_active, speed_gate=self._speed_gate, 
                  raw_correction=raw_correction, correction=self.correction, **log_data)
    
    return self.correction

  def _calculate_raw_correction(self, model_v2, v_ego: float):
    log_data = {"lane_width": 0, "target_y": 0, "model_y": 0, "error": 0}

    try:
      lane_lines = model_v2.laneLines
      probs = np.asarray(model_v2.laneLineProbs, dtype=float)
      stds = np.asarray(model_v2.laneLineStds, dtype=float)
      
      if len(lane_lines) < 3 or probs.size < 3 or stds.size < 3:
        return False, 0.0, log_data
      if not np.isfinite(probs[[1, 2]]).all() or not np.isfinite(stds[[1, 2]]).all():
        return False, 0.0, log_data
      
      if np.any(probs[[1, 2]] < _MIN_LANE_PROB) or np.any(probs[[1, 2]] > 1.0):
        return False, 0.0, log_data
      if np.any(stds[[1, 2]] < 0.0) or np.any(stds[[1, 2]] > _MAX_LANE_STD):
        return False, 0.0, log_data

      left_x = np.asarray(lane_lines[1].x, dtype=float)
      left_y = np.asarray(lane_lines[1].y, dtype=float)
      right_x = np.asarray(lane_lines[2].x, dtype=float)
      right_y = np.asarray(lane_lines[2].y, dtype=float)
      pos_x = np.asarray(model_v2.position.x, dtype=float)
      pos_y = np.asarray(model_v2.position.y, dtype=float)
      
      if not (self._valid_path(left_x, left_y) and self._valid_path(right_x, right_y) and self._valid_path(pos_x, pos_y)):
        return False, 0.0, log_data

      lookahead = float(np.clip(v_ego, 8.0, 35.0))
      if not all(self._covers(x, lookahead) for x in (left_x, right_x, pos_x)):
        return False, 0.0, log_data

      left = float(np.interp(lookahead, left_x, left_y))
      right = float(np.interp(lookahead, right_x, right_y))
      width = right - left
      log_data["lane_width"] = width
      
      if not _MIN_LANE_WIDTH <= width <= _MAX_LANE_WIDTH:
        return False, 0.0, log_data

      target_y = 0.5 * (left + right)
      model_y = float(np.interp(lookahead, pos_x, pos_y))
      error = target_y - model_y
      
      log_data["target_y"] = target_y
      log_data["model_y"] = model_y

      error_abs = abs(error)
      if error_abs <= _CENTER_ERROR_DEADBAND:
        error = 0.0
      else:
        error = np.copysign(error_abs - _CENTER_ERROR_DEADBAND, error)

      log_data["error"] = error

      return True, float(2.0 * error / lookahead ** 2), log_data

    except (AttributeError, IndexError, TypeError, ValueError):
      return False, 0.0, log_data
