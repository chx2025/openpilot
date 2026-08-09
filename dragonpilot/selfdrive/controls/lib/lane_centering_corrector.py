#!/usr/bin/env python3
"""
Lane Centering Corrector (LCC)

根據 modelV2 的車道線 (laneLines) 幾何位置，計算「本車道中心 offset」，
並轉換成一個很小的曲率修正量，疊加在 model 原本輸出的 desired_curvature 上。

設計原則：
  - 完全獨立模組，不觸碰 latcontrol.py / latcontrol_pid.py / latcontrol_torque.py /
    latcontrol_angle.py 任何邏輯，這些檔案維持 upstream 原樣。
  - controlsd.py 只在組出 new_desired_curvature 之後、丟進 clip_curvature() 之前，
    疊加本模組算出的修正量，修正量算錯或關閉時 correction=0，行為等同原本 model。
  - 下游 clip_curvature() 的曲率/側向加速度/側向 jerk 限幅仍然完整作用於
    「model + 修正量」的總和，所以就算本模組給錯值，也會被既有安全限制包住。
  - 車道線信心不足（單線消失、施工、匝道...）時修正量直接淡出到 0，不做外插硬猜。

尚未實際路測，預設 dp_lcc_enabled=False，需在 Params 手動開啟。
"""

import os
import time

import numpy as np

from openpilot.common.params import Params
from openpilot.common.filter_simple import FirstOrderFilter

PARAM_REFRESH_SEC = 2.0

# --- Debug log 設定（獨立 CSV，供路測後回傳分析用） ---
LOG_PATH = "/data/media/0/realdata/lcc_debug.csv"
LOG_INTERVAL_SEC = 0.1  # 穩態最多每 0.1s（10Hz）寫一次，避免頻繁寫入 SD 卡
LOG_COLUMNS = [
  "t", "dt", "v_kph", "state",
  "lat_active", "speed_gate", "sharp_turn", "model_curvature",
  "yield_hold_timer", "engage_ramp_timer", "ramp_factor",
  "weight", "lane_center_error", "error_rate", "p_term", "d_term",
  "raw_correction", "correction",
]

# --- 系統內建參數（不從 Params 讀取，先求穩，之後有需要再開放調參） ---
SPEED_ON_KPH = 40.0
SPEED_OFF_KPH = 30.0
KPH_TO_MS = 1000.0 / 3600.0

LOOKAHEAD_DIST_M = 15.0     # 用來算車道中心 offset 的前視距離
FILTER_RC_SEC = 0.5         # 修正量一階低通時間常數，避免抖動
SHARP_TURN_CURVATURE = 0.06  # 1/m，約等於路徑半徑 17m 以下的轉彎

PROB_MIN = 0.3
PROB_FULL = 0.6

MAX_CORRECTION = 0.006  # 修正量曲率上限 (1/m)，寫死在程式碼內

KD_GAIN = 0.6  # 阻尼增益：誤差變化率的抑制強度

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

    # --- UI 控制參數（從 Params 讀取） ---
    self._enabled = False

    # --- 狀態 ---
    self._filter = FirstOrderFilter(0.0, FILTER_RC_SEC, 0.01)
    self.correction = 0.0
    self.lane_center_error = 0.0
    self._prev_lane_center_error = 0.0  # 上一幀誤差，用於 D 項計算
    self._prev_error_valid = False  # False 時代表沒有上一幀可比較，D 項該幀直接視為 0
    self.weight = 0.0
    self._active = False
    self._speed_gate = False  # 車速遲滯開關的目前狀態
    self._yield_hold_timer = 0.0  # 「方向燈+出力」持續時間累計，用於去彈跳
    self._engage_ramp_timer = 0.0  # 剛啟用時的漸強爬升時間累計

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
    self.lane_center_error = 0.0
    self._prev_lane_center_error = 0.0
    self._prev_error_valid = False
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

  def update(self, model_v2, v_ego: float, current_curvature: float, lat_active: bool, dt: float,
             left_blinker: bool = False, right_blinker: bool = False,
             steering_pressed: bool = False) -> float:
    """回傳要疊加到 desired_curvature 的修正量 (1/m)"""
    self._read_params()
    self._filter.dt = dt

    v_ego_kph = v_ego / KPH_TO_MS
    if v_ego_kph >= SPEED_ON_KPH:
      self._speed_gate = True
    elif v_ego_kph <= SPEED_OFF_KPH:
      self._speed_gate = False

    model_curvature = getattr(model_v2.action, "desiredCurvature", 0.0) if lat_active else 0.0
    is_sharp_turn = abs(model_curvature) > SHARP_TURN_CURVATURE

    hard_invalid = (
      not self._enabled or
      not lat_active or
      not self._speed_gate or
      is_sharp_turn
    )
    if hard_invalid:
      if not self._enabled:
        _state = "DISABLED"
      elif not lat_active:
        _state = "NOT_LAT_ACTIVE"
      elif not self._speed_gate:
        _state = "SPEED_GATE_OFF"
      else:
        _state = "SHARP_TURN"
      self._log_row(_state, dt=dt, v_kph=v_ego_kph, lat_active=lat_active,
                    speed_gate=self._speed_gate, sharp_turn=is_sharp_turn,
                    model_curvature=model_curvature,
                    yield_hold_timer=self._yield_hold_timer,
                    engage_ramp_timer=self._engage_ramp_timer,
                    ramp_factor=0.0, weight=0.0, lane_center_error=0.0, error_rate=0.0,
                    p_term=0.0, d_term=0.0, raw_correction=0.0, correction=0.0)
      self.reset()
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
      self.lane_center_error = 0.0
      self._prev_lane_center_error = 0.0
      self._prev_error_valid = False
      self.correction = self._filter.update(0.0)
      self._log_row("YIELDING", dt=dt, v_kph=v_ego_kph, lat_active=lat_active,
                    speed_gate=self._speed_gate, sharp_turn=is_sharp_turn,
                    model_curvature=model_curvature,
                    yield_hold_timer=self._yield_hold_timer,
                    engage_ramp_timer=self._engage_ramp_timer,
                    ramp_factor=ramp_factor, weight=self.weight,
                    lane_center_error=self.lane_center_error, error_rate=0.0,
                    p_term=0.0, d_term=0.0, raw_correction=0.0, correction=self.correction)
      return self.correction

    lane_lines = model_v2.laneLines
    lane_line_probs = model_v2.laneLineProbs

    if len(lane_lines) < 3 or len(lane_line_probs) < 3:
      self._log_row("NO_LANE_DATA", dt=dt, v_kph=v_ego_kph, lat_active=lat_active,
                    speed_gate=self._speed_gate, sharp_turn=is_sharp_turn,
                    model_curvature=model_curvature,
                    yield_hold_timer=self._yield_hold_timer,
                    engage_ramp_timer=self._engage_ramp_timer,
                    ramp_factor=ramp_factor, weight=0.0, lane_center_error=0.0, error_rate=0.0,
                    p_term=0.0, d_term=0.0, raw_correction=0.0, correction=0.0)
      self.reset()
      return 0.0

    lll_prob = lane_line_probs[1]
    rll_prob = lane_line_probs[2]
    min_prob = min(lll_prob, rll_prob)

    self.weight = float(np.clip((min_prob - PROB_MIN) / (PROB_FULL - PROB_MIN), 0.0, 1.0))
    if self.weight <= 0.0:
      self._active = False
      self.correction = self._filter.update(0.0)
      self.lane_center_error = 0.0
      self._prev_lane_center_error = 0.0
      self._prev_error_valid = False
      self._log_row("LOW_CONFIDENCE", dt=dt, v_kph=v_ego_kph, lat_active=lat_active,
                    speed_gate=self._speed_gate, sharp_turn=is_sharp_turn,
                    model_curvature=model_curvature,
                    yield_hold_timer=self._yield_hold_timer,
                    engage_ramp_timer=self._engage_ramp_timer,
                    ramp_factor=ramp_factor, weight=self.weight, lane_center_error=0.0,
                    error_rate=0.0, p_term=0.0, d_term=0.0, raw_correction=0.0,
                    correction=self.correction)
      return self.correction

    lll = lane_lines[1]
    rll = lane_lines[2]
    lll_y = _clip_interp(LOOKAHEAD_DIST_M, lll.x, lll.y)
    rll_y = _clip_interp(LOOKAHEAD_DIST_M, rll.x, rll.y)

    if lll_y is None or rll_y is None:
      self._active = False
      self.correction = self._filter.update(0.0)
      self.lane_center_error = 0.0
      self._prev_lane_center_error = 0.0
      self._prev_error_valid = False
      self._log_row("INTERP_FAIL", dt=dt, v_kph=v_ego_kph, lat_active=lat_active,
                    speed_gate=self._speed_gate, sharp_turn=is_sharp_turn,
                    model_curvature=model_curvature,
                    yield_hold_timer=self._yield_hold_timer,
                    engage_ramp_timer=self._engage_ramp_timer,
                    ramp_factor=ramp_factor, weight=self.weight, lane_center_error=0.0,
                    error_rate=0.0, p_term=0.0, d_term=0.0, raw_correction=0.0,
                    correction=self.correction)
      return self.correction

    self.lane_center_error = (lll_y + rll_y) / 2.0

    # 引入 Yaw Rate 解耦核心邏輯
    if self._prev_error_valid:
      raw_error_rate = (self.lane_center_error - self._prev_lane_center_error) / dt if dt > 1e-3 else 0.0
      # 扣除因為車輛自身旋轉（航向角變化）造成的假性位移速率
      yaw_rate = v_ego * current_curvature
      # yaw_rate 正值代表向左轉，這會導致 15m 前方的量測點在視野中相對向右移（y 變小，產生負的 raw_error_rate）
      # 因此要加上 LOOKAHEAD_DIST_M * yaw_rate 來補償這個旋轉分量，還原真實的橫向位移速率
      error_rate = raw_error_rate + (LOOKAHEAD_DIST_M * yaw_rate)
    else:
      error_rate = 0.0
      self._prev_error_valid = True
      
    self._prev_lane_center_error = self.lane_center_error

    p_term = 2.0 * self.lane_center_error / (LOOKAHEAD_DIST_M ** 2)
    d_term = KD_GAIN * 2.0 * error_rate / (LOOKAHEAD_DIST_M ** 2)
    raw_correction = p_term - d_term
    raw_correction = float(np.clip(raw_correction, -MAX_CORRECTION, MAX_CORRECTION))
    raw_correction *= self.weight * ramp_factor

    self.correction = self._filter.update(raw_correction)
    self._active = True
    self._log_row("ACTIVE", dt=dt, v_kph=v_ego_kph, lat_active=lat_active,
                  speed_gate=self._speed_gate, sharp_turn=is_sharp_turn,
                  model_curvature=model_curvature,
                  yield_hold_timer=self._yield_hold_timer,
                  engage_ramp_timer=self._engage_ramp_timer,
                  ramp_factor=ramp_factor, weight=self.weight,
                  lane_center_error=self.lane_center_error, error_rate=error_rate,
                  p_term=p_term, d_term=d_term, raw_correction=raw_correction,
                  correction=self.correction)
    return self.correction
