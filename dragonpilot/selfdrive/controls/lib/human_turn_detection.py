import time
from enum import Enum, auto

from openpilot.common.params import Params


PARAM_REFRESH_SEC = 2.0
MIN_SPEED_MS = 0.1
MAX_SPEED_MS = 9.72  # 最高車速限制，約 35 km/h


class HTDState(Enum):
  INACTIVE = auto()
  MANUAL_TURN = auto()
  RAMPING = auto()


class HumanTurnDetection:
  def __init__(self) -> None:
    self._params = Params()
    self._last_params_read = 0.0
    
    # --- 參數設定 ---
    self._enabled = True
    self._angle_threshold_deg = 60.0
    self._angle_release_deg = 20.0
    self._torque_start_nm = 2.0
    self._torque_release_nm = 0.6
    self._resume_angle_lock_deg = 60.0  # 安全接管角度鎖

    # --- 系統狀態 ---
    self._state: HTDState = HTDState.INACTIVE
    self._state_change_time = 0.0

    # --- 實時數據 ---
    self._last_angle_raw = 0.0
    self._last_torque_raw = 0.0
    self._last_angle = 0.0
    self._last_torque = 0.0
    self._last_pressed = False

    self._max_turn_angle = 0.0
    self._dynamic_delay = 0.5

  def _read_params(self) -> None:
    now = time.monotonic()
    if now - self._last_params_read < PARAM_REFRESH_SEC:
      return
    self._last_params_read = now
    
    self._enabled = self._params.get_bool("dp_htd_enabled")
    self._angle_threshold_deg = self._get_float("dp_htd_turn_angle_threshold", 90.0)

  def _transition(self, new_state: HTDState, reason: str = "") -> None:
    if new_state != self._state:
      self._state = new_state
      self._state_change_time = time.monotonic()

  def update(
    self, lat_active: bool, cruise_enabled: bool, steering_angle_deg: float, steering_torque_nm: float, v_ego: float, steering_pressed: bool = False
  ) -> tuple[bool, HTDState]:
    self._read_params()

    # 儲存實時數據
    self._last_angle_raw = steering_angle_deg
    self._last_torque_raw = steering_torque_nm
    self._last_angle = abs(steering_angle_deg)
    self._last_torque = abs(steering_torque_nm)
    self._last_pressed = steering_pressed

    # 合併無效條件：定速開啟、未啟用、橫向未啟用、車速不符
    if cruise_enabled or not self._enabled or not lat_active or not (MIN_SPEED_MS <= v_ego <= MAX_SPEED_MS):
      self._transition(HTDState.INACTIVE, "disabled_or_invalid_condition")
      return True, self._state

    if self._state == HTDState.INACTIVE:
      if self._should_trigger():
        self._max_turn_angle = self._last_angle
        self._transition(HTDState.MANUAL_TURN, "trigger")
        return False, self._state
      return True, self._state

    if self._state == HTDState.MANUAL_TURN:
      self._max_turn_angle = max(self._max_turn_angle, self._last_angle)
      if self._should_release():
        # 計算動態延遲：限制在 0.5 到 1.0 秒之間
        self._dynamic_delay = max(0.5, min(self._max_turn_angle / 270.0, 1.0))
        self._transition(HTDState.RAMPING, "release")
      return False, self._state

    # RAMPING 狀態 (安全緩衝區)
    if self._should_trigger():
      self._transition(HTDState.MANUAL_TURN, "retrigger")
      return False, self._state

    # 確保等待時間達到動態秒數，避免扭力突波報錯
    if time.monotonic() - self._state_change_time >= self._dynamic_delay:
      # 安全接管角度鎖：確認方向盤已回到安全範圍才恢復自動駕駛
      if self._last_angle <= self._resume_angle_lock_deg:
        self._max_turn_angle = 0.0
        self._transition(HTDState.INACTIVE, "resume")
        return True, self._state

    return False, self._state

  def _should_trigger(self) -> bool:
    # 判斷：有出力 + 方向盤與扭力方向一致 + 達到扭力門檻 + 達到角度門檻
    return (self._last_pressed and 
            (self._last_angle_raw * self._last_torque_raw > 0) and 
            self._last_torque >= self._torque_start_nm and 
            self._last_angle >= self._angle_threshold_deg)

  def _should_release(self) -> bool:
    # 判斷：完美回正 (角度與扭力皆小) 或是 駕駛已放開方向盤
    return (self._last_torque <= self._torque_release_nm and self._last_angle <= self._angle_release_deg) or not self._last_pressed

  def _get_float(self, key: str, default: float) -> float:
    try:
      val = self._params.get(key)
      return float(val) if val is not None else default
    except Exception:
      return default
