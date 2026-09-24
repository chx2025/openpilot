import numpy as np
import time
import threading
from collections.abc import Callable
from enum import Enum
from openpilot.cereal import messaging, log
from opendbc.car.structs import car
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import drop_realtime
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.lib.prime_state import PrimeState
from openpilot.system.ui.lib.application import gui_app
from openpilot.common.hardware import HARDWARE, PC
from openpilot.common.hardware.usb import TYPEC_CC_ORIENTATION_PATH, get_usb_state, is_chestnut_usb_id, read_int
from openpilot.selfdrive.modeld.helpers import chestnut_compiled
from openpilot.selfdrive.ui.egpu_status import CHESTNUT_LOAD_NOMINAL_S

from openpilot.selfdrive.ui.sunnypilot.ui_state import UIStateSP, DeviceSP

BACKLIGHT_OFFROAD = 65 if HARDWARE.get_device_type() == "mici" else 50
PARAM_UPDATE_TIME = 1 / 5.0


class UIStatus(Enum):
  DISENGAGED = "disengaged"
  ENGAGED = "engaged"
  OVERRIDE = "override"
  LAT_ONLY = "lat_only"
  LONG_ONLY = "long_only"


class ChestnutState(Enum):
  DISCONNECTED = "disconnected"
  UNCOMPILED = "uncompiled"
  READY = "ready"
  LOADING = "loading"
  ACTIVE = "active"
  FAILED = "failed"


class UIState(UIStateSP):
  _instance: 'UIState | None' = None

  def __new__(cls):
    if cls._instance is None:
      cls._instance = super().__new__(cls)
      cls._instance._initialize()
    return cls._instance

  def _initialize(self):
    UIStateSP.__init__(self)
    self.params = Params()
    self.sm = messaging.SubMaster(
      [
        "modelV2",
        "controlsState",
        "onroadEvents",
        "extrinsicsCalibration",
        "radarState",
        "deviceState",
        "pandaStates",
        "carParams",
        "driverMonitoringState",
        "carState",
        "driverStateV2",
        "narrowRoadCameraState",
        "wideRoadCameraState",
        "managerState",
        "selfdriveState",
        "longitudinalPlan",
        "gpsLocationExternal",
        "carOutput",
        "carControl",
        "vehicleParameters",
        "testJoystick",
        "rawAudioData",
        "chestnutState",
      ] + self.sm_services_ext
    )

    self.prime_state = PrimeState()

    # UI Status tracking
    self.status: UIStatus = UIStatus.DISENGAGED
    self.started_frame: int = 0
    self.started_time: float = 0.0
    self._engaged_prev: bool = False
    self._started_prev: bool = False

    # Core state variables
    self.is_metric: bool = self.params.get_bool("IsMetric")
    self.is_release = False  # self.params.get_bool("IsReleaseBranch")
    self.always_on_dm: bool = self.params.get_bool("AlwaysOnDM")
    self.experimental_mode: bool = self.params.get_bool("ExperimentalMode")
    self.experimental_mode_confirmed: bool = self.params.get_bool("ExperimentalModeConfirmed")
    self.chestnut_present: bool = False
    self.chestnut_compiled: bool = chestnut_compiled()
    self.chestnut_active: bool | None = None
    self.chestnut_loading: bool = False
    self.chestnut_loading_progress: int = 0
    self._chestnut_loading_started_ts: float | None = None
    self.usb_connected: bool = False
    self.usb_connected_ts: float | None = None
    self.usb_disconnected_ts: float | None = None
    self.usb_unknown: bool = False
    self.chestnut_state = ChestnutState.DISCONNECTED
    self.started: bool = False
    self.ignition: bool = False
    self.recording_audio: bool = False
    self.panda_type: log.PandaState.PandaType = log.PandaState.PandaType.unknown
    self.personality: log.LongitudinalPersonality = log.LongitudinalPersonality.standard
    self.has_longitudinal_control: bool = False
    self.is_body: bool | None = False
    self.CP: car.CarParams | None = None
    self.light_sensor: float = -1.0

    self._params_thread: threading.Thread | None = None

    # Callbacks
    self._offroad_transition_callbacks: list[Callable[[], None]] = []
    self._engaged_transition_callbacks: list[Callable[[], None]] = []
    self._on_body_changed_callbacks: list[Callable[[], None]] = []

  def add_offroad_transition_callback(self, callback: Callable[[], None]):
    self._offroad_transition_callbacks.append(callback)

  def add_engaged_transition_callback(self, callback: Callable[[], None]):
    self._engaged_transition_callbacks.append(callback)

  def add_on_body_changed_callbacks(self, callback: Callable[[], None]):
    self._on_body_changed_callbacks.append(callback)

  @property
  def engaged(self) -> bool:
    return self.started and (self.sm["selfdriveState"].enabled or self.sm["selfdriveStateSP"].mads.enabled)

  @property
  def big_model_failed(self) -> bool:
    """大模型链路正常但加载/运行失败（侧边栏显示 MODEL ERR）。"""
    return self.chestnut_state == ChestnutState.FAILED

  def _read_chestnut_loading_progress(self) -> int:
    """大模型加载进度（0~100）。

    优先读真实进度参数；当前分支的 modeld 不写该参数（见 `modeld_v2/modeld.py`，
    它只 put_bool("ChestnutLoading", ...)），于是退化成「按已加载时长单调估算、
    99% 封顶」——真正的就绪信号仍是 ChestnutLoading 变 False / ChestnutActive 变 True。

    基准时长 CHESTNUT_LOAD_NOMINAL_S 来自实车日志实测（见 egpu_status.py 注释）。
    """
    # 先短路：不在加载中就没必要去碰那两个不存在的参数键
    if not self.chestnut_loading:
      self._chestnut_loading_started_ts = None
      return 0

    # 当前分支没有这两个键（params_keys.h 里查无此名），get() 会抛异常；
    # 留着是为了将来 modeld 真写了进度参数时能直接接上。
    for key in ("ChestnutLoadingProgress", "UsbGpuLoadingProgress"):
      try:
        raw = self.params.get(key, return_default=True)
      except Exception:
        raw = None
      try:
        value = int(raw) if raw is not None else 0
      except (TypeError, ValueError):
        value = 0
      if value > 0:
        return max(0, min(100, value))

    now = time.monotonic()
    if self._chestnut_loading_started_ts is None:
      self._chestnut_loading_started_ts = now
      return 0

    elapsed = max(0.0, now - self._chestnut_loading_started_ts)
    return max(0, min(99, int(elapsed / CHESTNUT_LOAD_NOMINAL_S * 100)))

  def is_onroad(self) -> bool:
    return self.started

  def is_offroad(self) -> bool:
    return not self.started

  def update(self) -> None:
    self.prime_state.start()  # start thread after manager forks ui
    if self._params_thread is None:
      self._params_thread = threading.Thread(target=self._params_refresh_worker, daemon=True)
      self._params_thread.start()

    self.sm.update(0)
    self._update_state()
    self._update_status()
    self._update_chestnut_state()
    device.update()
    UIStateSP.update(self)

  def _params_refresh_worker(self):
    drop_realtime()
    while True:
      self.update_params()
      time.sleep(PARAM_UPDATE_TIME)

  def _update_state(self) -> None:
    # Handle panda states updates
    if self.sm.updated["pandaStates"]:
      panda_states = self.sm["pandaStates"]

      if len(panda_states) > 0:
        # Get panda type from first panda
        self.panda_type = panda_states[0].pandaType
        # Check ignition status across all pandas
        if self.panda_type != log.PandaState.PandaType.unknown:
          self.ignition = any(state.ignitionLine or state.ignitionCan for state in panda_states)
    elif not self.sm.alive["pandaStates"]:
      self.panda_type = log.PandaState.PandaType.unknown

    # Handle wide road camera state updates
    if self.sm.updated["wideRoadCameraState"]:
      cam_state = self.sm["wideRoadCameraState"]
      self.light_sensor = max(100.0 - cam_state.exposureValPercent, 0.0)
    elif not self.sm.alive["wideRoadCameraState"] or not self.sm.valid["wideRoadCameraState"]:
      self.light_sensor = -1

    # Update started state
    self.started = self.sm["deviceState"].started and self.ignition

    # Update body state
    if self.CP is not None and self.is_body != self.CP.notCar:
      self.is_body = self.CP.notCar
      for callback in self._on_body_changed_callbacks:
        callback()

  def _update_status(self) -> None:
    if self.started and self.sm.updated["selfdriveState"]:
      ss = self.sm["selfdriveState"]
      state = ss.state

      if state in (log.SelfdriveState.OpenpilotState.preEnabled, log.SelfdriveState.OpenpilotState.overriding):
        self.status = UIStatus.OVERRIDE
      else:
        self.status = UIStatus.ENGAGED if ss.enabled else UIStatus.DISENGAGED

      self.status = UIStatus(UIStateSP.update_status(ss, self.sm["selfdriveStateSP"], self.sm["onroadEvents"]))

    # Check for engagement state changes
    if self.engaged != self._engaged_prev:
      for callback in self._engaged_transition_callbacks:
        callback()
      self._engaged_prev = self.engaged

    # Handle onroad/offroad transition
    if self.started != self._started_prev or self.sm.frame == 1:
      if self.started:
        self.status = UIStatus.DISENGAGED
        self.started_frame = self.sm.frame
        self.started_time = time.monotonic()
        self.chestnut_present = self.sm["deviceState"].chestnutPresent

      for callback in self._offroad_transition_callbacks:
        callback()

      self._started_prev = self.started

  def _update_chestnut_state(self) -> None:
    detected = self.sm["deviceState"].chestnutPresent
    if not self.started:
      self.chestnut_present = detected
      self.chestnut_state = (ChestnutState.READY if detected and self.chestnut_compiled else
                             ChestnutState.UNCOMPILED if detected else ChestnutState.DISCONNECTED)
      return

    model_seen = self.sm.recv_frame["modelV2"] > self.started_frame
    # [egpu-late-online-heal]
    # (1) chestnut_present 只在 onroad 转换那一刻锁存一次，此后整趟行程不再刷新。
    # eGPU「上电较晚」（车机先上路、显卡后枚举）时锁进去的就是 False，于是
    # chestnut_state 恒为 DISCONNECTED，左下角首页图标一直显示「GPU x」，
    # 而第四格用实时 deviceState 判据、早已显示绿色 —— 两处自相矛盾。
    # 这里放开 False -> True 的上行自愈；下行仍保持锁存，运行期掉线交给下面
    # 的 modelV2.big 判据表达成 FAILED，免得 USB 瞬时重枚举把图标抖成「未连接」。
    if detected and not self.chestnut_present:
      self.chestnut_present = True

    # [egpu-late-online-heal]
    # (2) 判据顺序必须是「真在跑」最优先。原实现把加载中/成功都排在
    # (model_seen and not modelV2.big) -> FAILED 之后，且把「曾经 FAILED」当永久
    # 锁存，于是 eGPU 晚上电时先被判 FAILED，此后即使大模型真跑起来也永远停在
    # 橙色 FAILED；LOADING 分支也被彻底遮蔽（加载进度看不见）。
    # 模型真在跑是链路通畅的最强证据，必须压过加载/失败判据；modeld 掉线自愈
    # 重试成功后图标能否回到绿色，也依赖这一点。
    model_running = bool(model_seen and self.sm.alive["modelV2"] and self.sm["modelV2"].big)

    if not self.chestnut_present:
      self.chestnut_state = ChestnutState.DISCONNECTED
    elif not self.chestnut_compiled:
      self.chestnut_state = ChestnutState.UNCOMPILED
    elif model_running:
      self.chestnut_state = ChestnutState.ACTIVE
    elif self.chestnut_loading or not model_seen:
      self.chestnut_state = ChestnutState.LOADING
    else:
      self.chestnut_state = ChestnutState.FAILED

  def update_params(self) -> None:
    # For slower operations
    # Update longitudinal control state
    CP_bytes = self.params.get("CarParamsPersistent")
    if CP_bytes is not None:
      self.CP = messaging.log_from_bytes(CP_bytes, car.CarParams)
      if self.CP.alphaLongitudinalAvailable:
        self.has_longitudinal_control = self.params.get_bool("AlphaLongitudinalEnabled")
      else:
        self.has_longitudinal_control = self.CP.openpilotLongitudinalControl

    self.recording_audio = self.params.get_bool("RecordAudio") and self.started
    self.is_metric = self.params.get_bool("IsMetric")
    self.always_on_dm = self.params.get_bool("AlwaysOnDM")
    self.experimental_mode = self.params.get_bool("ExperimentalMode")
    self.experimental_mode_confirmed = self.params.get_bool("ExperimentalModeConfirmed")
    if not self.chestnut_compiled:
      self.chestnut_compiled = chestnut_compiled()
    self.chestnut_active = self.params.get_bool("ChestnutActive")
    self.chestnut_loading = self.params.get_bool("ChestnutLoading")
    self.chestnut_loading_progress = self._read_chestnut_loading_progress()
    now = time.monotonic()
    if read_int(TYPEC_CC_ORIENTATION_PATH) != 0:
      self.usb_disconnected_ts = None
      if not self.usb_connected:
        self.usb_connected = True
        self.usb_connected_ts = now
        self.usb_unknown = False
      elif self.usb_connected_ts is not None and now - self.usb_connected_ts > 10.:
        self.usb_unknown = not any(is_chestnut_usb_id(d["vendorId"], d["productId"], True) for d in get_usb_state())
        self.usb_connected_ts = None
    elif self.usb_connected:
      if self.usb_disconnected_ts is None:
        self.usb_disconnected_ts = now
      elif now - self.usb_disconnected_ts > PARAM_UPDATE_TIME:
        self.usb_connected = False
        self.usb_connected_ts = None
        self.usb_unknown = False

    UIStateSP.update_params(self)


class Device(DeviceSP):
  def __init__(self):
    DeviceSP.__init__(self)
    self._ignition = False
    self._interaction_time: float = -1
    self._override_interactive_timeout: int | None = None
    self._interactive_timeout_callbacks: list[Callable] = []
    self._prev_timed_out = False
    self._awake: bool = True

    self._offroad_brightness: int = BACKLIGHT_OFFROAD
    self._last_brightness: int = 0
    self._brightness_filter = FirstOrderFilter(BACKLIGHT_OFFROAD, 10.00, 1 / gui_app.target_fps)
    self._brightness_thread: threading.Thread | None = None
    self._brightness_event = threading.Event()
    self._brightness_target: int = 0

  @property
  def awake(self) -> bool:
    return self._awake

  def set_override_interactive_timeout(self, timeout: int | None) -> None:
    # Override the interactive timeout duration temporarily
    self._override_interactive_timeout = timeout
    self._reset_interactive_timeout()

  @property
  def interactive_timeout(self) -> int:
    if self._override_interactive_timeout is not None:
      return self._override_interactive_timeout

    if gui_app.sunnypilot_ui() and ui_state.custom_interactive_timeout != 0:
      return ui_state.custom_interactive_timeout

    ignition_timeout = 10 if gui_app.big_ui() else 5
    return ignition_timeout if ui_state.ignition else 30

  def _reset_interactive_timeout(self) -> None:
    self._interaction_time = time.monotonic() + self.interactive_timeout

  def add_interactive_timeout_callback(self, callback: Callable):
    if callback not in self._interactive_timeout_callbacks:
      self._interactive_timeout_callbacks.append(callback)

  def remove_interactive_timeout_callback(self, callback: Callable):
    if callback in self._interactive_timeout_callbacks:
      self._interactive_timeout_callbacks.remove(callback)

  def update(self):
    self._start_brightness_thread()  # start thread after manager forks ui

    # do initial reset
    if self._interaction_time <= 0:
      self._reset_interactive_timeout()

    self._update_brightness()
    self._update_wakefulness()

  def _start_brightness_thread(self):
    if self._brightness_thread is None or not self._brightness_thread.is_alive():
      self._brightness_thread = threading.Thread(target=self._brightness_worker, daemon=True)
      self._brightness_thread.start()

  def _brightness_worker(self):
    drop_realtime()
    while True:
      self._brightness_event.wait()
      self._brightness_event.clear()
      HARDWARE.set_screen_brightness(self._brightness_target)

  def set_offroad_brightness(self, brightness: int | None):
    if brightness is None:
      brightness = BACKLIGHT_OFFROAD
    self._offroad_brightness = min(max(brightness, 0), 100)

  def _update_brightness(self):
    clipped_brightness = self._offroad_brightness

    if ui_state.started and ui_state.light_sensor >= 0:
      clipped_brightness = ui_state.light_sensor

      # CIE 1931 - https://www.photonstophotos.net/GeneralTopics/Exposure/Psychometric_Lightness_and_Gamma.htm
      if clipped_brightness <= 8:
        clipped_brightness = clipped_brightness / 903.3
      else:
        clipped_brightness = ((clipped_brightness + 16.0) / 116.0) ** 3.0

      min_brightness = 30
      if gui_app.sunnypilot_ui():
        min_brightness = DeviceSP.set_min_onroad_brightness(ui_state, min_brightness)

      clipped_brightness = float(np.interp(clipped_brightness, [0, 1], [min_brightness, 100]))

    brightness = round(self._brightness_filter.update(clipped_brightness))

    if gui_app.sunnypilot_ui():
      brightness = DeviceSP.set_onroad_brightness(ui_state, self._awake, brightness)

    if not self._awake:
      brightness = 0

    if brightness != self._last_brightness:
      self._brightness_target = int(brightness)
      self._brightness_event.set()
      self._last_brightness = int(brightness)

  def _update_wakefulness(self):
    # Handle interactive timeout
    ignition_just_turned_off = not ui_state.ignition and self._ignition
    self._ignition = ui_state.ignition

    if ignition_just_turned_off or any(ev.left_down for ev in gui_app.mouse_events):
      if gui_app.sunnypilot_ui():
        DeviceSP.wake_from_dimmed_onroad_brightness(ui_state, gui_app.mouse_events)

      self._reset_interactive_timeout()

    interaction_timeout = time.monotonic() > self._interaction_time
    if interaction_timeout and not self._prev_timed_out:
      for callback in self._interactive_timeout_callbacks:
        callback()
    self._prev_timed_out = interaction_timeout

    self._set_awake(ui_state.ignition or not interaction_timeout or PC)

  def _set_awake(self, on: bool, _ui_state=None):
    # screensaver holds _awake True, so waking is not a state change
    if on and self._blocked_by_screensaver:
      self.dismiss_screensaver(_ui_state or ui_state)

    if on != self._awake:
      super()._set_awake(on, _ui_state or ui_state)
      if self._blocked_by_screensaver:
        return
      self._awake = on
      cloudlog.debug(f"setting display power {int(on)}")
      HARDWARE.set_display_power(on)
      gui_app.set_should_render(on)


# Global instance
ui_state = UIState()
device = Device()
