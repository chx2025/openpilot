from openpilot.cereal import log
from openpilot.common.params import Params, UnknownKeyName
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import multiple_button_item, toggle_item
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.widgets import DialogResult
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.common.hardware import HARDWARE

if gui_app.sunnypilot_ui():
  from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp as toggle_item
  from openpilot.system.ui.sunnypilot.widgets.list_view import multiple_button_item_sp as multiple_button_item
  # 数值选择器（绑 Params）：红灯辅助的停位微调，见下面 _traffic_stop_adjust。
  # sunnypilot UI 才有；非 SP UI 时保持 None，下面会跳过该项而不是崩。
  from openpilot.system.ui.sunnypilot.widgets.list_view import option_item_sp
else:
  option_item_sp = None

PERSONALITY_TO_INT = log.LongitudinalPersonality.schema.enumerants

# ── 列表顺序 / 默认值（阿丽定制）───────────────────────────────────────
# **实验模式置顶**，其后是自定义的「踩油门让位 / 红绿灯辅助 / 使用公制单位」（三项连在一起），
# 其余项整体下移。渲染顺序完全由下面这张表决定，**不再依赖 dict 的插入顺序**；
# 表里写了但没注册的键（比如 params 未声明 / 该项被隐藏）会被自动跳过。
HEAD_TOGGLE_ORDER = (
  "ExperimentalMode",
  "GasPedalOverride",
  "TrafficStopAssist",
  "IsMetric",
)

# 这三项按需求「首次运行默认开启」：Params.get_bool() 对不存在的键返回 False，
# 会让 toggle 首次显示成关，所以首次运行时显式落一次 True（不覆盖用户已改过的值）。
DEFAULT_ON_PARAMS = ("GasPedalOverride", "TrafficStopAssist", "IsMetric")

# Description constants
DESCRIPTIONS = {
  "OpenpilotEnabledToggle": tr_noop(
    "Use the sunnypilot system for adaptive cruise control and lane keep driver assistance. " +
    "Your attention is required at all times to use this feature."
  ),
  "DisengageOnAccelerator": tr_noop("When enabled, pressing the accelerator pedal will disengage sunnypilot."),
  "GasPedalOverride": tr_noop(
    "踩下油门时，模型或系统请求的减速度会被削弱，让车辆平顺地贴近前车。松开油门后自动滑行回落至设定速度；"
    "若当前车速低于设定速度，0.5 秒后交还控制权。当前车距离小于 2 米，或距离小于 4 米且本车比前车快 10 km/h 以上时，"
    "本功能不介入（把刹车交还给系统）。"
  ),
  "LongitudinalPersonality": tr_noop(
    "Standard is recommended. In aggressive mode, sunnypilot will follow lead cars closer and be more aggressive with the gas and brake. " +
    "In relaxed mode sunnypilot will stay further away from lead cars. On supported cars, you can cycle through these personalities with " +
    "your steering wheel distance button."
  ),
  "IsLdwEnabled": tr_noop(
    "Receive alerts to steer back into the lane when your vehicle drifts over a detected lane line " +
    "without a turn signal activated while driving over 31 mph (50 km/h)."
  ),
  "AlwaysOnDM": tr_noop("Enable driver monitoring even when sunnypilot is not engaged."),
  'RecordFront': tr_noop("Upload data from the driver facing camera and help improve the driver monitoring algorithm."),
  "IsMetric": tr_noop("以 km/h 显示速度，而不是 mph。"),
  "RecordAudio": tr_noop("Record and store microphone audio while driving. The audio will be included in the dashcam video in comma connect."),
  "UseMiciLayout": tr_noop("Use the compact UI layout (Comma 4)."),
  "TrafficStopAssist": tr_noop(
    "用驾驶模型自己预测的轨迹作为前方红灯 / 停止标志的证据，据此对一条虚拟停止线制动（该停止线喂给纵向 MPC），"
    "不需要专门的红绿灯识别模型。chill 与 experimental 模式都可用。"
  ),
  "TrafficStopDistanceAdjust": tr_noop(
    "微调车辆相对检测到的停止线的停车位置：正值让停车点前移（更贴近停止线），负值后移。"
    "它叠加在一个固定的「摄像头到车头」修正之上，建议从小幅度开始、按自己车上的实际安装情况调整。"
  ),
}


class TogglesLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._is_release = False  # self._params.get_bool("IsReleaseBranch")

    # 红灯辅助的停位微调控件：在下面"traffic-stop 注册块"里创建（需要 Param 已生效）；
    # 未注册时保持 None，循环里会据此跳过它。
    self._traffic_stop_adjust = None

    # 自定义项按需求「首次运行默认开启」：Params.get_bool() 对不存在的键返回 False
    # （C++ Params::get 不走 default_value 回退），会让 toggle 首次显示成关。
    # 所以在首次运行时显式落一次声明里的默认值；已经有值的（用户改过的）不覆盖。
    for _p in DEFAULT_ON_PARAMS:
      try:
        if self._params.get(_p) is None:
          self._params.put_bool(_p, True, block=True)
      except UnknownKeyName:
        # 该 key 还没在 params_keys.h 里生效（例如 libparams_c.so 未重编）——跳过，
        # 功能侧同样是安全退化。
        pass

    # param, title, desc, icon, needs_restart
    self._toggle_defs = {
      # 「启用 sunnypilot」(OpenpilotEnabledToggle) 按需求隐藏 ⇒ 不注册，列表里不再出现。
      # ⚠️ 隐藏后 UI 上没有总开关（param 本身、功能都不受影响，仍由其它入口控制）。
      # 需要恢复时把下面这块取消注释即可：
      # "OpenpilotEnabledToggle": (
      #   lambda: tr("Enable sunnypilot"),
      #   DESCRIPTIONS["OpenpilotEnabledToggle"],
      #   "chffr_wheel.png",
      #   True,
      # ),
      "ExperimentalMode": (
        lambda: tr("Experimental Mode"),
        "",
        "experimental_white.png",
        False,
      ),
      # 踩油门时暂停/减小模型与系统的减速（see gas_override.py）。
      # 默认开启：参数在 params_keys.h 里声明为 "1"，而 Params.get_bool 对
      # 不存在的键返回 False，所以首次运行前显式落一次默认值（见 __init__）。
      "GasPedalOverride": (
        lambda: tr("踩油门让位"),
        DESCRIPTIONS["GasPedalOverride"],
        "disengage_on_accelerator.png",
        False,
      ),
      # "DisengageOnAccelerator": (
      #   lambda: tr("Disengage on Accelerator Pedal"),
      #   DESCRIPTIONS["DisengageOnAccelerator"],
      #   "disengage_on_accelerator.png",
      #   False,
      # ),
      # "IsLdwEnabled": (
      #   lambda: tr("Enable Lane Departure Warnings"),
      #   DESCRIPTIONS["IsLdwEnabled"],
      #   "warning.png",
      #   False,
      # ),
      # "AlwaysOnDM": (
      #   lambda: tr("Always-On Driver Monitoring"),
      #   DESCRIPTIONS["AlwaysOnDM"],
      #   "monitoring.png",
      #   False,
      # ),
      "RecordFront": (
        lambda: tr("Record and Upload Driver Camera"),
        DESCRIPTIONS["RecordFront"],
        "monitoring.png",
        True,
      ),
      "RecordAudio": (
        lambda: tr("Record and Upload Microphone Audio"),
        DESCRIPTIONS["RecordAudio"],
        "microphone.png",
        True,
      ),
      "IsMetric": (
        lambda: tr("使用公制单位"),
        DESCRIPTIONS["IsMetric"],
        "metric.png",
        False,
      ),
    }

    # 「使用C4界面」(UseMiciLayout) 按需求隐藏 ⇒ 不注册，列表里就不再出现。
    # 需要恢复时把下面这三行取消注释即可（HARDWARE 的 import 已保留）：
    # if HARDWARE.get_device_type() in ("tici", "tizi", "pc"):
    #   self._toggle_defs["UseMiciLayout"] = (
    #     lambda: tr("Use Compact UI Layout"), DESCRIPTIONS["UseMiciLayout"], "settings.png", False,
    #   )

    # ── 红灯 / 停止标志辅助（sunnypilot 追加；机制见 .../traffic_stop.py）──────
    # 它的两个 Params 由 params_keys.h 声明 ⇒ **必须先重编译 libparams_c.so** 才存在。
    # 若 key 尚未生效（例如 .so 还没重编），这里就整个不注册（开关 + 偏移值都不加）：
    # 因为下面 `self._params.get_bool(param)` 对未声明的 key 会抛 UnknownKeyName，
    # 那会把**整个设置页打崩**。功能侧同样是安全退化（traffic_stop.py 用容错读取）。
    try:
      self._params.get_bool("TrafficStopAssist")
    except UnknownKeyName:
      pass
    else:
      # needs_restart=False：参数由控制器自己 1 Hz 轮询 ⇒ 行车中切换即时生效，
      # 不触发 OnroadCycleRequested、也不需要重启。
      self._toggle_defs["TrafficStopAssist"] = (
        lambda: tr("红绿灯/停止标志辅助"),
        DESCRIPTIONS["TrafficStopAssist"],
        "",
        False,
      )
      if option_item_sp is not None:
        # 停位微调（单位分米；±50 dm = ±5.0 m，步长 5 dm = 0.5 m）。
        # ★ 正值 = 停止位置往前移（车头更靠前 / 离停止线更近）；负值 = 往后移。
        #   正负号的实际方向由后端 ADJUST_POSITIVE_MOVES_STOP_FORWARD 决定。
        self._traffic_stop_adjust = option_item_sp(
          title=lambda: tr("红灯停位微调"),
          param="TrafficStopDistanceAdjust",
          min_value=-50,
          max_value=50,
          value_change_step=5,
          label_callback=lambda dm: ("0.0 m" if dm == 0 else f"{dm / 10.0:+.1f} m"),
          description=lambda: tr(DESCRIPTIONS["TrafficStopDistanceAdjust"]),
          icon="",
        )

    self._long_personality_setting = multiple_button_item(
      lambda: tr("Driving Personality"),
      lambda: tr(DESCRIPTIONS["LongitudinalPersonality"]),
      buttons=[lambda: tr("Aggressive"), lambda: tr("Standard"), lambda: tr("Relaxed")],
      button_width=300,
      callback=self._set_longitudinal_personality,
      selected_index=self._params.get("LongitudinalPersonality", return_default=True),
      icon="speed_limit.png"
    )

    # 跟在自己主开关后面渲染的**从属项**（不是纯开关，所以不进 _toggle_defs）。
    FOLLOWERS = {
      "TrafficStopAssist": ("TrafficStopDistanceAdjust", self._traffic_stop_adjust),
      "ExperimentalMode": ("LongitudinalPersonality", self._long_personality_setting),
    }

    self._toggles = {}
    self._locked_toggles = set()

    # 渲染顺序 = 置顶表（实验模式 → 自定义三项）+ 其余按 _toggle_defs 原有顺序（⇒ 其它项整体下移）。
    ordered_params = [p for p in HEAD_TOGGLE_ORDER if p in self._toggle_defs]
    ordered_params += [p for p in self._toggle_defs if p not in ordered_params]

    for param in ordered_params:
      title, desc, icon, needs_restart = self._toggle_defs[param]
      toggle = toggle_item(
        title,
        desc,
        self._params.get_bool(param),
        callback=lambda state, p=param: self._toggle_callback(state, p),
        icon=icon,
      )

      try:
        locked = self._params.get_bool(param + "Lock")
      except UnknownKeyName:
        locked = False
      toggle.action_item.set_enabled(not locked)

      # Make description callable for live translation
      additional_desc = ""
      if needs_restart and not locked:
        additional_desc = tr("Changing this setting will restart sunnypilot if the car is powered on.")
      toggle.set_description(lambda og_desc=toggle.description, add_desc=additional_desc: tr(og_desc) + (" " + tr(add_desc) if add_desc else ""))

      # track for engaged state updates
      if locked:
        self._locked_toggles.add(param)

      self._toggles[param] = toggle

      # 从属项（停位微调 / 驾驶风格）紧跟在自己的主开关后面
      follower = FOLLOWERS.get(param)
      if follower is not None and follower[1] is not None:
        self._toggles[follower[0]] = follower[1]

    self._update_experimental_mode_icon()
    self._scroller = Scroller(list(self._toggles.values()), line_separator=True, spacing=0)

    ui_state.add_engaged_transition_callback(self._update_toggles)

  def _update_state(self):
    if ui_state.sm.updated["selfdriveState"]:
      personality = PERSONALITY_TO_INT[ui_state.sm["selfdriveState"].personality]
      if personality != ui_state.personality and ui_state.started:
        self._long_personality_setting.action_item.set_selected_button(personality)
      ui_state.personality = personality

  def show_event(self):
    super().show_event()
    self._scroller.show_event()
    self._update_toggles()

  def _update_toggles(self):
    ui_state.update_params()

    e2e_description = tr(
      "sunnypilot defaults to driving in chill mode. Experimental mode enables alpha-level features that aren't ready for chill mode. " +
      "Experimental features are listed below:<br>" +
      "<h4>End-to-End Longitudinal Control</h4><br>" +
      "Let the driving model control the gas and brakes. sunnypilot will drive as it thinks a human would, including stopping for red lights and stop signs. " +
      "Since the driving model decides the speed to drive, the set speed will only act as an upper bound. This is an alpha quality feature; " +
      "mistakes should be expected.<br>" +
      "<h4>New Driving Visualization</h4><br>" +
      "The driving visualization will transition to the road-facing wide-angle camera at low speeds to better show some turns. " +
      "The Experimental mode logo will also be shown in the top right corner."
    )

    if ui_state.CP is not None:
      if ui_state.has_longitudinal_control:
        self._toggles["ExperimentalMode"].action_item.set_enabled(True)
        self._toggles["ExperimentalMode"].set_description(e2e_description)
        self._long_personality_setting.action_item.set_enabled(True)
      else:
        # no long for now
        self._toggles["ExperimentalMode"].action_item.set_enabled(False)
        self._toggles["ExperimentalMode"].action_item.set_state(False)
        self._long_personality_setting.action_item.set_enabled(False)
        self._params.remove("ExperimentalMode")

        unavailable = tr("Experimental mode is currently unavailable on this car since the car's stock ACC is used for longitudinal control.")

        long_desc = unavailable + " " + tr("sunnypilot longitudinal control may come in a future update.")
        if ui_state.CP.alphaLongitudinalAvailable:
          if self._is_release:
            long_desc = unavailable + " " + tr("An alpha version of sunnypilot longitudinal control can be tested, along with " +
                                               "Experimental mode, on non-release branches.")
          else:
            long_desc = tr("Enable the sunnypilot longitudinal control (alpha) toggle to allow Experimental mode.")

        self._toggles["ExperimentalMode"].set_description("<b>" + long_desc + "</b><br><br>" + e2e_description)
    else:
      self._toggles["ExperimentalMode"].set_description(e2e_description)

    self._update_experimental_mode_icon()

    # TODO: make a param control list item so we don't need to manage internal state as much here
    # refresh toggles from params to mirror external changes
    for param in self._toggle_defs:
      self._toggles[param].action_item.set_state(self._params.get_bool(param))

    # these toggles need restart, block while engaged
    for toggle_def in self._toggle_defs:
      if self._toggle_defs[toggle_def][3] and toggle_def not in self._locked_toggles:
        self._toggles[toggle_def].action_item.set_enabled(not ui_state.engaged)

    # Block compact layout switching while engaged to avoid mid-drive UI rebuilds
    if "UseMiciLayout" in self._toggles and "UseMiciLayout" not in self._locked_toggles:
      self._toggles["UseMiciLayout"].action_item.set_enabled(not ui_state.engaged)

  def _render(self, rect):
    self._scroller.render(rect)

  def _update_experimental_mode_icon(self):
    icon = "experimental.png" if self._toggles["ExperimentalMode"].action_item.get_state() else "experimental_white.png"
    self._toggles["ExperimentalMode"].set_icon(icon)

  def _handle_experimental_mode_toggle(self, state: bool):
    confirmed = self._params.get_bool("ExperimentalModeConfirmed")
    if state and not confirmed:
      def confirm_callback(result: DialogResult):
        if result == DialogResult.CONFIRM:
          self._params.put_bool("ExperimentalMode", True, block=True)
          self._params.put_bool("ExperimentalModeConfirmed", True, block=True)
        else:
          self._toggles["ExperimentalMode"].action_item.set_state(False)
        self._update_experimental_mode_icon()

      # show confirmation dialog
      content = (f"<h1>{self._toggles['ExperimentalMode'].title}</h1><br>" +
                 f"<p>{self._toggles['ExperimentalMode'].description}</p>")
      dlg = ConfirmDialog(content, tr("Enable"), rich=True, callback=confirm_callback)
      gui_app.push_widget(dlg)
    else:
      self._update_experimental_mode_icon()
      self._params.put_bool("ExperimentalMode", state, block=True)

  def _toggle_callback(self, state: bool, param: str):
    if param == "ExperimentalMode":
      self._handle_experimental_mode_toggle(state)
      return

    self._params.put_bool(param, state, block=True)
    if self._toggle_defs[param][3]:
      self._params.put_bool("OnroadCycleRequested", True, block=True)

  def _set_longitudinal_personality(self, button_index: int):
    self._params.put("LongitudinalPersonality", button_index, block=True)
