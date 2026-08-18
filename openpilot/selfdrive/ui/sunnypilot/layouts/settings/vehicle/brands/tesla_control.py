"""Tesla control-profile settings.

This page owns only settings implemented by the optional Tesla control module.
Keeping it separate from the upstream Tesla brand page makes future upstream UI
updates a small, single-import merge instead of a whole-file conflict.
"""

from collections.abc import Callable

import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.selfdrive.car.tesla.control_profile import normalize_mads_screen_button
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.registry import BackendId, ordered_backends
from openpilot.sunnypilot.selfdrive.traffic_control import TrafficControlMode, configured_mode, planner_session_is_active
from openpilot.selfdrive.debug.device_console_auth import ensure_console_token, rotate_console_token
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.list_view import button_item_sp, multiple_button_item_sp, option_item_sp, toggle_item_sp
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.network import NavButton
from openpilot.system.ui.widgets.scroller_tici import Scroller


class TeslaControlSettingsLayout(Widget):
  def __init__(self, back_btn_callback: Callable):
    super().__init__()
    self._back_button = NavButton(tr("Back"))
    self._back_button.set_click_callback(back_btn_callback)
    self._planner_backends = ordered_backends()

    self.planner_backend = multiple_button_item_sp(
      title=lambda: tr("Longitudinal Planner"),
      description=lambda: tr("Official follows the current dev planner. TN-NoDEC is isolated and ignores Dynamic Experimental Control."),
      buttons=[lambda label=backend.label: tr(label) for backend in self._planner_backends],
      callback=self._set_planner_backend,
      inline=False,
    )
    self.accel_personality_enabled = toggle_item_sp(
      title=tr("TN Accel Personality"),
      param="AccelPersonalityEnabled",
      description=tr("Enable TN's acceleration profile controller."),
    )
    self.accel_personality = multiple_button_item_sp(
      title=lambda: tr("TN Accel Profile"),
      description=lambda: tr("Choose Eco, Normal, or Sport acceleration limits for TN-NoDEC."),
      buttons=[lambda: tr("Eco"), lambda: tr("Normal"), lambda: tr("Sport")],
      param="AccelPersonality",
      inline=False,
    )

    self.touch_longitudinal_switch = toggle_item_sp(
      title=tr("4-Finger Longitudinal Switch"),
      param="TeslaTouchLongitudinalSwitch",
      description=tr("Use a 4-finger infotainment press to switch longitudinal control between sunnypilot and Tesla ACC."),
      enabled=ui_state.is_offroad,
    )
    self.ap_hybrid = toggle_item_sp(
      title=tr("AP Hybrid Control (Experimental)"),
      param="TeslaApHybrid",
      description=tr("Keep a Tesla AP session available while sunnypilot arbitrates lateral and longitudinal control."),
      enabled=ui_state.is_offroad,
    )
    self.dynamic_ap_longitudinal = toggle_item_sp(
      title=tr("Dynamic AP Control (Experimental)"),
      param="TeslaDynamicApLongitudinal",
      description=tr("Use speed hysteresis to select both Tesla AP control axes or sunnypilot control axes."),
      enabled=ui_state.is_offroad,
    )
    self.dynamic_auto_stock = toggle_item_sp(
      title=tr("Dynamic Auto Stock ACC"),
      param="DynamicAutoStock",
      description=tr("Select Tesla ACC above the high-speed threshold when the stock longitudinal handoff is ready."),
      enabled=ui_state.is_offroad,
    )
    self.blinker_to_sp = toggle_item_sp(
      title=tr("Turn Signal → SP Longitudinal"),
      param="DynamicAutoStockBlinkerToSP",
      description=tr("Return Dynamic Auto Stock ACC to sunnypilot after a confirmed turn signal."),
      enabled=ui_state.is_offroad,
    )
    self.curve_to_sp = toggle_item_sp(
      title=tr("Curve → SP Longitudinal"),
      param="DynamicAutoStockCurveToSP",
      description=tr("Return Dynamic Auto Stock ACC to sunnypilot when vision or map curve control becomes active."),
      enabled=ui_state.is_offroad,
    )
    self.speed_high = option_item_sp(
      title=tr("Speed Threshold High"),
      param="DynamicAutoStockSpeedKph",
      min_value=40,
      max_value=120,
      value_change_step=5,
      label_callback=lambda value: f"{value} km/h",
      description=tr("Allow a configured dynamic mode to select Tesla control above this speed."),
    )
    self.speed_low = option_item_sp(
      title=tr("Speed Threshold Low"),
      param="DynamicAutoStockSpeedLowKph",
      min_value=20,
      max_value=100,
      value_change_step=5,
      label_callback=lambda value: f"{value} km/h",
      description=tr("Return a configured dynamic mode to sunnypilot below this speed."),
    )
    self.traffic_control_mode = multiple_button_item_sp(
      title=lambda: tr("Tesla Traffic Control"),
      description=lambda: tr("Observe and Shadow never alter control. Stop and Stop/Go are experimental closed-course constraints."),
      buttons=[lambda: tr("Off"), lambda: tr("Observe"), lambda: tr("Shadow"), lambda: tr("Stop"), lambda: tr("Stop/Go")],
      param="TeslaTrafficControlMode",
      inline=False,
    )
    self.traffic_stop_reference = option_item_sp(
      title=tr("Traffic Stop Reference"),
      param="TeslaTrafficStopReference",
      min_value=20,
      max_value=120,
      value_change_step=5,
      label_callback=lambda value: f"{value / 10.0:.1f} m",
      description=tr("Distance between the OEM traffic-control target and the desired stopping point."),
    )
    self.traffic_adaptive_reference = toggle_item_sp(
      title=tr("Adaptive Stop Reference"),
      param="TeslaTrafficAdaptiveReference",
      description=tr("Use a confirmed model stop to slowly adapt the OEM target reference."),
    )
    ensure_console_token(ui_state.params)
    self.device_console = toggle_item_sp(
      title=tr("Local Device Console"),
      param="DeviceConsoleEnabled",
      description=tr("Expose the authenticated settings, hotspot, driving status, and Tesla validation page on the local network."),
    )
    self.console_token = button_item_sp(
      title=tr("Device Console Access Token"),
      button_text=tr("Rotate"),
      description=lambda: f"http://192.168.43.1:8088\n{ensure_console_token(ui_state.params)}",
      callback=lambda: rotate_console_token(ui_state.params),
    )
    self.web_terminal = toggle_item_sp(
      title=tr("Arbitrary Web Terminal (High Risk)"),
      param="WebTerminalEnabled",
      description=tr("Allow arbitrary commands only while offroad, authenticated by the device console token."),
    )
    self.web_driving_visualization = toggle_item_sp(
      title=tr("Browser Driving and HW4 View"),
      param="TeslaWebDrivingVisualization",
      description=tr("Publish a read-only browser view of driving state and Tesla CAN/HW4 perception."),
    )
    self.turn_validation = toggle_item_sp(
      title=tr("Tesla Turn Signal Validation"),
      param="TeslaTurnSignalValidation",
      description=tr("Allow authenticated validation using fresh OEM 0x3E9 templates. Restart after changing."),
    )
    self.speed_validation = toggle_item_sp(
      title=tr("Tesla Speed Button Validation"),
      param="TeslaSpeedButtonValidation",
      description=tr("Allow authenticated validation using fresh OEM 0x3C2 templates. Restart after changing."),
    )
    self.external_buzzer = toggle_item_sp(
      title=tr("C3XL GPIO42 Buzzer"),
      param="ExternalBuzzerEnabled",
      description=tr("Use the C3XL hardware profile's external alert output."),
    )
    self.custom_alert_sounds = toggle_item_sp(
      title=tr("C3XL Custom Alert Sounds"),
      param="CustomAlertSounds",
      description=tr("Use the C3XL engage and disengage sound profile after restart."),
    )

    self.items = [
      self.planner_backend,
      self.accel_personality_enabled,
      self.accel_personality,
      self.touch_longitudinal_switch,
      self.ap_hybrid,
      self.dynamic_ap_longitudinal,
      self.dynamic_auto_stock,
      self.blinker_to_sp,
      self.curve_to_sp,
      self.speed_high,
      self.speed_low,
      self.traffic_control_mode,
      self.traffic_stop_reference,
      self.traffic_adaptive_reference,
      self.device_console,
      self.console_token,
      self.web_terminal,
      self.web_driving_visualization,
      self.turn_validation,
      self.speed_validation,
      self.external_buzzer,
      self.custom_alert_sounds,
    ]
    self._scroller = Scroller(self.items, line_separator=True, spacing=0)

  def _planner_backend_index(self) -> int:
    configured = int(ui_state.params.get("LongitudinalPlannerMode", return_default=True))
    return next((index for index, backend in enumerate(self._planner_backends) if backend.id == configured), 0)

  def _set_planner_backend(self, index: int) -> None:
    if 0 <= index < len(self._planner_backends):
      ui_state.params.put("LongitudinalPlannerMode", int(self._planner_backends[index].id), block=True)

  def _update_visibility(self) -> None:
    dynamic_stock = ui_state.params.get_bool("DynamicAutoStock")
    dynamic_ap = ui_state.params.get_bool("TeslaDynamicApLongitudinal")
    ap_hybrid = ui_state.params.get_bool("TeslaApHybrid")
    tn_selected = self._planner_backends[self._planner_backend_index()].id == BackendId.TN_NO_DEC

    self.accel_personality_enabled.set_visible(tn_selected)
    self.accel_personality.set_visible(tn_selected and ui_state.params.get_bool("AccelPersonalityEnabled"))
    self.dynamic_ap_longitudinal.set_visible(ap_hybrid)
    self.blinker_to_sp.set_visible(dynamic_stock)
    self.curve_to_sp.set_visible(dynamic_stock)
    self.speed_high.set_visible(dynamic_stock or dynamic_ap)
    self.speed_low.set_visible(dynamic_stock or dynamic_ap)
    traffic_mode = configured_mode(ui_state.params)
    self.traffic_stop_reference.set_visible(traffic_mode in (TrafficControlMode.stopOnly, TrafficControlMode.stopGo))
    self.traffic_adaptive_reference.set_visible(traffic_mode in (TrafficControlMode.stopOnly, TrafficControlMode.stopGo))

  def _update_state(self):
    super()._update_state()
    offroad = ui_state.is_offroad()
    has_longitudinal = ui_state.has_longitudinal_control

    for item in (self.touch_longitudinal_switch, self.dynamic_auto_stock,
                 self.blinker_to_sp, self.curve_to_sp):
      item.action_item.set_enabled(offroad and has_longitudinal)
    self.ap_hybrid.action_item.set_enabled(offroad and has_longitudinal)
    self.dynamic_ap_longitudinal.action_item.set_enabled(offroad and has_longitudinal)
    self.speed_high.action_item.set_enabled(offroad)
    self.speed_low.action_item.set_enabled(offroad)
    planner_stopped = not planner_session_is_active(ui_state.sm)
    self.planner_backend.action_item.selected_button = self._planner_backend_index()
    self.planner_backend.action_item.set_enabled(planner_stopped)
    self.traffic_control_mode.action_item.set_enabled(planner_stopped)
    self.traffic_control_mode.action_item.set_enabled_buttons(None if has_longitudinal else {0, 1, 2})
    self.traffic_stop_reference.action_item.set_enabled(planner_stopped)
    self.traffic_adaptive_reference.action_item.set_enabled(planner_stopped)
    for item in (self.device_console, self.console_token, self.web_terminal,
                 self.web_driving_visualization, self.turn_validation,
                 self.speed_validation, self.external_buzzer, self.custom_alert_sounds):
      item.action_item.set_enabled(offroad)
    self._update_visibility()

  def _render(self, rect):
    self._back_button.set_position(self._rect.x, self._rect.y + 20)
    self._back_button.render()
    content_rect = rl.Rectangle(
      rect.x,
      rect.y + self._back_button.rect.height + 40,
      rect.width,
      rect.height - self._back_button.rect.height - 40,
    )
    self._scroller.render(content_rect)

  def show_event(self):
    self._update_visibility()
    self._scroller.show_event()


class TeslaControlSettingsAdapter:
  """The complete UI seam consumed by the upstream Tesla brand page."""

  def __init__(self):
    raw_screen_button = ui_state.params.get("TeslaMadsScreenButton", return_default=True)
    screen_button = normalize_mads_screen_button(raw_screen_button)
    if screen_button != raw_screen_button:
      ui_state.params.put("TeslaMadsScreenButton", screen_button, block=True)

    self.radar_backend = multiple_button_item_sp(
      title=lambda: tr("Tesla Radar Backend"),
      description=lambda: tr("Select Tesla radar, an external Continental ARS408, or disable radar input. Restart after changing."),
      buttons=[lambda: tr("OEM"), lambda: tr("ARS408"), lambda: tr("Off")],
      param="TeslaARS408Radar",
      inline=False,
    )
    self._settings_layout = TeslaControlSettingsLayout(lambda: gui_app.pop_widget())
    self.settings_button = button_item_sp(
      title=tr("Tesla Control Profile"),
      button_text=tr("Customize"),
      description=tr("Configure longitudinal handoff, Dynamic Auto Stock ACC, and AP Hybrid control."),
      callback=lambda: gui_app.push_widget(self._settings_layout),
    )

  def update_settings(self) -> None:
    self.radar_backend.action_item.set_enabled(ui_state.is_offroad())
