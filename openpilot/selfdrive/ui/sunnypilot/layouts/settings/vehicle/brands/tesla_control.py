"""Tesla control-profile settings.

This page owns only settings implemented by the optional Tesla control module.
Keeping it separate from the upstream Tesla brand page makes future upstream UI
updates a small, single-import merge instead of a whole-file conflict.
"""

from collections.abc import Callable

import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.selfdrive.car.tesla.control_profile import normalize_mads_screen_button
from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.tesla_planner import TeslaPlannerSettingsLayout
from openpilot.sunnypilot.selfdrive.traffic_control import TRAFFIC_SIGNAL_CONTROL_PARAM, planner_session_is_active
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
    self._planner_settings = TeslaPlannerSettingsLayout(lambda: gui_app.pop_widget())
    self.planner_settings = button_item_sp(
      title=tr("Longitudinal Planner"), button_text=tr("Customize"),
      description=tr("Select and independently tune Official, Experimental, or TN-NoDEC."),
      callback=lambda: gui_app.push_widget(self._planner_settings),
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
    self.turn_signal_validation = toggle_item_sp(
      title=tr("Tesla Turn Signal CAN Test"),
      param="TeslaTurnSignalValidation",
      description=tr("Allow the local browser to send bounded turn-signal validation frames using fresh OEM templates. Restart after changing."),
      enabled=ui_state.is_offroad,
    )
    self.speed_button_validation = toggle_item_sp(
      title=tr("Tesla Speed Button CAN Test"),
      param="TeslaSpeedButtonValidation",
      description=tr("Allow the local browser to send one bounded speed-button validation tick using a fresh OEM template. Restart after changing."),
      enabled=ui_state.is_offroad,
    )
    self.items = [
      self.planner_settings,
      self.touch_longitudinal_switch,
      self.ap_hybrid,
      self.dynamic_ap_longitudinal,
      self.dynamic_auto_stock,
      self.blinker_to_sp,
      self.curve_to_sp,
      self.speed_high,
      self.speed_low,
      self.turn_signal_validation,
      self.speed_button_validation,
    ]
    self._scroller = Scroller(self.items, line_separator=True, spacing=0)

  def _update_visibility(self) -> None:
    dynamic_stock = ui_state.params.get_bool("DynamicAutoStock")
    dynamic_ap = ui_state.params.get_bool("TeslaDynamicApLongitudinal")
    ap_hybrid = ui_state.params.get_bool("TeslaApHybrid")
    self.dynamic_ap_longitudinal.set_visible(ap_hybrid)
    self.blinker_to_sp.set_visible(dynamic_stock)
    self.curve_to_sp.set_visible(dynamic_stock)
    self.speed_high.set_visible(dynamic_stock or dynamic_ap)
    self.speed_low.set_visible(dynamic_stock or dynamic_ap)

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
    self.turn_signal_validation.action_item.set_enabled(offroad)
    self.speed_button_validation.action_item.set_enabled(offroad)
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
    self.traffic_control_mode = toggle_item_sp(
      title=tr("Traffic Light Control (Experimental)"),
      description=tr("When off, traffic-light data is recorded in the background without changing control. When on, confirmed red lights can stop the vehicle and confirmed green lights can start it when the path is clear."),  # noqa: E501
      param=TRAFFIC_SIGNAL_CONTROL_PARAM,
    )
    self.traffic_stop_reference = option_item_sp(
      title=tr("Traffic Light Stop Reference"),
      param="TeslaTrafficStopReference",
      min_value=20,
      max_value=120,
      value_change_step=5,
      label_callback=lambda value: f"{value / 10.0:.1f} m",
      description=tr("Adjust how far before Tesla's reported traffic-control point the vehicle stops. Higher values stop earlier; lower values stop closer. Changes apply to the next traffic-light event without restarting."),  # noqa: E501
    )
    self.traffic_control_max_speed = option_item_sp(
      title=tr("Traffic Light Control Maximum Speed"),
      param="TeslaTrafficControlMaxSpeed",
      min_value=20,
      max_value=120,
      value_change_step=5,
      label_callback=lambda value: f"{value} km/h",
      description=tr("Do not establish a new traffic-light control event above this speed. Existing braking is never cancelled abruptly, and changes apply without restarting."),  # noqa: E501
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
    planner_stopped = not planner_session_is_active(ui_state.sm)
    self.traffic_control_mode.action_item.set_enabled(planner_stopped and ui_state.has_longitudinal_control)
    self.traffic_stop_reference.action_item.set_enabled(ui_state.has_longitudinal_control)
    self.traffic_control_max_speed.action_item.set_enabled(ui_state.has_longitudinal_control)
