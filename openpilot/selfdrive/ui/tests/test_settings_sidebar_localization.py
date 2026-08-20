from types import SimpleNamespace
from unittest import TestCase
from unittest.mock import patch

from openpilot.selfdrive.ui.sunnypilot.layouts.settings.settings import NavButton, PanelInfo


class TestSettingsSidebarLocalization(TestCase):
  def test_nav_button_translates_panel_name_at_render_time(self):
    parent = SimpleNamespace(_current_panel=2, _font_medium=object())
    panel_info = PanelInfo("Device", None, icon="")
    button = NavButton(parent, 1, panel_info)
    drawn_text: list[str] = []

    with patch("openpilot.selfdrive.ui.sunnypilot.layouts.settings.settings.tr", side_effect=lambda text: {"Device": "设备"}.get(text, text)), \
         patch("openpilot.selfdrive.ui.sunnypilot.layouts.settings.settings.measure_text_cached", return_value=SimpleNamespace(y=10)), \
         patch("openpilot.selfdrive.ui.sunnypilot.layouts.settings.settings.rl.draw_text_ex",
               side_effect=lambda _font, text, *_args: drawn_text.append(text)):
      button._render(SimpleNamespace(x=0, y=0, width=300, height=100))

    self.assertEqual(drawn_text, ["设备"])
