import pytest

from openpilot.selfdrive.ui.onroad.alert_localizer import localize_alert_text


def test_simplified_chinese_restores_legacy_static_sp_alert():
  text1, text2 = localize_alert_text(
    "manualSteeringRequired/userDisable",
    "Automatic Lane Centering is OFF",
    "Manual Steering Required",
    "zh-CHS",
  )

  assert text1 == "自动车道居中功能已关闭"
  assert text2 == "请手动控制方向"


def test_simplified_chinese_formats_dynamic_speed_limit_target():
  text1, text2 = localize_alert_text(
    "speedLimitPreActive/warning",
    "Speed Limit Assist: set to 70 km/h to engage",
    "",
    "zh-CHS",
  )

  assert text1 == "限速辅助：手动将设定速度更改为 70 km/h 以激活"
  assert text2 == ""


def test_simplified_chinese_formats_dynamic_storage_percentage():
  text1, text2 = localize_alert_text(
    "outOfSpace/permanent",
    "Out of Storage",
    "83% full",
    "zh-CHS",
  )

  assert text1 == "存储空间不足"
  assert text2 == "83% 已使用"


def test_language_switch_does_not_change_english_or_other_languages():
  alert = (
    "manualSteeringRequired/userDisable",
    "Automatic Lane Centering is OFF",
    "Manual Steering Required",
  )

  assert localize_alert_text(*alert, "zh-CHS") == ("自动车道居中功能已关闭", "请手动控制方向")
  assert localize_alert_text(*alert, "en") == alert[1:]
  assert localize_alert_text(*alert, "de") == alert[1:]


def test_same_upstream_text_keeps_legacy_generic_and_sp_context():
  generic = localize_alert_text("parkBrake/noEntry", "openpilot Unavailable", "Parking Brake Engaged", "zh-CHS")
  sp = localize_alert_text("silentParkBrake/noEntry", "openpilot Unavailable", "Parking Brake Engaged", "zh-CHS")

  assert generic == ("openpilot Unavailable", "正在使用驻车制动")
  assert sp == ("openpilot Unavailable", "驻车制动已启用")


@pytest.mark.parametrize(("alert_type", "english", "chinese"), [
  ("belowEngageSpeed/noEntry", "Drive above 13 km/h to engage", "请保持 13 km/h 以上速度行驶以启用辅助驾驶"),
  ("calibrationIncomplete/permanent", "Calibrating: 42%", "自动校准 进行中: 42%"),
  ("calibrationIncomplete/permanent", "Drive Above 15 km/h", "请保持车速高于 15 km/h"),
  ("posenetInvalid/noEntry", "Speed Error: -1.2 m/s", "车速异常: -1.2 m/s"),
  ("calibrationInvalid/permanent", "Remount Device (Pitch: 1.2°, Yaw: -0.5°)", "请调整设备安装 (Pitch: 1.2°, Yaw: -0.5°)"),
  ("paramsdTemporaryError/noEntry", "Angle offset too high (Offset: 2.3°)", "角度偏移过大 (偏移: 2.3°)"),
  ("lowMemory/permanent", "74% used", "74% 已使用"),
  ("personalityChanged/warning", "Driving Personality: Aggressive", "驾驶风格: Aggressive"),
])
def test_simplified_chinese_restores_legacy_dynamic_generic_alerts(alert_type, english, chinese):
  assert localize_alert_text(alert_type, english, "", "zh-CHS") == (chinese, "")
