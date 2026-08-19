import pytest
import re

from openpilot.selfdrive.selfdrived.events import EVENTS, EVENT_NAME
from openpilot.selfdrive.ui.onroad.alert_localizer import localize_alert_text
from openpilot.sunnypilot.selfdrive.selfdrived.events import EVENTS_SP, EVENT_NAME_SP
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import Alert


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

  assert generic == ("openpilot 暂不可用", "正在使用驻车制动")
  assert sp == ("openpilot 暂不可用", "驻车制动已启用")


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


@pytest.mark.parametrize(("alert_type", "english", "chinese"), [
  ("modeldLagging/permanent", "Driving Model Lagging", "驾驶模型运行延迟"),
  ("modeldLagging/permanent", "12.3% frames dropped", "已丢弃 12.3% 的帧"),
  ("joystickDebug/warning", "Joystick Mode", "摇杆模式"),
  ("joystickDebug/warning", "Gas: 25%, Steer: -10%", "油门: 25%，转向: -10%"),
  ("tooDistracted/noEntry", "Too Distracted", "过度分心"),
  ("tooDistracted/noEntry", "1 minute Left", "剩余 1 分钟"),
  ("tooDistracted/noEntry", "5 minutes Left", "剩余 5 分钟"),
  ("tooDistracted/noEntry", "Pay Attention to Engage", "请集中注意力后再启用"),
  ("selfdrivedLagging/softDisable", "System Lagging", "系统运行延迟"),
  ("wrongCarMode/noEntry", "Enable Adaptive Cruise to Engage", "启用自适应巡航后再启用辅助驾驶"),
  ("wrongCarMode/noEntry", "Enable Main Switch to Engage", "开启巡航主开关后再启用辅助驾驶"),
  ("wrongGear/softDisable", "openpilot will disengage", "openpilot 即将退出"),
])
def test_simplified_chinese_localizes_current_dynamic_alerts(alert_type, english, chinese):
  assert localize_alert_text(alert_type, english, "", "zh-CHS") == (chinese, "")


def test_untested_branch_alert_keeps_explicit_risk_wording():
  assert localize_alert_text(
    "startupMaster/permanent", "WARNING: This branch is not tested", "dev-sp-egpu", "zh-CHS",
  ) == ("警告：此分支未经测试", "dev-sp-egpu")


def test_simplified_chinese_localizes_every_current_static_onroad_alert():
  untranslated = []
  for events, event_names in ((EVENTS, EVENT_NAME), (EVENTS_SP, EVENT_NAME_SP)):
    for event, alerts_by_type in events.items():
      for event_type, alert in alerts_by_type.items():
        if not isinstance(alert, Alert):
          continue

        alert_type = f"{event_names[event]}/{event_type}"
        localized = localize_alert_text(alert_type, alert.alert_text_1, alert.alert_text_2, "zh-CHS")
        for source, translated in zip((alert.alert_text_1, alert.alert_text_2), localized, strict=True):
          if source and re.search(r"[A-Za-z]", source) and translated == source:
            untranslated.append((alert_type, source))

  assert not untranslated, f"Untranslated static onroad alerts: {untranslated}"
