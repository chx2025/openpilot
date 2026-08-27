"""Whitelisted settings exposed by the local device settings web UI.

The web server must never provide a generic Params write endpoint: many Params
contain credentials, calibration, or safety state.  This module derives the
normal user-facing settings from sunnypilot's UI schema and adds the local
Tesla/MPC controls that are intentionally maintained outside that schema.
"""
import ast
# ruff: noqa: E501  # Declarative setting rows are intentionally kept one per line.
import json
import math
import re
from pathlib import Path
from typing import Any

from openpilot.common.params import Params
SETTINGS_SCHEMA_PATH = Path(__file__).resolve().parents[2] / "sunnypilot" / "sunnylink" / "settings_ui.json"
TRANSLATIONS_PATH = Path(__file__).resolve().parents[2] / "selfdrive" / "ui" / "translations" / "app_zh-CHS.po"

CATEGORY_TRANSLATIONS = {
  "Steering": "转向", "Cruise": "巡航", "Display": "显示", "Visuals": "视觉", "Toggles": "通用开关",
  "Device": "设备", "Software": "软件", "Developer": "开发者", "Models": "模型", "Vehicle": "车辆",
  "Tesla Settings": "车辆",
  "MADS Settings": "MADS 设置", "Torque Settings": "扭矩设置", "Custom ACC Speed Intervals Settings": "自定义 ACC 速度步长",
  "Speed Limit Settings": "限速设置", "Tesla": "特斯拉",
}
MENU_ORDER = ("设备", "网络", "sunnylink", "通用开关", "软件", "模型", "转向", "巡航", "视觉", "显示", "地图", "行程", "车辆", "开发者")
MENU_CATEGORY_ALIASES = {"特斯拉": "车辆", "Tesla": "车辆", "纵向 MPC": "车辆"}
TITLE_OVERRIDES = {
  "Mads": "启用模块化辅助驾驶（MADS）", "DynamicExperimentalControl": "动态实验控制",
  "DisengageOnAccelerator": "踩加速踏板退出巡航", "CustomAccIncrementsEnabled": "启用自定义 ACC 速度步长",
  "SpeedLimitMode": "限速辅助模式", "SpeedLimitOffsetType": "限速偏移类型", "SpeedLimitValueOffset": "限速偏移值",
  "SmartCruiseControlVision": "视觉", "SmartCruiseControlMap": "地图", "StandstillTimer": "静止计时器",
  "RainbowMode": "Tesla 彩虹模式", "OffroadMode": "强制设置模式", "LanguageSetting": "语言",
  "LateralManeuverMode": "【测试】横向动作模式", "LongitudinalManeuverMode": "【测试】纵向动作模式",
  "CameraOffset": "调整摄像头偏移", "DynamicAutoStock": "动态原车控制",
  "RoadEdgeLaneChangeEnabled": "道路边缘变道保护",
  "ScreenSaverEnabled": "屏幕保护程序",
  "ScreenSaverTimeout": "屏幕保护持续时间",
  "LateralJerkTorqueController": "横向加加速度扭矩控制器",
}
OPTION_OVERRIDES = {
  "Information": "提示", "Car State Only": "仅车辆状态", "Map Data Only": "仅地图数据",
  "Car State Priority": "车辆状态优先", "Map Data Priority": "地图数据优先", "Percentage": "百分比",
  "Always Offroad": "始终设置模式", "Default": "默认",
}
PANEL_CATEGORY_OVERRIDES = {
  "MADS Settings": "转向", "Torque Settings": "转向", "Custom ACC Speed Intervals Settings": "巡航",
  "Speed Limit Settings": "巡航", "Tesla Settings": "特斯拉",
}


def _load_translations() -> dict[str, str]:
  """Read the existing Chinese UI catalog without introducing a PO dependency."""
  translations: dict[str, str] = {}
  msgid: str | None = None
  msgstr: str | None = None
  state: str | None = None
  for raw_line in TRANSLATIONS_PATH.read_text().splitlines():
    line = raw_line.strip()
    if line.startswith("msgid "):
      if msgid and msgstr:
        translations[msgid] = msgstr
      msgid, msgstr, state = ast.literal_eval(line[6:]), "", "id"
    elif line.startswith("msgstr "):
      msgstr, state = ast.literal_eval(line[7:]), "str"
    elif line.startswith('"') and state is not None:
      if state == "id":
        msgid = (msgid or "") + ast.literal_eval(line)
      else:
        msgstr = (msgstr or "") + ast.literal_eval(line)
    elif not line:
      if msgid and msgstr:
        translations[msgid] = msgstr
      msgid = msgstr = state = None
  if msgid and msgstr:
    translations[msgid] = msgstr
  return translations


TRANSLATIONS = _load_translations()


def _translate(text: str, key: str = "", description: bool = False) -> str:
  if not text:
    return ""
  if translated := TRANSLATIONS.get(text):
    return translated
  if not description and key in TITLE_OVERRIDES:
    return TITLE_OVERRIDES[key]
  if not description and text in OPTION_OVERRIDES:
    return OPTION_OVERRIDES[text]
  # Omit untranslated prose instead of mixing English into the Chinese UI.
  return "" if description else text


def _translate_option(text: str, key: str) -> str:
  translated = _translate(text, key)
  if translated != text:
    return translated
  if match := re.fullmatch(r"(\d+(?:\.\d+)?) seconds?", text):
    return f"{match.group(1)} 秒"
  if match := re.fullmatch(r"(\d+) ?s", text):
    return f"{match.group(1)} 秒"
  if match := re.fullmatch(r"(\d+) ?m", text):
    return f"{match.group(1)} 分钟"
  if match := re.fullmatch(r"(\d+)h", text):
    return f"{match.group(1)} 小时"
  if text == "30h (Default)":
    return "30 小时（默认）"
  if text == "CrazyMax":
    return "CrazyMax"
  return text

# These controls intentionally have no SunnyLink schema entry. Keep the local
# console whitelist narrow: it is not a generic Params editor.
EXTRA_SETTINGS: tuple[dict[str, Any], ...] = (
  {"key": "Brightness", "widget": "option", "title": "停车界面亮度", "category": "Display", "group": "显示", "min": 10, "max": 100, "step": 10, "unit": "%", "offroad_only": True},
  {"key": "TeslaARS408Radar", "widget": "multiple_button", "title": "Tesla 雷达后端", "category": "Tesla", "group": "Tesla", "options": [{"value": 0, "label": "原车"}, {"value": 1, "label": "ARS408"}, {"value": 2, "label": "关闭"}], "offroad_only": True},
  {"key": "DynamicAutoStock", "widget": "toggle", "title": "动态原车 ACC", "category": "Tesla", "group": "Tesla", "offroad_only": True},
  {"key": "TeslaApHybrid", "widget": "toggle", "title": "Tesla AP 混合控制", "category": "Tesla", "group": "Tesla", "offroad_only": True},
  {"key": "TeslaDynamicApLongitudinal", "widget": "toggle", "title": "Tesla 动态 AP 纵向", "group": "Tesla", "offroad_only": True},
  {"key": "DynamicAutoStockBlinkerToSP", "widget": "toggle", "title": "动态原车：转向灯切换 SP", "group": "Tesla", "offroad_only": True},
  {"key": "DynamicAutoStockCurveToSP", "widget": "toggle", "title": "动态原车：弯道切换 SP", "group": "Tesla", "offroad_only": True},
  {"key": "DynamicAutoStockSpeedKph", "widget": "option", "title": "动态原车切换速度", "group": "Tesla", "min": 0, "max": 200, "step": 1, "unit": "km/h", "offroad_only": True},
  {"key": "DynamicAutoStockSpeedLowKph", "widget": "option", "title": "动态原车回切速度", "group": "Tesla", "min": 0, "max": 200, "step": 1, "unit": "km/h", "offroad_only": True},
  {"key": "TeslaMadsScreenButton", "widget": "multiple_button", "title": "Tesla MADS 屏幕手势", "group": "Tesla", "options": [{"value": 0, "label": "关闭"}, {"value": 1, "label": "三指"}, {"value": 2, "label": "五指"}], "offroad_only": True},
  {"key": "TeslaTouchLongitudinalSwitch", "widget": "toggle", "title": "4指触摸切换原车ACC", "group": "Tesla", "offroad_only": True},
  {"key": "TeslaTurnSignalValidation", "widget": "toggle", "title": "启用 Tesla 转向 CAN 测试", "category": "Developer", "group": "Tesla 测试", "offroad_only": True},
  {"key": "TeslaSpeedButtonValidation", "widget": "toggle", "title": "启用 Tesla 速度按钮测试", "category": "Developer", "group": "Tesla 测试", "offroad_only": True},
  {"key": "WebTerminalEnabled", "widget": "toggle", "title": "启用网页终端（高风险）", "category": "Developer", "group": "本地网页控制台", "offroad_only": True},
  {"key": "RecordRoadVideo", "widget": "toggle", "title": "录制前向道路视频", "category": "Developer", "group": "路线记录", "offroad_only": True},
  {"key": "LongitudinalPlannerMode", "widget": "multiple_button", "title": "纵向规划器", "group": "纵向 MPC", "options": [{"value": 0, "label": "官方（默认）"}, {"value": 1, "label": "实验"}, {"value": 2, "label": "TN-NoDEC"}], "offroad_only": True},
  {"key": "AccelPersonalityEnabled", "widget": "toggle", "title": "TN 加速个性控制", "group": "纵向 MPC", "offroad_only": False},
  {"key": "AccelPersonality", "widget": "multiple_button", "title": "TN 加速个性", "group": "纵向 MPC", "options": [{"value": 0, "label": "Eco"}, {"value": 1, "label": "Normal"}, {"value": 2, "label": "Sport"}], "offroad_only": False},
)
VEHICLE_SETTING_BRANDS = {
  "HyundaiLongitudinalTuning": "hyundai", "SubaruStopAndGo": "subaru",
  "SubaruStopAndGoManualParkingBrake": "subaru", "ToyotaEnforceStockLongitudinal": "toyota",
  "ToyotaStopAndGoHack": "toyota", "TeslaCoopSteering": "tesla", "DynamicAutoStock": "tesla",
  "StopLineDeceleration": "tesla",
}


def _has_offroad_only(value: Any) -> bool:
  if isinstance(value, dict):
    if value.get("type") == "offroad_only":
      return True
    return any(_has_offroad_only(child) for child in value.values())
  if isinstance(value, list):
    return any(_has_offroad_only(child) for child in value)
  return False


def _schema_settings() -> list[dict[str, Any]]:
  schema = json.loads(SETTINGS_SCHEMA_PATH.read_text())
  settings: list[dict[str, Any]] = []

  def walk(value: Any, panel: str = "通用", section: str = "") -> None:
    if isinstance(value, dict):
      current_panel = value.get("label", panel)
      current_section = value.get("title", section)
      if "key" in value and value.get("widget") in {"toggle", "option", "multiple_button"}:
        setting = {k: value[k] for k in ("key", "widget", "min", "max", "step", "unit", "value_map", "needs_onroad_cycle") if k in value}
        setting["title"] = _translate(value.get("title", value["key"]), value["key"])
        setting["description"] = _translate(value.get("description", ""), value["key"], description=True)
        if "details" in value:
          setting["details"] = _translate(value["details"], value["key"], description=True)
        if "options" in value:
          setting["options"] = [{**option, "label": _translate_option(option["label"], value["key"])} for option in value["options"]]
        setting["category"] = PANEL_CATEGORY_OVERRIDES.get(panel, CATEGORY_TRANSLATIONS.get(panel, panel))
        if panel == "通用" and section in CATEGORY_TRANSLATIONS:
          setting["category"] = CATEGORY_TRANSLATIONS[section]
        translated_section = TRANSLATIONS.get(section, "")
        setting["group"] = translated_section or setting["category"]
        setting["offroad_only"] = _has_offroad_only(value.get("enablement", []))
        settings.append(setting)
      for child_key, child in value.items():
        if child_key not in {"options", "enablement", "visibility", "trigger_condition"}:
          walk(child, current_panel, current_section)
    elif isinstance(value, list):
      for child in value:
        walk(child, panel, section)

  walk(schema)
  return settings


def get_settings(brand: str | None = None) -> dict[str, dict[str, Any]]:
  settings = _schema_settings()
  settings.extend(EXTRA_SETTINGS)
  # Duplicate keys in a nested schema are harmless; retain the first canonical definition.
  for setting in settings:
    setting["category"] = CATEGORY_TRANSLATIONS.get(setting.get("category", setting["group"]), setting.get("category", setting["group"]))
    setting["group"] = CATEGORY_TRANSLATIONS.get(setting["group"], setting["group"])
    setting["category"] = MENU_CATEGORY_ALIASES.get(setting["category"], setting["category"])
  return {
    setting["key"]: setting for setting in settings
    if not brand or VEHICLE_SETTING_BRANDS.get(setting["key"], brand) == brand
  }


def _current_brand(params: Params) -> str:
  value = params.get("CarPlatformBundle")
  if isinstance(value, str):
    try:
      value = json.loads(value)
    except json.JSONDecodeError:
      return ""
  return str(value.get("brand", "")).lower() if isinstance(value, dict) else ""


def _read_value(params: Params, setting: dict[str, Any]) -> bool | int | float | str:
  if setting["widget"] == "toggle":
    return params.get_bool(setting["key"])
  value = params.get(setting["key"], return_default=True)
  options = setting.get("options")
  if options:
    allowed_values = {option["value"] for option in options}
    if "" in allowed_values and not value:
      return ""
    if any(isinstance(option, float) for option in allowed_values):
      return float(value) if value is not None else 0.0
  if not isinstance(setting.get("step"), int):
    return float(value) if value is not None else 0.0
  return int(value) if value is not None else 0


def settings_snapshot(params: Params | None = None) -> dict[str, Any]:
  params = params or Params()
  settings = get_settings(_current_brand(params))
  visible_settings = [
    {**setting, "value": _read_value(params, setting), "order": order,
     "enabled": True}
    for order, setting in enumerate(settings.values())
  ]
  return {
    "onroad": not params.get_bool("IsOffroad"),
    "menu": [category for category in MENU_ORDER if any(setting["category"] == category for setting in visible_settings)],
    "settings": visible_settings,
  }


def validate_and_write(key: str, value: Any, params: Params | None = None) -> dict[str, Any]:
  params = params or Params()
  setting = get_settings(_current_brand(params)).get(key)
  if setting is None:
    raise KeyError(key)
  if setting["offroad_only"] and not params.get_bool("IsOffroad"):
    raise PermissionError("该设置只能在停车后的设置模式修改")
  if setting["widget"] == "toggle":
    if not isinstance(value, bool):
      raise ValueError("开关值必须是 true 或 false")
    params.put_bool(key, value, block=True)
  else:
    options = setting.get("options")
    if options is not None:
      allowed_values = {option["value"] for option in options}
      if value not in allowed_values:
        raise ValueError("不支持的选项值")
    else:
      if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("数值设置必须是有限数字")
      if isinstance(setting.get("step"), int):
        if int(value) != value:
          raise ValueError("该设置只接受整数")
        value = int(value)
      else:
        value = float(value)
    minimum, maximum = setting.get("min"), setting.get("max")
    if minimum is not None and not minimum <= value <= maximum:
      raise ValueError(f"数值必须在 {minimum} 到 {maximum} 之间")
    step = setting.get("step")
    if step and abs(round(value / step) * step - value) > 1e-9:
      raise ValueError(f"数值必须按 {step} 递增")
    params.put(key, value, block=True)
  return {**setting, "value": _read_value(params, setting)}
