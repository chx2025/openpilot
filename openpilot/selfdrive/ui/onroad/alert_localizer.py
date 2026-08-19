"""Language adapter for alert text generated outside of the UI translation catalog."""

import re

_ZH_CHS_EXACT = {
  "Adaptive Cruise Disabled": "自适应巡航已禁用",
  "Auto adjusting to last speed limit": "正在自动调整至上个限速值",
  "Auto adjusting to speed limit": "正在自动调整至当前道路限速",
  "Automatic Lane Centering is OFF": "自动车道居中功能已关闭",
  "Be ready to take over at any time": "请随时准备接管",
  "Big Model Failed ": "大模型运行失败",
  "Big Model Loading": "大模型正在加载",
  "Brake Hold Active": "正在使用刹车保持",
  "CAN Bus Disconnected: Check Connections": "CAN Bus Disconnected: 请检查连接",
  "CAN Bus Disconnected: Likely Faulty Cable": "CAN Bus Disconnected: 请检查连接",
  "Calibration Incomplete": "校准未完成",
  "Calibration Invalid: Remount Device & Recalibrate": "校准无效：请重新安装并校准设备",
  "Calibration in Progress": "正在校准",
  "Calibration Invalid": "校准出错",
  "Camera Malfunction": "相机故障",
  "Camera Malfunction: Reboot Your Device": "相机故障，请尝试重启设备",
  "Car Detected in Blindspot": "盲区检测到障碍",
  "Car Unrecognized": "车辆未识别",
  "Changing Lanes": "正在变道",
  "Communication Issue Between Processes": "进程通信故障",
  "Contact comma.ai/support": "这是硬件故障",
  "Controls Mismatch": "控制指令不一致",
  "Controls Mismatch: Lateral": "控制不匹配：横向",
  "Cruise Fault: Restart the Car": "巡航故障: 请尝试重启车辆",
  "Cruise Fault: Restart the car to engage": "巡航故障: 请尝试重启车辆",
  "Cruise Is Off": "巡航已关闭",
  "DISENGAGE IMMEDIATELY": "辅助驾驶正在解除",
  "Dashcam Mode": "行车记录模式",
  "Dashcam mode": "行车记录模式",
  "Dashcam mode for unsupported car": "车辆未识别",
  "Device Remount Detected: Recalibrating": "检测到设备重新安装: 正在校准",
  "Door Open": "车门未关好",
  "Drive to Calibrate": "请行驶以完成校准",
  "Driver Distracted": "驾驶员分心",
  "Driver Unresponsive": "驾驶员无反应",
  "Electronic Stability Control Active": "电子稳定系统已激活",
  "Electronic Stability Control Disabled": "电子稳定控制系统已禁用",
  "Ensure road ahead is clear": "请确认前方道路畅通",
  "Excessive Actuation": "执行器过度动作",
  "Experimental Mode Switched": "已切换到实验模式",
  "Fan Malfunction": "散热风扇故障",
  "Gear not D": "请切换到D档",
  "Invalid LKAS setting": "车道保持辅助系统设置无效",
  "Lane Change Unavailable: Road Edge": "道路边缘：暂时无法变道",
  "Lane Departure Detected": "检测到车道偏离",
  "Longitudinal Maneuver Mode": "纵向操控模式",
  "Low Memory: Reboot Your Device": "内存不足: 请重启设备",
  "Low Memory": "内存不足",
  "Manual Steering Required": "请手动控制方向",
  "Manual Speed Control Required": "请手动控制车速",
  "Out of Storage": "存储空间不足",
  "Pay Attention": "请注意！专心驾驶",
  "Pedal Pressed": "踏板被踩下",
  "Press Resume to Exit Brake Hold": "[ 踩油门 ] 退出刹车保持",
  "Press Resume to Exit Standstill": "[ 踩油门 ] 退出刹车保持",
  "Press + to confirm speed limit": "按 + 确认速度限制",
  "Press - to confirm speed limit": "按 - 确认速度限制",
  "Press Set to Engage": "按下Set键以启用",
  "Process Not Running": "进程未运行",
  "Posenet Speed Invalid": "视觉测速失效",
  "Release Brake to Engage": "松开刹车以启用",
  "Remount Detected: Recalibrating": "检测到设备重新安装: 正在校准",
  "Restart the car to retry,\nsmall model is still available": "请重新启动车辆后重试\n当前已降级为小模型",
  "Resume Driving Manually": "辅助驾驶已停用",
  "Reverse\nGear": "倒车中\n请注意周围环境",
  "Reverse Gear": "倒车中",
  "Seatbelt Unlatched": "请系好安全带",
  "Set speed changed": "设定速度已更改",
  "Switch to Traffic-Aware Cruise Control to engage": "切换到 主动巡航控制（TACC） 以启用",
  "Slow down to engage": "请减速以恢复辅助驾驶",
  "Smart/Adaptive Cruise Control: OFF": "自适应巡航控制：关闭",
  "Steer Left to Start Lane Change Once Safe": "确认安全后左转开始变道",
  "Steer Right to Start Lane Change Once Safe": "确认安全后右转开始变道",
  "Steering Pressed": "转向干预已触发",
  "Steering Temporarily Unavailable": "自动转向不可用",
  "System Initializing": "系统启动中",
  "System Overheated": "系统过热",
  "Steering misalignment detected": "检测到转向系统未校准",
  "Steer ratio mismatch": "转向比不匹配",
  "Abnormal tire stiffness": "轮胎刚度异常",
  "paramsd Temporary Error": "参数临时异常",
  "TAKE CONTROL": "请接管车辆",
  "Take Control": "请接管车辆",
  "Touch Steering Wheel": "请触碰方向盘",
  "Touch Steering Wheel: No Face Detected": "请触碰方向盘: 未检测到面部",
  "Toggle stock LKAS on or off to engage": "切换原厂车道保持辅助系统（LKAS）的开关以启用",
  "Turn Exceeds Steering Limit": "超过转向限制",
  "Turning Left": "正在左转",
  "Turning Right": "正在右转",
  "Unknown Vehicle Variant": "车型数据不匹配",
  "Vehicle Sensors Calibrating": "车辆传感器校准中",
  "Vehicle Sensors Invalid": "车辆传感器无效",
  "Vehicle Steering Time Limit": "转向时间限制",
  "WARNING: This branch is not tested": "警告：此分支未经测试",
  "Enable your car's LKAS to engage": "启用车辆的LKAS系统以启用",
  "Disable your car's stock LKAS to engage": "禁用原车的LKAS系统以启用",
  "AEB: Risk of Collision": "AEB：存在碰撞风险",
  "Always keep hands on wheel and eyes on road": "请始终手握方向盘并注视道路",
  "BRAKE!": "刹车！",
  "Bookmark Saved": "书签已保存",
  "CAN Bus Disconnected": "CAN 总线已断开",
  "Camera Frame Rate Low": "相机帧率过低",
  "Camera Frame Rate Low: Reboot Your Device": "相机帧率过低：请重启设备",
  "Cancel Pressed": "已按下取消键",
  "Car Not Ready": "车辆未就绪",
  "Driving Model Lagging": "驾驶模型运行延迟",
  "Emergency Braking: Risk of Collision": "紧急制动：存在碰撞风险",
  "Enable Adaptive Cruise to Engage": "启用自适应巡航后再启用辅助驾驶",
  "Enable Main Switch to Engage": "开启巡航主开关后再启用辅助驾驶",
  "Harness Relay Malfunction": "线束继电器故障",
  "Joystick Mode": "摇杆模式",
  "LKAS Fault: Restart the Car": "LKAS 故障：请重启车辆",
  "LKAS Fault: Restart the car to engage": "LKAS 故障：请重启车辆后再启用",
  "Lateral Maneuver Mode": "横向操控模式",
  "Low Communication Rate Between Processes": "进程间通信频率过低",
  "Model uncertain at this speed": "模型在此速度下置信度不足",
  "Pay Attention to Engage": "请集中注意力后再启用",
  "Radar Error: Restart the Car": "雷达错误：请重启车辆",
  "Radar Temporarily Unavailable": "雷达暂时不可用",
  "Reboot your Device": "请重启设备",
  "Risk of Collision": "存在碰撞风险",
  "Security Key Not Available": "安全密钥不可用",
  "Selfdrive Process Lagging: Reboot Your Device": "Selfdrive 进程延迟：请重启设备",
  "Sensor Data Invalid": "传感器数据无效",
  "Speed Too High": "车速过高",
  "Speed too low": "车速过低",
  "Stock AEB: Risk of Collision": "原车 AEB：存在碰撞风险",
  "Stock LKAS: Lane Departure Detected": "原车 LKAS：检测到车道偏离",
  "System Lagging": "系统运行延迟",
  "TAKE CONTROL IMMEDIATELY": "请立即接管车辆",
  "Too Distracted": "过度分心",
  "locationd Permanent Error": "locationd 永久错误",
  "locationd Temporary Error": "locationd 临时错误",
  "openpilot Canceled": "openpilot 已取消",
  "openpilot Unavailable": "openpilot 暂不可用",
  "openpilot will disengage": "openpilot 即将退出",
  "paramsd Permanent Error": "paramsd 永久错误",
}

_ZH_CHS_CONTEXT = {
  ("invalidLkasSetting/permanent", "Invalid LKAS setting"): "LKAS设置无效",
  ("overheat/permanent", "System Overheated"): "系统运行过热",
  ("parkBrake/noEntry", "Parking Brake Engaged"): "正在使用驻车制动",
  ("silentWrongGear/noEntry", "openpilot Unavailable"): "openpilot 暂不可用",
  ("silentParkBrake/noEntry", "Parking Brake Engaged"): "驻车制动已启用",
  ("steerTempUnavailable/softDisable", "Steering Assist Temporarily Unavailable"): "自动转向不可用",
  ("steerTempUnavailableSilent/warning", "Steering Assist Temporarily Unavailable"): "自动转向暂不可用",
}

_SPEED_LIMIT_PRE_ACTIVE_RE = re.compile(r"^Speed Limit Assist: set to (.+) to engage$")
_PERCENT_FULL_RE = re.compile(r"^(\d+)% full$")
_PERCENT_USED_RE = re.compile(r"^(\d+)% used$")
_BELOW_ENGAGE_SPEED_RE = re.compile(r"^Drive above (.+) to engage$")
_BELOW_STEER_SPEED_RE = re.compile(r"^Steer Assist Unavailable Below (.+)$")
_CALIBRATION_PROGRESS_RE = re.compile(r"^(Recalibrating|Calibrating): (.+)%$")
_CALIBRATION_MIN_SPEED_RE = re.compile(r"^Drive Above (.+)$")
_SPEED_ERROR_RE = re.compile(r"^Speed Error: (.+) m/s$")
_REMOUNT_ANGLES_RE = re.compile(r"^Remount Device \(Pitch: (.+)°, Yaw: (.+)°\)$")
_ANGLE_OFFSET_RE = re.compile(r"^Angle offset too high \(Offset: (.+)°\)$")
_STEER_RATIO_RE = re.compile(r"^Steering rack geometry may be off \(Ratio: (.+)\)$")
_STIFFNESS_FACTOR_RE = re.compile(r"^Check tires, pressure, or alignment \(Factor: (.+)\)$")
_PERSONALITY_RE = re.compile(r"^Driving Personality: (.+)$")
_SPEED_LIMIT_ADJUST_RE = re.compile(r"^Adjusting to (.+) speed limit$")
_FRAME_DROP_RE = re.compile(r"^(.+)% frames dropped$")
_JOYSTICK_RE = re.compile(r"^Gas: (.+)%, Steer: (.+)%$")
_MINUTES_LEFT_RE = re.compile(r"^(\d+) minutes? Left$")

_ZH_CHS_DYNAMIC_TEXT = "".join((
  "限速辅助：手动将设定速度更改为以激活正在调整限速速度至已使用",
  "请保持以上速度行驶以启用辅助驾驶以下速度行驶时无法自动转向",
  "重新校准自动校准进行中车速高于车速异常请调整设备安装",
  "角度偏移过大偏移转向齿条位置可能异常比例",
  "请检查轮胎胎压或四轮定位系数驾驶风格",
  "已丢弃的帧油门转向剩余分钟",
))


def _localize_zh_chs(alert_type: str, text: str) -> str:
  if translated := _ZH_CHS_CONTEXT.get((alert_type, text)):
    return translated
  if match := _SPEED_LIMIT_PRE_ACTIVE_RE.fullmatch(text):
    return f"限速辅助：手动将设定速度更改为 {match.group(1)} 以激活"
  if match := _SPEED_LIMIT_ADJUST_RE.fullmatch(text):
    return f"正在调整限速速度至 {match.group(1)} "
  if match := _PERCENT_FULL_RE.fullmatch(text):
    return f"{match.group(1)}% 已使用"
  if match := _PERCENT_USED_RE.fullmatch(text):
    return f"{match.group(1)}% 已使用"
  if match := _BELOW_ENGAGE_SPEED_RE.fullmatch(text):
    return f"请保持 {match.group(1)} 以上速度行驶以启用辅助驾驶"
  if match := _BELOW_STEER_SPEED_RE.fullmatch(text):
    return f" {match.group(1)} 以下速度行驶时无法自动转向"
  if match := _CALIBRATION_PROGRESS_RE.fullmatch(text):
    action = "重新校准" if match.group(1) == "Recalibrating" else "自动校准"
    return f"{action} 进行中: {match.group(2)}%"
  if match := _CALIBRATION_MIN_SPEED_RE.fullmatch(text):
    return f"请保持车速高于 {match.group(1)}"
  if match := _SPEED_ERROR_RE.fullmatch(text):
    return f"车速异常: {match.group(1)} m/s"
  if match := _REMOUNT_ANGLES_RE.fullmatch(text):
    return f"请调整设备安装 (Pitch: {match.group(1)}°, Yaw: {match.group(2)}°)"
  if match := _ANGLE_OFFSET_RE.fullmatch(text):
    return f"角度偏移过大 (偏移: {match.group(1)}°)"
  if match := _STEER_RATIO_RE.fullmatch(text):
    return f"转向齿条位置可能异常 (比例: {match.group(1)})"
  if match := _STIFFNESS_FACTOR_RE.fullmatch(text):
    return f"请检查轮胎、胎压或四轮定位 (系数: {match.group(1)})"
  if match := _PERSONALITY_RE.fullmatch(text):
    return f"驾驶风格: {match.group(1)}"
  if match := _FRAME_DROP_RE.fullmatch(text):
    return f"已丢弃 {match.group(1)}% 的帧"
  if match := _JOYSTICK_RE.fullmatch(text):
    return f"油门: {match.group(1)}%，转向: {match.group(2)}%"
  if match := _MINUTES_LEFT_RE.fullmatch(text):
    return f"剩余 {match.group(1)} 分钟"
  return _ZH_CHS_EXACT.get(text, text)


def localize_alert_text(alert_type: str, text1: str, text2: str, language: str) -> tuple[str, str]:
  if language != "zh-CHS":
    return text1, text2
  return _localize_zh_chs(alert_type, text1), _localize_zh_chs(alert_type, text2)


def localized_alert_characters(language: str) -> set[str]:
  """Return every fixed glyph the alert adapter can emit for the active language."""
  if language != "zh-CHS":
    return set()
  translations = (*_ZH_CHS_EXACT.values(), *_ZH_CHS_CONTEXT.values(), _ZH_CHS_DYNAMIC_TEXT)
  return set().union(*map(set, translations))
