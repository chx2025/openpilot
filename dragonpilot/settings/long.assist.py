"""dp 纵向辅助开关（UI：設定 → Longitudinal）。

两个功能各一个 toggle，**默认开启**：
  * TurnDecel     —— 打灯减速 + 大角度不主动加速（dragonpilot/selfdrive/controls/lib/turn_decel.py）
  * GasLongCancel —— 踩油门挂起纵向，松油门 0.2 s 后恢复
                     （dragonpilot/selfdrive/controls/lib/gas_long_cancel.py）

param_type / flags 会被 dragonpilot/settings 的生成器用来产出 common/params_keys.h；
两个键已手工同步声明在该文件里（GasLongCancel 默认 "1"、TurnDecel 默认 "1"）。

"默认开启"能落到实处靠两件事：
  1. params_keys.h 里 default = "1"；
  2. 两个模块首次运行时把该默认值**落盘**（Params.get 对未设置的键返回 None，
     而 UI 的 get_bool() 不走 default 回退 ⇒ 不落盘会显示成"关"）。
"""
from dragonpilot.settings import tr

ITEMS = [
  {
    "section": "Longitudinal",
    "key": "TurnDecel",
    "type": "toggle_item",
    "title": lambda: tr("打灯减速 / 大角度不加速"),
    "description": lambda: tr(
      "打转向灯 2 秒后减速到 15 km/h（0.5→1.2 m/s² 渐进抬升）；"
      "方向盘超过 15° 时暂停减速但仍不主动加速；"
      "方向盘超过 60° 且车速超过 15 km/h 时不主动加速（与转向灯无关，"
      "适合掉头、入库、螺旋匝道）。踩油门时本功能完全让位。"),
    "default": "1",
    "flags": "PERSISTENT",
    "param_type": "BOOL",
  },
  {
    "section": "Longitudinal",
    "key": "GasLongCancel",
    "type": "toggle_item",
    "title": lambda: tr("踩油门挂起纵向"),
    "description": lambda: tr(
      "踩下油门即挂起整个纵向控制（本车滑行，既不加速也不主动刹车），"
      "松开油门 0.2 秒后自动恢复。"
      "若纵向是被刹车或按键取消的，之后踩／松油门都不会把它恢复，"
      "必须重新 engage；挂起不会取消车辆自身的 ACC。"),
    "default": "1",
    "flags": "PERSISTENT",
    "param_type": "BOOL",
  },
]
