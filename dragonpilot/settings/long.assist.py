"""dp 纵向辅助开关（UI：設定 → Longitudinal）。

两个功能各一个 toggle，**默认开启**：
  * TurnDecel     —— 打灯减速 + 大角度不主动加速（dragonpilot/selfdrive/controls/lib/turn_decel.py）
  * GasLongCancel —— 踩油门挂起纵向，松油门 0.2 s 后恢复
                     （dragonpilot/selfdrive/controls/lib/gas_long_cancel.py）

i18n：msgid 写英文，繁体译文在
`selfdrive/ui/translations/dragonpilot_zh-CHT.po`。
★ 不要在 msgid 里写简体中文 —— UI 的字体是预烘焙图集
（`selfdrive/assets/fonts/OpFont-*-zh-CHT.fnt`），且 `dragonpilot/settings/process.py`
只从 .po 收集字符；msgid 里带的字若不在图集内，raylib 会直接画成 `?`。

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
    "title": lambda: tr("Turn Signal Decel + Sharp Turn Accel Block"),
    "description": lambda: tr(
      "After the turn signal is on for 2 s, decelerate toward 15 km/h "
      "(0.5 ramping up to 1.2 m/s^2). Above 15 deg of steering the deceleration "
      "pauses, but acceleration stays blocked. Above 60 deg and faster than 15 km/h, "
      "acceleration is blocked regardless of the turn signal (U-turns, parking, "
      "spiral ramps). Pressing the gas pedal overrides everything."),
    "default": "1",
    "flags": "PERSISTENT",
    "param_type": "BOOL",
  },
  {
    "section": "Longitudinal",
    "key": "GasLongCancel",
    "type": "toggle_item",
    "title": lambda: tr("Gas Pedal Suspends Longitudinal"),
    "description": lambda: tr(
      "Pressing the gas pedal suspends all longitudinal control (the car coasts: "
      "no acceleration, no active braking) and it resumes 0.2 s after you release it. "
      "If longitudinal was cancelled by braking or a button press, the pedal will not "
      "bring it back: you must re-engage. This does not cancel the car's own ACC."),
    "default": "1",
    "flags": "PERSISTENT",
    "param_type": "BOOL",
  },
]
