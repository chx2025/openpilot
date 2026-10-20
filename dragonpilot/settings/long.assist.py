"""dp 纵向辅助开关（UI：設定 → Longitudinal）。

两个功能各一个 toggle，**默认开启**：
  * TurnDecel     —— 打燈減速 + 大角度不主動加速（dragonpilot/selfdrive/controls/lib/turn_decel.py）
  * GasLongCancel —— 踩油門掛起縱向，鬆油門 0.2 s 後恢復
                     （dragonpilot/selfdrive/controls/lib/gas_long_cancel.py）

★ 文案直接用**繁體中文**，绝不写简体：
UI 不在运行时渲染 OTF，而是加载预烘焙字库
`selfdrive/assets/fonts/OpFont-<weight>-zh-CHT.fnt`（4941 字，只有繁体字形）。
raylib 遇到字库里没有的码点会**直接画成 `?`**，所以简体 `灯减门挂纵`
必然变成 `打??速`。改文案后要确认这些字都在图集里（见 selfdrive/assets/fonts/process.py）。

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
    "title": lambda: tr("打燈減速 / 大角度不加速"),
    "description": lambda: tr(
      "打轉向燈 2 秒後向 15 km/h 減速（由 0.5 逐步升至 1.2 m/s^2）；"
      "方向盤超過 15 度時暫停減速，但仍不主動加速；"
      "方向盤超過 60 度且車速超過 15 km/h 時不主動加速"
      "（與轉向燈無關，適合掉頭、入庫、螺旋匝道）。踩油門時本功能完全讓位。"),
    "default": "1",
    "flags": "PERSISTENT",
    "param_type": "BOOL",
  },
  {
    "section": "Longitudinal",
    "key": "GasLongCancel",
    "type": "toggle_item",
    "title": lambda: tr("踩油門掛起縱向"),
    "description": lambda: tr(
      "踩下油門即掛起整個縱向控制（本車滑行，既不加速也不主動煞車），"
      "鬆開油門 0.2 秒後自動恢復。"
      "若縱向是被煞車或按鍵取消的，之後踩/鬆油門都不會恢復，必須重新 engage；"
      "掛起不會取消車輛自身的 ACC。"),
    "default": "1",
    "flags": "PERSISTENT",
    "param_type": "BOOL",
  },
]
