from dragonpilot.settings import tr

# dp: 加速度曲線分段倍率。倍率作用在
# selfdrive/controls/lib/longitudinal_planner.py 頂部的 A_CRUISE_MAX_VALS 上。
ITEMS = [
  {
    "section": "Longitudinal",
    "key": "dp_accel_tbl_enabled",
    "type": "toggle_item",
    "title": lambda: tr("調校加速度曲線"),
    "description": lambda: tr(
      "開啟後可用下面三段倍率微調加速上限曲線（原始表是 longitudinal_planner.py 頂部的 "
      "A_CRUISE_MAX_VALS）。調大 = 該速度段加速更強，調小 = 更柔和。"
      "本頁只影響實驗模式，以及 AEM 關閉時的 ACC；若 AEM 開啟並使用 ACC，"
      "加速上限由「加速度個性化」的 eco/normal/sport 三檔決定，本頁不參與。"
      "倍率帶 1.5 秒平滑過渡，行駛中調整不會造成加速度突變。"),
    "default": "0",
    "flags": "PERSISTENT",
    "param_type": "BOOL",
  },
  {
    "section": "Longitudinal",
    "key": "dp_accel_tbl_low_pct",
    "type": "spin_button_item",
    "title": lambda: tr("低速段倍率 0-30 km/h"),
    "description": lambda: tr("起步與低速跟車的加速力度。100% 為原始表數值。"),
    "min_val": 50,
    "max_val": 150,
    "step": 5,
    "suffix": "%",
    "default": "100",
    "depends_on": "dp_accel_tbl_enabled == 1",
    "flags": "PERSISTENT",
    "param_type": "INT",
  },
  {
    "section": "Longitudinal",
    "key": "dp_accel_tbl_mid_pct",
    "type": "spin_button_item",
    "title": lambda: tr("中速段倍率 30-70 km/h"),
    "description": lambda: tr("市區與郊區常用速度的加速力度。"),
    "min_val": 50,
    "max_val": 150,
    "step": 5,
    "suffix": "%",
    "default": "100",
    "depends_on": "dp_accel_tbl_enabled == 1",
    "flags": "PERSISTENT",
    "param_type": "INT",
  },
  {
    "section": "Longitudinal",
    "key": "dp_accel_tbl_high_pct",
    "type": "spin_button_item",
    "title": lambda: tr("高速段倍率 70 km/h 以上"),
    "description": lambda: tr("高速再加速的力度。"),
    "min_val": 50,
    "max_val": 150,
    "step": 5,
    "suffix": "%",
    "default": "100",
    "depends_on": "dp_accel_tbl_enabled == 1",
    "flags": "PERSISTENT",
    "param_type": "INT",
  },
]
