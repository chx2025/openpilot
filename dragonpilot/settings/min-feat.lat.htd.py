from dragonpilot.settings import tr

ITEMS = [
  {
    "section": "Lateral",
    "key": "dp_htd_enabled",
    "type": "toggle_item",
    "title": lambda: tr("Enable Human Turn Detection"),
    "description": lambda: tr("Unavailable during cruise control.Automatically pause steering when the driver applies large manual steering input, then smoothly resume."),
    "flags": "PERSISTENT",
    "param_type": "BOOL",
    "default": "0",
    "on_change": [{
      "target": "dp_htd_turn_angle_threshold",
      "action": "set_visible",
      "condition": "value"
    }]
  },
  {
    "section": "Lateral",
    "key": "dp_htd_turn_angle_threshold",
    "type": "spin_button_item",
    "title": lambda: tr("Trigger angle"),
    "description": lambda: tr("Driver steering angle that triggers HTD (degrees)."),
    "flags": "PERSISTENT",
    "param_type": "INT",
    "default": "60",
    "min_val": 60,
    "max_val": 120,
    "step": 5,
    "suffix": lambda: tr(" °"),
    "initially_visible_by": {
      "param": "dp_htd_enabled",
      "condition": "value == True",
      "default": 0
    }
  },
]
