from dragonpilot.settings import tr

ITEMS = [
  {
    "section": "Longitudinal",
    "key": "AccelPersonalityEnabled",
    "type": "toggle_item",
    "title": lambda: tr("Dynamic Accel Personality"),
    "description": lambda: tr("Dynamic acceleration switch"),
    #"condition": "openpilotLongitudinalControl",
    "flags": "PERSISTENT",
    "param_type": "BOOL",
    "default": "0",
    "on_change": [{
      "target": "AccelPersonality",
      "action": "set_enabled",
      "condition": "value"
    }]
  },
  {
    "section": "Longitudinal",
    "key": "AccelPersonality",
    "type": "text_spin_button_item",
    "title": lambda: tr("Accel Personality"),
    "description": lambda: tr("Dynamic acceleration selection mode"),
    #"condition": "openpilotLongitudinalControl",
    "flags": "PERSISTENT",
    "param_type": "INT",
    "default": "1",
    "options": [
      lambda: tr("Sport"),
      lambda: tr("Normal"),
      lambda: tr("Eco"),
    ],
  },
]
