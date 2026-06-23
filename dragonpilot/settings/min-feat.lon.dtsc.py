from dragonpilot.settings import tr

ITEMS = [
  {
    "section": "Longitudinal",
    "key": "dp_lon_dtsc",
    "type": "toggle_item",
    "title": lambda: tr("Dynamic Turn Speed Control (DTSC)"),
    "description": lambda: tr("DTSC automatically adjusts the vehicle's predicted speed based on upcoming road curvature and grip conditions.<br>Originally from the openpilot TACO branch."),
   #"condition": "openpilotLongitudinalControl",
    "flags": "PERSISTENT",
    "param_type": "BOOL",
    "default": "0",
  },
]
