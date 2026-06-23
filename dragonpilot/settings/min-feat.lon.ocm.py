from dragonpilot.settings import tr

ITEMS = [
  {
    "section": "Longitudinal",
    "key": "dp_lon_ocm",
    "type": "toggle_item",
    "title": lambda: tr("Overtaking Coasting Mode (OCM)"),
    "description": lambda: tr("Smoothly coast down to your set speed without engine braking when you exceed the cruise speed by 20 km/h after overtaking."),
   #"condition": "openpilotLongitudinalControl",
    "flags": "PERSISTENT",
    "param_type": "BOOL",
    "default": "0",
  },
]
