from dragonpilot.settings import tr

ITEMS = [
  {
    "section": "Lateral",
    "key": "dp_lcc_enabled",
    "type": "toggle_item",
    "title": lambda: tr("Enable Lane Centering Correction"),
    "description": lambda: tr("Adds a small curvature trim based on lane line geometry to keep the car centered in its lane. Only active above 50 km/h; drops back to stock openpilot lateral control at or below 40 km/h, and also disables automatically on sharp turns (e.g. intersection right turns). If the turn signal is on and you apply steering torque, the correction yields immediately; without a signal it stays engaged even if you nudge the wheel. Not yet road tested — enable with caution."),
    "flags": "PERSISTENT",
    "param_type": "BOOL",
    "default": "0",
  },
]
