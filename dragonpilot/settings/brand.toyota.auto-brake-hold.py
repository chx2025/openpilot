from dragonpilot.settings import tr

ITEMS = [
  {
    "section": "Toyota / Lexus",
    "key": "dp_toyota_auto_brake_hold",
    "type": "text_spin_button_item",
    "title": lambda: tr("Auto Brake Hold (TSS2)"),
    "description": lambda: tr("For vehicles without a factory Auto Hold. Holds the car at a standstill using the PCS brake once stopped for 2 seconds with cruise disengaged. Press the gas to release; press the brake again to release and restart the timer.<br>Off: Disabled.<br>On: Active whenever in Drive (D).<br>Speed: Active in Drive (D) only after exceeding 18 km/h (5 m/s) since shifting into D.<br>Requires a TSS2 camera-ACC vehicle with openpilot longitudinal control. Takes effect after the car is restarted."),
    "options": [lambda: tr("Off"), lambda: tr("On"), lambda: tr("Speed")],
    "default": "0",
    "brands": ["toyota"],
    "condition": "openpilotLongitudinalControl",
    "flags": "PERSISTENT",
    "param_type": "INT",
  },
  {
    "section": "Toyota / Lexus",
    "key": "dp_toyota_auto_brake_hold_force",
    "type": "text_spin_button_item",
    "title": lambda: tr("Auto Brake Hold Force"),
    "description": lambda: tr("Brake force used while holding. Standard (-2.0 m/s²) matches the original implementation. If the car still creeps forward while holding (common on gasoline models due to idle creep in Drive), try High (-3.0) or Max (-4.0). Takes effect after the car is restarted."),
    "options": [lambda: tr("Standard"), lambda: tr("High"), lambda: tr("Max")],
    "default": "0",
    "brands": ["toyota"],
    "condition": "openpilotLongitudinalControl",
    "flags": "PERSISTENT",
    "param_type": "INT",
  },
]
