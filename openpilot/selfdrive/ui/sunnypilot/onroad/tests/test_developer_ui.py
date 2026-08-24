from types import SimpleNamespace

from openpilot.selfdrive.ui.sunnypilot.onroad.developer_ui.elements import build_device_resource_elements


def test_bottom_resources_replace_driving_debug_abbreviations():
  device_state = SimpleNamespace(cpuTempC=[54.2, 61.8], memoryUsagePercent=43, freeSpacePercent=39.4)

  elements = build_device_resource_elements(device_state)

  assert [(item.label, item.value, item.unit) for item in elements] == [
    ("CPU", "62", "°"),
    ("MEM", "43", "%"),
    ("DSK", "39", "%"),
  ]
  assert not {"ACC.", "L.S.", "E.T."} & {item.label for item in elements}


def test_cpu_temperature_handles_missing_sensor_values():
  device_state = SimpleNamespace(cpuTempC=[], memoryUsagePercent=0, freeSpacePercent=100.0)

  elements = build_device_resource_elements(device_state)

  assert elements[0].value == "-"
