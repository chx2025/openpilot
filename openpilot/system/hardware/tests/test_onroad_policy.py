import inspect

from openpilot.system.hardware import hardwared


def test_branch_and_device_labels_never_block_onroad():
  source = inspect.getsource(hardwared.hardware_thread)

  assert 'startup_conditions["not_tici"]' not in source
  assert 'onroad_conditions["not_tici"]' not in source
  assert 'set_offroad_alert("Offroad_TiciSupport", is_unsupported_combo' not in source


def test_safety_and_user_start_conditions_remain_enforced():
  source = inspect.getsource(hardwared.hardware_thread)

  for condition in (
    "up_to_date",
    "no_excessive_actuation",
    "not_uninstalling",
    "accepted_terms",
    "accepted_terms_sp",
    "free_space",
    "completed_training",
    "not_driver_view",
    "device_temp_engageable",
    "device_booted",
    "not_always_offroad",
  ):
    assert f'startup_conditions["{condition}"]' in source
  assert 'onroad_conditions["ignition"]' in source
