# dp - egpu: big driving model on a USB eGPU (chestnut), ported from sunnypilot, compiled on the device. UI lives in its own
# settings panel (dragonpilot/selfdrive/ui/layouts/settings/egpu.py), so these are param-only entries.
ITEMS = [
  {"key": "dp_egpu_enabled", "flags": "PERSISTENT", "param_type": "BOOL", "default": "1"},
  {"key": "dp_egpu_present", "flags": "CLEAR_ON_MANAGER_START", "param_type": "BOOL", "default": "0"},
  {"key": "dp_egpu_bundle", "flags": "PERSISTENT", "param_type": "JSON"},
  {"key": "dp_egpu_manifest_sync", "flags": "CLEAR_ON_MANAGER_START", "param_type": "INT", "default": "0"},
  {"key": "dp_egpu_pointers", "flags": "PERSISTENT", "param_type": "JSON"},
  {"key": "dp_egpu_delete_ref", "flags": "CLEAR_ON_MANAGER_START", "param_type": "STRING"},
  {"key": "dp_egpu_download_ref", "flags": "CLEAR_ON_MANAGER_START", "param_type": "STRING"},
  {"key": "dp_egpu_status", "flags": "CLEAR_ON_MANAGER_START", "param_type": "JSON"},
  {"key": "dp_egpu_clear_cache", "flags": "CLEAR_ON_MANAGER_START", "param_type": "BOOL", "default": "0"},
  {"key": "dp_egpu_loading", "flags": "CLEAR_ON_MANAGER_START|CLEAR_ON_OFFROAD_TRANSITION", "param_type": "BOOL", "default": "0"},
  {"key": "dp_egpu_running", "flags": "CLEAR_ON_MANAGER_START|CLEAR_ON_OFFROAD_TRANSITION", "param_type": "BOOL", "default": "0"},
  {"key": "dp_egpu_failed", "flags": "CLEAR_ON_MANAGER_START|CLEAR_ON_OFFROAD_TRANSITION", "param_type": "BOOL", "default": "0"},
]
