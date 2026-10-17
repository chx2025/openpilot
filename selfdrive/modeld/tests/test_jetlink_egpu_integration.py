import ast
import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[3]


def load(name, path):
  spec = importlib.util.spec_from_file_location(name, ROOT / path)
  module = importlib.util.module_from_spec(spec)
  sys.modules[name] = module
  spec.loader.exec_module(module)
  return module


@pytest.fixture
def adapter():
  return load('integration_adapter', 'dragonpilot/jetlink_adapter/__init__.py')


def test_api3_contract(adapter):
  interface = load('integration_interface', 'jetlink_repo/jetlink/openpilot/interface.py')
  obj = adapter.Adapter.__new__(adapter.Adapter)
  obj.keys = adapter.KEYS
  obj.log = SimpleNamespace()
  obj.basedir = ROOT
  assert adapter.API == 3
  assert interface.conformance(obj, interface.Openpilot) == []


@pytest.mark.parametrize('enabled,seen,link_enabled,expected', [
  (True, True, True, False), (True, True, False, False),
  (False, True, True, True), (True, False, True, True),
  (False, False, False, False),
])
def test_usb_reservation(adapter, enabled, seen, link_enabled, expected):
  calls = []
  adapter._api = lambda: calls.append(True) or SimpleNamespace(enabled=lambda: link_enabled)
  params = SimpleNamespace(get_bool=lambda key: {'dp_egpu_enabled': enabled, 'dp_egpu_seen': seen}[key])
  assert adapter.should_run(False, params, None) is expected
  assert bool(calls) is not (enabled and seen)


@pytest.mark.parametrize('usb_id', [(0xADD1, 1), (0x3801, 1), (0xADD1, 2), (0x174C, 0x2464), (0x174C, 0x2463)])
def test_host_device_ids_preserved(adapter, usb_id):
  assert usb_id in adapter.CHESTNUT_IDS


@pytest.mark.parametrize('valid,enabled,lat,alka,main,gear,expected', [
  (True, False, False, False, False, 'drive', False),
  (True, True, False, False, False, 'drive', True),
  (True, False, True, False, False, 'drive', True),
  (True, False, False, True, False, 'drive', True),
  (True, False, False, False, True, 'drive', True),
  (True, False, False, False, True, 'park', False),
  (False, False, False, False, False, 'park', True),
])
def test_api3_reads_shared_control_state(adapter, valid, enabled, lat, alka, main, gear, expected):
  class SM:
    def all_alive(self, services): return valid
    def all_valid(self, services): return valid
    def __getitem__(self, name):
      return {'carControl': SimpleNamespace(enabled=enabled, latActive=lat),
              'controlsStateExt': SimpleNamespace(alkaActive=alka),
              'carStateExt': SimpleNamespace(lkasOn=main),
              'carState': SimpleNamespace(gearShifter=gear)}[name]
  adapter._alka = lambda: True
  adapter.use_submaster(SM())
  assert adapter.Adapter.__new__(adapter.Adapter).in_control() is expected


def test_modeld_wires_shared_submaster_before_inference():
  tree = ast.parse((ROOT / 'selfdrive/modeld/modeld.py').read_text())
  main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
  wire = next(n for n in ast.walk(main) if isinstance(n, ast.Call) and ast.unparse(n.func) == 'jetlink_adapter.use_submaster')
  loop = [n for n in main.body if isinstance(n, ast.While)][-1]
  assert wire.lineno < loop.lineno
  assert 'egpu_loader' in [a.arg for a in main.args.args]


def test_new_warp_name_does_not_load_old_jit(adapter):
  path = adapter.warp_path(1928, 1208, 512, 256)
  assert path.name == 'driving_warp_1928x1208_512x256_tinygrad.pkl'
  build = (ROOT / 'dragonpilot/jetlink_adapter/SConscript').read_text()
  assert 'dragonpilot.jetlink_adapter.build_warp' in build


def test_egpu_routes_and_params_preserved():
  config = (ROOT / 'system/manager/process_config.py').read_text()
  assert 'NativeProcess("modeld_egpu"' in config
  assert 'PythonProcess("egpu_manager"' in config
  assert 'and_(only_onroad, not_egpu_big_model)' in config
  assert 'cluster_d' not in config
  params = (ROOT / 'common/params_keys.h').read_text()
  for key in ['dp_egpu_enabled', 'dp_egpu_failed', 'dp_egpu_running', 'JetlinkLink', 'JetlinkSpec']:
    assert '"' + key + '"' in params
