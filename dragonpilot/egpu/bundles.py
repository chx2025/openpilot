"""
dp - eGPU model catalog and local state. No tinygrad / capnp imports: used by process_config, the UI and
egpu_manager.

The catalog is jetlink's (JetlinkCatalog, shared). A catalog entry ("bundle") here only names a model: the comma openpilot commit (`ref`) its
big_driving_supercombo.onnx comes from, and sunnypilot's display fields. sunnypilot's own pkls are not
used (other tinygrads, whole frames over USB); the model is compiled on this device (compile.py).

  bundle = {"ref", "short_name", "display_name", "index", "generation", "folder", "overrides": {k: v}, "source"}

The active bundle (dp_egpu_bundle) additionally carries "sha256": the ONNX it was compiled from.
"""
import os

from openpilot.common.params import Params
from dragonpilot.egpu.usb import egpu_present
from dragonpilot.egpu.constants import (P_ENABLED, P_PRESENT, P_ACTIVE_BUNDLE, P_POINTERS, P_FAILED,
                                        CAMERA_CONFIGS, DEFAULT_BUNDLE, DEFAULT_REF, JETLINK_POINTERS_PARAM, KNOWN_POINTERS, JETLINK_CATALOG_PARAM, model_root, model_pkl_name, warp_pkl_name, onnx_path)


def parse_bundle(b: dict, source: str = "") -> dict | None:
  try:
    ref = b.get("ref")
    if not isinstance(ref, str) or len(ref) != 40:
      return None
    if b.get("is_big") is False:
      return None
    overrides = {str(k): str(v) for k, v in (b.get("overrides") or {}).items()}
    return {
      "ref": ref,
      "short_name": str(b.get("short_name") or ref[:8]),
      "display_name": str(b.get("display_name") or ref[:10]),
      "index": int(b.get("index", 0)),
      "generation": int(b.get("generation", 0)),
      "folder": overrides.get("folder", ""),
      "overrides": overrides,
      "source": source,
    }
  except (TypeError, ValueError):
    return None


def cached_bundles(params: Params) -> list[dict]:
  """the catalog shared with jetlink (JetlinkCatalog), newest first; before the first refresh, jetlink's
  default big model alone, as the Jetlink panel shows it"""
  try:
    data = params.get(JETLINK_CATALOG_PARAM)
  except Exception:
    data = None
  if not (isinstance(data, dict) and data.get("bundles")):
    from dragonpilot.jetlink_adapter import _DEFAULT_CATALOG
    data = _DEFAULT_CATALOG
  seen, out = set(), []
  for raw in data.get("bundles", []):
    if isinstance(raw, dict) and (b := parse_bundle(raw, "jetlink")) is not None and b["ref"] not in seen:
      seen.add(b["ref"])
      out.append(b)
  return sorted(out, key=lambda b: (b["index"], b["display_name"]), reverse=True)


def find_bundle(bundles: list[dict], ref: str) -> dict | None:
  found = next((b for b in bundles if b["ref"] == ref), None)
  if found is None and ref == DEFAULT_REF:
    found = parse_bundle(DEFAULT_BUNDLE, "default")
  return found


def pointer_for(params: Params, ref: str) -> dict | None:
  """{oid, size} of a ref's ONNX if known: ours, else jetlink's cache of the same lookups"""
  for key in (P_POINTERS, JETLINK_POINTERS_PARAM):
    try:
      cache = params.get(key)
    except Exception:
      cache = None
    if isinstance(cache, dict) and isinstance(p := cache.get(ref), dict) and p.get("oid") and p.get("size"):
      return {"oid": str(p["oid"]), "size": int(p["size"])}
  if (p := KNOWN_POINTERS.get(ref)) is not None:
    return dict(p)
  return None


def onnx_present(pointer: dict | None) -> bool:
  if not pointer:
    return False
  try:
    return onnx_path(pointer["oid"]).stat().st_size == int(pointer["size"])
  except OSError:
    return False


def warps_present(root: str | None = None) -> bool:
  root = root or model_root()
  return all(os.path.isfile(os.path.join(root, warp_pkl_name(w, h))) for w, h in CAMERA_CONFIGS)


def model_compiled(sha256: str | None, root: str | None = None) -> bool:
  if not sha256:
    return False
  return os.path.isfile(os.path.join(root or model_root(), model_pkl_name(sha256)))


def get_active_bundle(params: Params) -> dict | None:
  raw = params.get(P_ACTIVE_BUNDLE)
  if not isinstance(raw, dict) or not raw.get("ref") or not raw.get("sha256"):
    return None
  return raw


def active_ready(bundle: dict | None) -> bool:
  return bool(bundle) and model_compiled(bundle.get("sha256")) and warps_present()


def egpu_should_drive(params: Params) -> bool:
  """Whether the next drive should run modeld_egpu instead of the stock modeld.

  dp - fix: probe the USB dock live instead of trusting dp_egpu_present. egpu_manager writes that
  param once per second, but EGPU_ROUTE.update() latches on the first onroad tick and that can race
  the manager's first write on a slow-enumerating dock -> the big model is skipped for the whole
  drive. egpu_present() is a cheap sysfs VID/PID scan, safe to call here."""
  if not params.get_bool(P_ENABLED) or params.get_bool(P_FAILED):
    return False
  if not params.get_bool(P_PRESENT) and not egpu_present():
    return False
  return active_ready(get_active_bundle(params))


class EgpuRoute:
  """
  Latched routing for process_config. The decision is taken once per drive (first onroad tick) so a
  flaky USB link cannot bounce modeld <-> modeld_egpu mid-drive; the only mid-drive switch is
  big -> stock after modeld_egpu sets dp_egpu_failed. Equivalent to sunnypilot's one-way
  "big model unavailable until the next drive" fallback.
  """
  def __init__(self):
    self._latched: bool | None = None

  def update(self, started: bool, params: Params) -> bool:
    if not started:
      self._latched = None
      return False
    if self._latched is None:
      self._latched = egpu_should_drive(params)
    elif self._latched and params.get_bool(P_FAILED):
      self._latched = False
    return self._latched


EGPU_ROUTE = EgpuRoute()
