"""
dp - eGPU constants. Keep this module free of tinygrad / capnp / hardware-layer imports: it is imported by
process_config (manager process), the UI and the model manager daemon.
"""
import os
from pathlib import Path

# same test as openpilot.system.hardware.PC, without importing the hardware layer
PC = not os.path.isfile('/TICI')

# ---------------------------------------------------------------------------------------------
# Model catalog: SHARED WITH JETLINK. The list is jetlink's JetlinkCatalog param, filled by
# dragonpilot/jetlink_adapter refresh_catalog(): sunnypilot's chestnut catalogs (the pinned one and every
# newer version) + zoompilot/jetlink's extra list, as the jetlink phone/Mac apps list them; it also
# resolves each model's ONNX pointer into JetlinkModelPointers. A refresh from either the Jetlink panel or
# the eGPU panel updates both. Nothing but names and the comma openpilot commit (`ref`) is used:
# sunnypilot's own pkls are for other tinygrads and send whole frames over USB.
# ---------------------------------------------------------------------------------------------
JETLINK_CATALOG_PARAM = "JetlinkCatalog"          # jetlink_adapter KEYS.catalog, {bundles: [...]}
MODEL_SOURCE_LABEL = "與 Jetlink 共用"

MANIFEST_CACHE_TIMEOUT_S = 3600

# ---------------------------------------------------------------------------------------------
# The ONNX: jetlink's registry (jetlink_repo/jetlink/registry/lfs.py) resolves a ref to the git-lfs
# pointer of its big_driving_supercombo.onnx and downloads it from comma's LFS servers. It is kept
# where jetlink keeps it, /data/media/0/models/jetlink/<sha256[:16]>.onnx, so the two share one copy.
# ---------------------------------------------------------------------------------------------
JETLINK_POINTERS_PARAM = "JetlinkModelPointers"   # jetlink's own {ref: {oid, size}} cache, read only

# ---------------------------------------------------------------------------------------------
# On-device compile. The vendored tinygrad is sunnypilot/tinygrad@9cd40014 (= sunnypilot-master's
# tinygrad_repo); the compiled pkls only load with it, so its ref is part of every cache key.
#   warp  : QCOM, one per camera, model-independent: only the warped crop crosses USB (AmyJeanes/
#           sunnypilot@173d90b / commaai#38862 idea, here as a separate warp JIT like sunnypilot-master)
#   model : USB+AMD, one per ONNX (stateful #38916 graphs as-is, queued graphs wrapped with their queues)
# ---------------------------------------------------------------------------------------------
VENDORED_TINYGRAD_REF = "9cd40014f651ac2472b3b5fc3b12178ab4bb1c66"
COMPILE_VERSION = 1  # bump when compile.py changes what it produces
CACHE_TAG = f"tg{VENDORED_TINYGRAD_REF[:8]}v{COMPILE_VERSION}"

# sunnypilot-master selfdrive/modeld/SConscript tg_flags (comma) / chestnut_tg_flags without FRAME_DEV=CPU
TG_FLAGS_QCOM = {"DEV": "QCOM", "IMAGE": "1", "FLOAT16": "1", "NOLOCALS": "1", "JIT_BATCH_SIZE": "0", "OPENPILOT_HACKS": "1"}
TG_FLAGS_AMD = {"DEBUG": "1", "DEV": "USB+AMD:LLVM", "FLOAT16": "1", "JIT_BATCH_SIZE": "0", "GMMU": "0",
                "TC_OPT": "2", "TC_MIN_GLOBALS": "32"}

MODEL_INPUT_SIZE = (512, 256)  # common/transformations/model.py MEDMODEL_INPUT_SIZE
# camera NV12 geometries (selfdrive/modeld/SConscript CAMERA_CONFIGS): tici/3X 1928x1208, mici 1344x760
CAMERA_CONFIGS = ((1928, 1208), (1344, 760))

# ---------------------------------------------------------------------------------------------
# eGPU USB identity
#   0xADD1:0x0001  comma chestnut / ASM2464 with comma firmware (dp0111pre selfdrive/modeld/helpers.py)
#   0x3801:0x0001  second id listed in sunnypilot common/hardware/usb.py CHESTNUT_USB_IDS
# ---------------------------------------------------------------------------------------------
EGPU_USB_IDS = ((0xADD1, 0x0001), (0x3801, 0x0001))

# ---------------------------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------------------------
EGPU_DIR = Path(__file__).resolve().parent
VENDORED_TINYGRAD_DIR = EGPU_DIR / "tinygrad_spm"


def models_root() -> Path:
  """the partition jetlink and sunnypilot keep models on"""
  if os.environ.get("EGPU_MODELS_ROOT"):
    return Path(os.environ["EGPU_MODELS_ROOT"])
  if PC:
    return Path.home() / (".comma" + os.environ.get("OPENPILOT_PREFIX", "")) / "media" / "0" / "models"
  return Path("/data/media/0/models")


def model_root() -> str:
  """compiled pkls (egpu's own; clear cache only touches this)"""
  return str(models_root() / "egpu")


def onnx_root() -> Path:
  """shared with jetlink"""
  return models_root() / "jetlink"


def onnx_path(sha256: str) -> Path:
  return onnx_root() / f"{sha256[:16]}.onnx"


def model_pkl_name(sha256: str) -> str:
  return f"model_{sha256[:16]}_{CACHE_TAG}.pkl"


def warp_pkl_name(cam_w: int, cam_h: int) -> str:
  return f"warp_{cam_w}x{cam_h}_{MODEL_INPUT_SIZE[0]}x{MODEL_INPUT_SIZE[1]}_{CACHE_TAG}.pkl"


# ---------------------------------------------------------------------------------------------
# Params (declared in dragonpilot/settings/min-feat.dev.egpu.py)
# ---------------------------------------------------------------------------------------------
P_ENABLED = "dp_egpu_enabled"                # BOOL  PERSISTENT   master switch for the big model
P_PRESENT = "dp_egpu_present"                # BOOL  manager start  eGPU seen on USB (written by egpu_manager)
P_ACTIVE_BUNDLE = "dp_egpu_bundle"           # JSON  PERSISTENT   selected model, compiled and ready
P_MANIFEST_SYNC = "dp_egpu_manifest_sync"    # INT   manager start  monotonic ns of the last catalog refresh, 0 = refresh now
P_POINTERS = "dp_egpu_pointers"              # JSON  PERSISTENT   {ref: {oid, size}} resolved ONNX pointers
P_DOWNLOAD_REF = "dp_egpu_download_ref"      # STRING manager start  ref to download + compile; removing it cancels
P_STATUS = "dp_egpu_status"                  # JSON  manager start  pipeline status for the UI
P_DELETE_REF = "dp_egpu_delete_ref"          # STRING manager start  request: delete one model (compiled pkl + ONNX)
P_CLEAR_CACHE = "dp_egpu_clear_cache"        # BOOL  manager start  request: delete all but the active model
P_LOADING = "dp_egpu_loading"                # BOOL  manager start | offroad  modeld_egpu is loading the model
P_RUNNING = "dp_egpu_running"                # BOOL  manager start | offroad  big model is driving
P_FAILED = "dp_egpu_failed"                  # BOOL  manager start | offroad  big model failed this drive -> stock modeld

# pipeline states (P_STATUS["state"])
ST_IDLE = "idle"
ST_RESOLVING = "resolving"          # ref -> ONNX pointer
ST_DOWNLOADING = "downloading"      # ONNX
ST_COMPILING_WARP = "compiling_warp"
ST_COMPILING = "compiling"          # model
ST_DONE = "done"
ST_FAILED = "failed"
ST_CANCELLED = "cancelled"
ST_WAITING_EGPU = "waiting_egpu"    # downloaded, compile needs the eGPU plugged in
BUSY_STATES = (ST_RESOLVING, ST_DOWNLOADING, ST_COMPILING_WARP, ST_COMPILING)
