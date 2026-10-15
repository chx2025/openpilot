"""
dp - egpu: put the vendored sunnypilot tinygrad (tinygrad_spm/) first on sys.path.

Must run before anything imports tinygrad, in a fresh interpreter (modeld_egpu.py and compile.py are
started as their own processes for exactly this reason: manager preimports dp's processes, whose
tinygrad is dp's tinygrad_repo). Importing this module is the setup; nothing else to call.
"""
import os
import sys
from pathlib import Path

EGPU_DIR = Path(__file__).resolve().parent
VENDORED = EGPU_DIR / "tinygrad_spm"
BASEDIR = EGPU_DIR.parent.parent

if "tinygrad" in sys.modules and not Path(sys.modules["tinygrad"].__file__).resolve().is_relative_to(VENDORED):
  raise ImportError(f"dp's tinygrad already imported ({sys.modules['tinygrad'].__file__}); egpu needs a fresh interpreter")

# drop the script's own dir: its module names (constants, usb, manager, ...) must not shadow anything
sys.path[:] = [p for p in sys.path if Path(p or '.').resolve() != EGPU_DIR]
sys.path.insert(0, str(VENDORED))
if str(BASEDIR) not in sys.path:
  sys.path.insert(1, str(BASEDIR))

os.environ.setdefault('GMMU', '0')  # chestnut fast loading (sunnypilot modeld)
if os.path.isfile('/TICI'):
  # UT3G author's sp branch (egpu_loader.py): "/home is an ephemeral overlay on C3XL. Keep AMD firmware and
  # compiler caches across reboots so model startup never depends on a live download." tinygrad's fetch_fw falls
  # back to downloading AMD firmware when /lib/firmware has none; without this the copy is lost on every reboot
  os.environ.setdefault('XDG_CACHE_HOME', '/data/cache')


def check() -> None:
  import tinygrad
  if not Path(tinygrad.__file__).resolve().is_relative_to(VENDORED):
    raise ImportError(f"wrong tinygrad: {tinygrad.__file__}")


def patch_fetch_fw() -> None:
  """AMD firmware from /lib/firmware/*.zst when present (as dp0111pre / sunnypilot compile_modeld do), so
  the driver needs no network on the road."""
  import hashlib
  import pathlib
  from tinygrad import helpers
  if getattr(helpers.fetch_fw, "_dp_patched", False):
    return
  orig = helpers.fetch_fw

  def fetch_fw(path, name, sha256):
    p = pathlib.Path(f"/lib/firmware/{path}/{name}.zst")
    if p.is_file():
      try:
        import zstandard
        blob = zstandard.ZstdDecompressor().stream_reader(p.read_bytes()).read()
        if hashlib.sha256(blob).hexdigest() == sha256:
          return blob
      except Exception:
        pass
    return orig(path, name, sha256)
  fetch_fw._dp_patched = True  # type: ignore[attr-defined]
  helpers.fetch_fw = fetch_fw
