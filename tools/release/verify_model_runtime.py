"""Offroad-only synthetic model check. No cereal publishing or model selection writes."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace

import numpy as np


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--repo", type=Path, required=True)
  parser.add_argument("--modeld-source", type=Path)
  parser.add_argument("--artifact", type=Path)
  parser.add_argument("--catalog", type=Path, help="Official catalog for an explicit artifact's generation/rate/overrides")
  parser.add_argument("--small", action="store_true")
  parser.add_argument("--seconds", type=float, default=30)
  args = parser.parse_args()
  if args.seconds <= 0:
    parser.error("seconds must be positive")
  if args.artifact and not args.catalog:
    parser.error("an explicit artifact requires its catalog metadata")
  repo = args.repo.resolve()
  os.chdir(repo)
  sys.path.insert(0, str(repo))
  from openpilot.common.params import Params
  params = Params()

  def check_offroad():
    if not params.get_bool("IsOffroad") or params.get_bool("IsEngaged"):
      raise RuntimeError("requires offroad and disengaged")

  check_offroad()
  service = subprocess.run(["systemctl", "is-active", "comma"], capture_output=True, text=True)
  if service.stdout.strip() != "inactive":
    raise RuntimeError("stop comma service before running this isolated test")

  if args.modeld_source:
    spec = importlib.util.spec_from_file_location("candidate_modeld", args.modeld_source.resolve())
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
  else:
    from openpilot.sunnypilot.modeld_v2 import modeld as module

  if args.artifact:
    matches = [b for b in json.loads(args.catalog.read_text())["bundles"]
               if any(m["artifact"]["file_name"] == args.artifact.name for m in b["models"])]
    if len(matches) != 1:
      raise RuntimeError("explicit artifact must match exactly one catalog bundle")
    metadata = matches[0]
    bundle = SimpleNamespace(generation=int(metadata["generation"]), is20hz=metadata.get("is_20hz", False),
                             overrides=[SimpleNamespace(key=k, value=v) for k, v in metadata.get("overrides", {}).items()])
    # Process-local injection: use the artifact's own metadata without changing selection Params.
    os.environ.pop("COMBINED_MODEL_PKL", None)
    module.get_active_bundle = lambda **kwargs: bundle
    module._find_driving_pkl = lambda _: str(args.artifact.resolve())

  started = time.monotonic()
  model = module.ModelState(1928, 1208, chestnut=not args.small,
                            loading_progress_callback=lambda n: print(f"loading={n}", flush=True))
  load_seconds = time.monotonic() - started
  frames = {k: np.zeros(model.frame_buf_params[k][3], dtype=np.uint8) for k in model.vision_input_names}
  transforms = {k: np.eye(3, dtype=np.float32) for k in (model._road_key, model._wide_key)}

  class Capture:
    last = None

    def send(self, service, message):
      self.last = message.to_dict()

  capture = Capture()
  telemetry = module.ChestnutState(capture, True) if not args.small else None
  latencies = []
  started = time.monotonic()
  interval = 1.0 / model.constants.MODEL_FREQ
  while time.monotonic() - started < args.seconds:
    check_offroad()
    inputs = {k: np.zeros(v.shape, dtype=v.dtype) for k, v in model.numpy_inputs.items()
              if k not in ("tfm", "big_tfm", "prev_feat")}
    before = time.monotonic()
    callback = telemetry.send if telemetry and len(latencies) % 20 == 0 else None
    outputs = model.run(frames, transforms, inputs, False, after_enqueue=callback)
    elapsed = time.monotonic() - before
    if outputs is None or "plan" not in outputs:
      raise RuntimeError("missing model plan")
    if not all(np.all(np.isfinite(v)) for v in outputs.values() if isinstance(v, np.ndarray)):
      raise RuntimeError("non-finite model output")
    latencies.append(elapsed)
    time.sleep(max(0.0, interval - elapsed))

  if telemetry:
    telemetry.model_fps = len(latencies) / (time.monotonic() - started)
    telemetry.send()
  print(json.dumps({
    "functional_pass": True, "synthetic_inputs": True, "vehicle_messages_sent": 0,
    "modeld_source": str(args.modeld_source or Path(module.__file__)),
    "artifact": str(args.artifact) if args.artifact else "active selected bundle",
    "chestnut": not args.small, "warp_device": model.WARP_DEV, "legacy": model.is_legacy_model,
    "load_seconds": load_seconds, "frames": len(latencies), "target_hz": model.constants.MODEL_FREQ,
    "mean_ms": float(np.mean(latencies) * 1000), "p95_ms": float(np.percentile(latencies, 95) * 1000),
    "over_budget_frames": sum(n > interval for n in latencies),
    "power_limit_env": os.getenv("AM_POWER_LIMIT"), "telemetry": capture.last,
  }, indent=2), flush=True)


if __name__ == "__main__":
  main()
