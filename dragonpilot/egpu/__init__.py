"""
dp - eGPU (chestnut / USB AMD GPU) big-model support, ported from sunnypilot, models compiled on the device.

Layout:
  constants.py    URLs, USB ids, cache keys, tinygrad flags, param keys, paths (no heavy imports)
  usb.py          eGPU presence detection (sysfs)
  bundles.py      catalog merge, local state (ONNX / compiled / active), routing decision
  manager.py      daemon: presence param, catalog fetch, ONNX download (jetlink registry), compile, delete, clear
  tg_env.py       puts the vendored sunnypilot tinygrad (tinygrad_spm/) first on sys.path, in a fresh interpreter
  compile.py      on-device compile: QCOM warp JIT per camera + eGPU model JIT per ONNX (stateful or queued)
  runner.py       EgpuModelState: QCOM warp -> crop over USB -> eGPU model
  modeld_egpu.py  entry point, executed as a NativeProcess (fresh interpreter, vendored tinygrad)
"""
