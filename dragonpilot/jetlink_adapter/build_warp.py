#!/usr/bin/env python3
"""
The comma-side warp jetlink 0.8.7 runs, as openpilot's modeld pickles its own.

dp: jetlink 0.8.7 no longer builds a warp. It loads openpilot's
models/driving_warp_{w}x{h}_tinygrad.pkl, a dict {'run': TinyJit,
'input_specs': {name: (shape, dtype, device)}} whose JIT takes both camera
frames as one (2, frame bytes) uint8 `input_frame` and their transforms as one
(2, 3, 3) float32 `M_inv`, and makes the (2, 6, H/2, W/2) uint8 frame the link
sends (jetlink.openpilot.warp.Warps.load checks every one of those).
dp0111pre's modeld fuses its warp into the queueing warp_enqueue JIT and has no
such pickle, so this compiles one in exactly that format from dp's own
make_frame_prepare, the graph warp_enqueue runs: what jetlink warps is what dp's
small model would have warped.

Run by dragonpilot/jetlink_adapter/SConscript on the comma (larch64):
  python3 -m dragonpilot.jetlink_adapter.build_warp --camera 1928x1208 --model 512x256 --output PATH
"""
import argparse
import os
import pickle
import sys
from pathlib import Path


def size(text: str) -> tuple[int, int]:
  w, sep, h = text.lower().partition('x')
  if not sep:
    raise argparse.ArgumentTypeError(f"expected WxH, not {text!r}")
  return int(w), int(h)


def frame_bytes(cam_w: int, cam_h: int) -> int:
  """What the warp reads of a camera buffer: the Y and UV planes, as openpilot's
  SConscript sizes its driving warp (stride * (y_height + uv_height)), short of
  the VisionIPC buffer's padded size. jetlink's Warp memmoves this much a frame."""
  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
  stride, y_height, uv_height, _ = get_nv12_info(cam_w, cam_h)
  return stride * (y_height + uv_height)


def compile_warp(cam_w: int, cam_h: int, model_w: int, model_h: int, runs: int = 3) -> dict:
  # compile_modeld first: it patches tinygrad's firmware fetch as it loads
  from openpilot.selfdrive.modeld.compile_modeld import NV12Frame, make_frame_prepare
  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
  import numpy as np
  from tinygrad.device import Device
  from tinygrad.engine.jit import TinyJit
  from tinygrad.tensor import Tensor

  stride, y_height, uv_height, _ = get_nv12_info(cam_w, cam_h)
  size = frame_bytes(cam_w, cam_h)
  frame_prepare = make_frame_prepare(NV12Frame(cam_w, cam_h, stride, y_height, uv_height, size), model_w, model_h)

  specs = {'input_frame': ((2, size), np.dtype(np.uint8).str, Device.DEFAULT),
           'M_inv': ((2, 3, 3), np.dtype(np.float32).str, Device.DEFAULT)}

  def run(input_frame, M_inv):
    M_inv = M_inv.to(Device.DEFAULT).realize()
    return Tensor.stack(*(frame_prepare(input_frame[i], M_inv[i]) for i in range(2)))

  jit = TinyJit(run, prune=True)
  # one set of inputs: TinyJit captures against the buffers it is first handed, on
  # the second call; a pickle taken earlier is an empty jit that computes nothing.
  # Random, so nothing constant-folds
  rng = np.random.default_rng(42)
  frames = Tensor(rng.integers(0, 256, (2, size), dtype=np.uint8), device=Device.DEFAULT).realize()
  tfm = Tensor(np.zeros((2, 3, 3), dtype=np.float32), device=Device.DEFAULT).realize()
  for _ in range(max(runs, 3)):
    tfm.assign(Tensor((rng.standard_normal((2, 3, 3)) * 8).astype(np.float32), device=Device.DEFAULT)).realize()
    jit(input_frame=frames, M_inv=tfm).realize()
  Device[Device.DEFAULT].synchronize()

  made = tuple(jit.captured.ret.shape)
  if made != (2, 6, model_h // 2, model_w // 2):
    raise RuntimeError(f"the warp makes {made}, not a {model_w}x{model_h} model's input")
  return {'metadata': {'builder': 'dragonpilot.jetlink_adapter.build_warp'}, 'run': jit, 'input_specs': specs}


def main(argv=None) -> int:
  p = argparse.ArgumentParser(prog='python3 -m dragonpilot.jetlink_adapter.build_warp', description=__doc__.split('\n\n')[0])
  p.add_argument('--camera', type=size, required=True, help='camera resolution, WxH')
  p.add_argument('--model', type=size, required=True, help='model input, WxH')
  p.add_argument('--output', type=Path, required=True)
  args = p.parse_args(argv)
  (cam_w, cam_h), (model_w, model_h) = args.camera, args.model
  print(f"Compiling jetlink driving warp for {cam_w}x{cam_h} -> {model_w}x{model_h}...")
  warp = compile_warp(cam_w, cam_h, model_w, model_h)
  out = args.output
  out.parent.mkdir(parents=True, exist_ok=True)
  # through a temporary: a compile killed mid-write leaves nothing under the name load() opens
  tmp = out.with_suffix('.pkl.tmp')
  with open(tmp, 'wb') as f:
    pickle.dump(warp, f)
  os.replace(tmp, out)
  print(f"  Saved to {out}")
  return 0


if __name__ == "__main__":
  sys.exit(main())
