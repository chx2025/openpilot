"""Shared eGPU output-to-action conversion; no GPU runtime imports."""
import numpy as np
from cereal import log
from openpilot.selfdrive.controls.lib.drive_helpers import get_accel_from_plan, get_curvature_from_plan, smooth_value
from openpilot.selfdrive.modeld.constants import ModelConstants, Plan
MIN_LAT_CONTROL_SPEED = 0.3


class EgpuAction:
  @property
  def mlsim(self) -> bool:
    return self.generation >= 11

  def get_action(self, model_output: dict[str, np.ndarray], prev_action: log.ModelDataV2.Action,
                 lat_action_t: float, long_action_t: float, v_ego: float, dp_lat_offset_cm: int) -> log.ModelDataV2.Action:
    # sunnypilot modeld_v2 ModelState.get_action_from_model + dp lateral offset
    if 'action' not in model_output:
      plan = model_output['plan'][0]
      desired_accel, _ = get_accel_from_plan(plan[:, Plan.VELOCITY][:, 0], plan[:, Plan.ACCELERATION][:, 0],
                                             ModelConstants.T_IDXS, action_t=long_action_t)
      if not self.mlsim and 'desired_curvature' in model_output:
        desired_curvature = float(model_output['desired_curvature'][0, 0])
      else:
        desired_curvature = get_curvature_from_plan(plan[:, Plan.T_FROM_CURRENT_EULER][:, 2], plan[:, Plan.ORIENTATION_RATE][:, 2],
                                                    ModelConstants.T_IDXS, v_ego, lat_action_t)
    else:
      desired_accel = model_output['action'][0, 1]
      desired_curvature = model_output['action'][0, 0] / (max(1.0, v_ego))**2

    # dp - lateral offset (dp0111pre selfdrive/modeld/modeld.py formula), added to the target BEFORE smoothing:
    # prev_action already carries it, adding it after a non-zero smoothing would amplify it to offset / alpha
    if dp_lat_offset_cm != 0:
      desired_curvature += 2.0 * (dp_lat_offset_cm / 100.0) / 900.0

    stop = v_ego < 0.3 and desired_accel < 0.1
    desired_accel = smooth_value(desired_accel, prev_action.desiredAcceleration, self.LONG_SMOOTH_SECONDS)
    if self.generation >= 10:  # smooth curvature for post FOF models
      if v_ego > MIN_LAT_CONTROL_SPEED:
        desired_curvature = smooth_value(desired_curvature, prev_action.desiredCurvature, self.LAT_SMOOTH_SECONDS)
      else:
        desired_curvature = prev_action.desiredCurvature
    return log.ModelDataV2.Action(desiredCurvature=float(desired_curvature), desiredAcceleration=float(desired_accel), shouldStop=bool(stop))
