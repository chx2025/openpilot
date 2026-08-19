import numpy as np

from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  ACCEL_MAX,
  ACCEL_MIN,
  CRASH_DISTANCE,
  FCW_IDXS,
  LEAD_DANGER_FACTOR,
  LongitudinalMpc as UpstreamLongitudinalMpc,
  LongitudinalPlanSource,
  T_IDXS,
  get_T_FOLLOW,
  get_stopped_equivalence_factor,
)
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.experimental.cruise_obstacle import build_cruise_obstacle


MPC_SOURCES = (LongitudinalPlanSource.lead0, LongitudinalPlanSource.lead1, LongitudinalPlanSource.cruise)


class LongitudinalMpc(UpstreamLongitudinalMpc):
  """Current upstream solver fed with the legacy Experimental cruise obstacle."""

  def update(self, radarstate, v_cruise: float, personality=0):
    t_follow = get_T_FOLLOW(personality)
    lead_xv_0 = self.process_lead(radarstate.leadOne)
    lead_xv_1 = self.process_lead(radarstate.leadTwo)
    lead_0_obstacle = lead_xv_0[:, 0] + get_stopped_equivalence_factor(lead_xv_0[:, 1])
    lead_1_obstacle = lead_xv_1[:, 0] + get_stopped_equivalence_factor(lead_xv_1[:, 1])
    cruise_obstacle = build_cruise_obstacle(
      self.x0[1], v_cruise, T_IDXS, t_follow=t_follow, comfort_brake=2.5, stop_distance=6.0,
    )

    x_obstacles = np.column_stack((lead_0_obstacle, lead_1_obstacle, cruise_obstacle))
    self.source = MPC_SOURCES[np.argmin(x_obstacles[0])]

    self.yref[:, :] = 0.0
    for i in range(len(T_IDXS) - 1):
      self.solver.set(i, "yref", self.yref[i])
    self.solver.set(len(T_IDXS) - 1, "yref", self.yref[-1][:-1])

    self.params[:, 0] = ACCEL_MIN
    self.params[:, 1] = ACCEL_MAX
    self.params[:, 2] = np.min(x_obstacles, axis=1)
    self.params[:, 3] = np.copy(self.a_prev)
    self.params[:, 4] = t_follow
    self.params[:, 5] = LEAD_DANGER_FACTOR
    self._apply_backend_params()

    self.run()
    if np.any(lead_xv_0[FCW_IDXS, 0] - self.x_sol[FCW_IDXS, 0] < CRASH_DISTANCE) and radarstate.leadOne.modelProb > 0.9:
      self.crash_cnt += 1
    else:
      self.crash_cnt = 0
