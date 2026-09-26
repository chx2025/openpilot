import numpy as np
from opendbc.car.structs import car
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.common.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

LongCtrlState = car.CarControl.Actuators.LongControlState


def long_control_state_trans(CP_SP, active, long_control_state,
                             should_stop, brake_pressed, cruise_standstill):
  # Gas Interceptor
  cruise_standstill = cruise_standstill and not CP_SP.enableGasInterceptor

  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if not starting_condition:
        long_control_state = LongCtrlState.stopping
      else:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.pid:
      if should_stop:
        long_control_state = LongCtrlState.stopping

  return long_control_state

class LongControl:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController(0.0, (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             rate=1 / DT_CTRL)
    self.last_output_accel = 0.0

  def reset(self):
    self.pid.reset()

  def update(self, active, CS, a_target, should_stop, accel_limits, freeze_integrator=False):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    # [2026-09-26 修 1] 驾驶员踩油门时，车机 ACC 的"停车保持"不再锁死状态机。
    #
    # 为什么需要：`CS.cruiseState.standstill` 来自**车机 CAN**（opendbc/car/toyota/
    # carstate.py: `= pcm_acc_status == 7`），**与车速无关** —— 车已经动起来它仍可能
    # 是 True。而下面的 stopping 分支**完全忽略 a_target**，只把 last_output_accel
    # 以 1.0 m/s²/s 爬向 CP.stopAccel(−2.0) 并锁存 ⇒ 踩油门让位（gas_override）
    # 把 should_stop 清了也出不来，油门 +2.0 刚好被 −2.0 抵消 ⇒ 车"只动一下"、
    # 一松油门"直接被按死"、点头严重。实车证据与复现见
    #   gas_patch/evidence/静止起步踩油门被刹_根因修正_2026-09-26.md
    #   gas_patch/verify_park_start3.py
    #
    # 语义：等价于官方 `CP_SP.enableGasInterceptor` 那一行的意图（"油门优先"），
    # 只是我们的车没有装硬件油门拦截器，所以用 gasPressed 直接表达。
    #
    # 为什么是安全的：单独这一行**不改变任何行为** —— 要进入 pid 还需
    # `should_stop` 为 False，而那只由让位模块在"确认无任何安全例外"之后才清。
    # 所以 `GasPedalOverride=0` 时本行逐位等价于改前（原版依然会被 standstill 锁住），
    # 且 a/b 两条绝对距离例外（<2 m / <4 m+快 10 km/h）一个字没动。
    self.long_control_state = long_control_state_trans(self.CP_SP, active, self.long_control_state,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill and not CS.gasPressed)
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        # TODO: can we just go straight to stopAccel?
        output_accel -= 1.0 * DT_CTRL  # m/s^2/s while trying to stop
      self.reset()

    else:  # LongCtrlState.pid
      error = a_target - CS.aEgo
      output_accel = self.pid.update(error, speed=CS.vEgo,
                                     feedforward=a_target,
                                     freeze_integrator=freeze_integrator)

    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel
