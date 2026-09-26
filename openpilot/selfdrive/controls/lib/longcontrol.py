import numpy as np
from opendbc.car.structs import car
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog
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
    self._probe_t = 0.0        # [2026-09-26] 探针节流用（见 update() 末尾）

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
    _prev_state = self.long_control_state
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

    # [2026-09-26] 状态机探针 —— **跳变那帧必打；低速（v < 3 m/s）时按 1 Hz 打**。
    #
    # 为什么必须加：`stopping` 分支（上面 elif）**完全不看 a_target**，它以
    # 1 m/s²/s 把 last_output_accel 爬向 `CP.stopAccel`(−2.0) 并锁存；是否接管
    # **只取决于 `should_stop`**。而这条状态机此前**没有任何留痕** ⇒ 实车出现
    # "本车静止起步、踩油门车 0.0 km/h 不动"时，无法区分是
    #   ① openpilot 的 stopping 在给 −2.0，还是
    #   ② 车机（TSS2 ACC / PCS）自己在刹 —— OP 这一侧根本没输出。
    # 看 `out=` 与 `aTgt=` 的差即可一眼定性：
    #   `act=0`                             ⇒ OP 纵向**根本没接管**，车是车机自己在刹
    #   `out` 明显比 `aTgt` 更负（≈ −2.0）  ⇒ ① 本模块 stopping 在刹
    #   `out ≈ aTgt`                        ⇒ OP 没在刹，往车机侧（PCM/ACC）查
    # `act=`（= CC.longActive）**必须打**：没有它就没法区分"OP 输出 0"与
    # "OP 压根没接管"——两者都是 out≈0，但结论完全相反。
    # 低速 1 Hz 那条是必需的：光靠"跳变"会在**状态一直不变**时完全静默，
    # 而那恰恰是"卡在 pid / 卡在 stopping"最需要证据的情况。
    #
    # ⚠️ 日志字段一律 `str()` / `int(bool)`：**绝不能把 capnp 枚举喂 int()**
    #    （plannerd 会崩循环）。这里枚举只走 f-string 的 `__str__`。
    # ⚠️ 整段包 try/except：这是 100 Hz 控制关键路径，日志出任何问题都不允许
    #    影响控制输出。
    _changed = self.long_control_state != _prev_state
    self._probe_t += DT_CTRL
    if _changed or (CS.vEgo < 3.0 and self._probe_t >= 1.0):
      self._probe_t = 0.0
      try:
        _trans = (f"{_prev_state} -> {self.long_control_state}" if _changed
                  else str(self.long_control_state))
        cloudlog.info(f"[LongCtrl] {_trans} "
                      f"act={int(active)} vEgo={CS.vEgo * 3.6:.1f} "
                      f"aTgt={a_target:.2f} out={self.last_output_accel:.2f} "
                      f"gas={int(CS.gasPressed)} brk={int(CS.brakePressed)} "
                      f"stSt={int(CS.cruiseState.standstill)} sStop={int(should_stop)}")
      except Exception:
        pass

    return self.last_output_accel
