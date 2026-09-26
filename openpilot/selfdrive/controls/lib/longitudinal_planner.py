#!/usr/bin/env python3
import math
import numpy as np

import openpilot.cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan, should_stop
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params

from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP
from openpilot.sunnypilot.selfdrive.controls.lib.turn_decel import TurnDecelController
from openpilot.sunnypilot.selfdrive.controls.lib.gas_override import GasOverrideController
from openpilot.sunnypilot.selfdrive.controls.lib.stop_soften import StopSoftenController
from openpilot.sunnypilot.selfdrive.controls.lib.crawl_gate import (
  CRAWL_GATE_A_BOOST, CrawlGate)
from openpilot.sunnypilot.selfdrive.controls.lib.creep_guard import CreepGuard
from openpilot.sunnypilot.selfdrive.controls.lib.creep_step import CreepStep

A_CRUISE_MAX_BP = [0., 2.77, 5.55, 8.33, 11.11, 13.89, 16.6, 19.4, 22.22, 25, 27.78, 33.33]
A_CRUISE_MAX_VALS = [1.10, 0.9, 0.80, 0.65, 0.50, 0.40, 0.35, 0.35, 0.35, 0.33, 0.31, 0.29]
J_CRUISE_VALS = [1.10, 0.9, 0.80, 0.65, 0.50, 0.40, 0.35, 0.35, 0.35, 0.33, 0.31, 0.29]
A_CRUISE_MIN = -1.2
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

# 实验模式（e2e）下的正加速上限。
#
# 原逻辑用 opendbc 的 ACCEL_MAX（= 2.0，那是**车辆物理上限**，用于最终
# np.clip 与 MPC 约束，不能动）：`max_accel = ACCEL_MAX if e2e else 查表`。
# 也就是说实验模式下 a_cruise 的加速上限直接顶到 2.0（≈0.20 g），
# 比非实验模式的查表值（低速 1.10 → 高速 0.29）猛得多，畅通路段模型
# 会把 desiredAcceleration 拉满，体感偏冲。
#
# 这里单独把 e2e 分支收窄到 1.3（≈0.13 g）。
# ★生效机制：a_cruise 永远在 candidates 里且 min() 取小 ⇒ 只要压住
#   a_cruise 的上限，最终 output_a_target 就不可能超过它，
#   无论 MPC 轨迹或 e2e 候选给多大都会被 min 掉。
#   （所以不需要去改 long_mpc.py 的求解约束，也就避开了动求解器的风险。）
E2E_MAX_ACCEL = 1.3

def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py

def get_cruise_accel(e2e, v_cruise, v_ego, a_cruise_prev, angle_steers, CP, dt, accel_coast, allow_throttle):
  # 实验模式走 E2E_MAX_ACCEL(1.3)，非实验模式走速度查表（低速 1.10 → 高速 0.29）。
  # 见上方 E2E_MAX_ACCEL 的说明：这里压住 a_cruise 的上限，就压住了整条链路的正加速上限。
  max_accel = E2E_MAX_ACCEL if e2e else get_max_accel(v_ego)

  if not e2e:
    a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
    a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
    a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))
    max_accel = min(max_accel, a_x_allowed)
    if not allow_throttle:
      clipped_accel_coast = max(accel_coast, ACCEL_MIN)
      coast_limit = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [max_accel, clipped_accel_coast])
      max_accel = min(max_accel, coast_limit)

  target_accel = np.clip(v_cruise - v_ego, A_CRUISE_MIN, max_accel)
  j_cruise = np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS)
  target_accel = float(np.clip(target_accel, a_cruise_prev - j_cruise * dt, a_cruise_prev + j_cruise * dt))

  return target_accel


class LongitudinalPlanner(LongitudinalPlannerSP):
  def __init__(self, CP, CP_SP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    LongitudinalPlannerSP.__init__(self, self.CP, CP_SP, self.mpc)
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.a_cruise = init_a
    self.output_a_target = init_a
    self.output_should_stop = False

    # 打灯减速 + 大角度限加速（sunnypilot 追加，见 turn_decel.py）
    self.turn_decel = TurnDecelController()

    # 驾驶员踩油门让位（需求三条 + 开关见 gas_override.py 文件头）。
    # 运行期开关是参数 GasPedalOverride（设置页 "Gas Pedal Override"，默认开），
    # 由控制器自己按 1 Hz 轮询，所以这里只把 Params 句柄注入进去。
    self.gas_override = GasOverrideController(Params())

    # 「低速/静止 + 前方静止目标」的减速柔化（见 stop_soften.py 文件头）。
    # 二版（2026-09-26）：只在**场景闸门**打开时参与 —— 即"本车静止 + 前车静止"，
    # 且 2 m 是最小车间距（用户："保证最小车距为 2"）。
    self.stop_soften = StopSoftenController()

    # 「静止起步蠕动」场景闸门（见 crawl_gate.py 文件头）。
    # 本车静止 + 前车静止 ⇒ 开门（latch，覆盖整个"起步→蠕动→再停住"），
    # 此后 gas_override 的 e/f 豁免与 stop_soften 的柔化才允许参与；
    # 其余一切工况（行进中减速接近、前车在动、高速）一律跳过。
    self.crawl_gate = CrawlGate()

    # 「前车静止 + 未踩油门」自动靠近守卫（第 6 版，见 creep_guard.py 文件头）。
    # 它只压上限（min），与 crawl_gate / stop_soften 的「抬下限」方向相反，
    # 所以在 planner 里必须排在最后（见接线段说明），否则收紧量会被 max 抬回去。
    self.creep_guard = CreepGuard()

    # 「松油门 ⇒ 定量蠕动一步」（第 8 版，见 creep_step.py 文件头）。
    # 需求：「松油门，如果车间距大于 4，蠕动 1 米；小于 2 m 直接刹死；
    #        二者之间蠕动 0.5 米」+「像油车怠速蠕行」。
    # 它同时提供一条独立的近距硬底线（dRel ≤ 2 m ⇒ −2.0），闸门内外都生效。
    self.creep_step = CreepStep()

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

  def update(self, sm):
    LongitudinalPlannerSP.update(self, sm)

    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    if sm['controlsState'].forceDecel:
      v_cruise = 0.0

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET
    reset_state = reset_state or not v_cruise_initialized

    throttle_probs = sm['modelV2'].meta.disengagePredictions.gasPressProbs
    throttle_prob = throttle_probs[1] if len(throttle_probs) > 1 else 1.0
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['vehicleParameters'].angleOffsetDeg

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.output_a_target = np.clip(sm['carState'].aEgo, ACCEL_MIN, ACCEL_MAX)
      self.a_cruise = self.output_a_target
      # 未接管/车速未就绪时清掉打灯减速的累计状态，避免再接管时
      # 立刻按「转向灯已开很久」的旧状态减速
      self.turn_decel.reset()
      # 同理清掉踩油门让位的相位机（否则再接管时可能带着旧的 coast/hold 相位）
      self.gas_override.reset()
      # 柔化模块无累计状态以外的相位，但保持与上面两个模块一致的复位语义
      self.stop_soften.reset()
      # 场景闸门：上下文已丢失（不知道车从哪来），必须重新满足
      # "本车静止 + 前车静止"才开门，不能把上一段行程的 latch 带过来
      self.crawl_gate.reset()
      # 自动靠近守卫无 latch，但保持与其它模块一致的复位语义
      self.creep_guard.reset()

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    # Get new v_cruise and a_target from Smart Cruise Control and Speed Limit Assist
    v_cruise, self.output_a_target = LongitudinalPlannerSP.update_targets(self, sm, self.v_desired_filter.x, self.output_a_target, v_cruise)

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.output_a_target)
    self.mpc.update(sm['radarState'], personality=sm['selfdriveState'].personality)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Save starting point for next iteration
    a_prev = self.output_a_target

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                              action_t=action_t)
    output_should_stop_mpc = should_stop(v_ego, output_a_target_mpc)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    is_e2e = self.is_e2e(sm)

    self.a_cruise = get_cruise_accel(is_e2e, v_cruise, v_ego,
                                     self.a_cruise, steer_angle_without_offset, self.CP, self.dt,
                                     accel_coast, self.allow_throttle)
    cruise_should_stop = should_stop(v_ego, self.a_cruise)

    candidates = [(output_a_target_mpc, self.mpc.source, output_should_stop_mpc),
                  (self.a_cruise, LongitudinalPlanSource.cruise, cruise_should_stop)]
    if is_e2e:
      candidates.append((output_a_target_e2e, LongitudinalPlanSource.e2e, output_should_stop_e2e))

    output_a_target, self.mpc.source, _ = min(candidates, key=lambda c: c[0])
    self.output_should_stop = any(should_stop for _, _, should_stop in candidates)

    # 打灯减速 + 大角度限加速（sunnypilot 追加）。
    # 放在 candidates 的 min() 之后、np.clip 之前：
    #   - min() 保证不会盖过 candidates 里更保守的一方（FCW/前车/MPC 该刹还是刹）
    #   - 同时不会被 e2e 的正加速度覆盖掉"不许加速"的约束
    # 两种意图：打转向灯后弯道减速；以及方向角度过大时（与转向灯无关）不许加速
    blinker_on = bool(sm['carState'].leftBlinker or sm['carState'].rightBlinker)
    turn_decel_res = self.turn_decel.update(
      blinker_on=blinker_on,
      v_ego=v_ego,
      steering_angle_deg=steer_angle_without_offset,
      dt=self.dt,
      # [2026-09-21] 驾驶员踩油门 -> 本模块完全让位（含松油门后 2 s 宽限）。
      # 依据：实车探针 05:05:52 三条候选 aMpc=2.33/aCruise=2.00/aE2e=+1.11 全为正，
      # 最终 aTgt 却是 0.000（gas=1）—— min() 不可能产生 0，唯一来源就是这里的
      # block_accel。机理与安全论证见 turn_decel.py 文件头「驾驶员加速意图让位」。
      gas_pressed=sm['carState'].gasPressed,
    )
    if turn_decel_res.a_target_override is not None:
      output_a_target = min(output_a_target, turn_decel_res.a_target_override)
    if turn_decel_res.block_accel and output_a_target > 0.0:
      output_a_target = 0.0

    # 驾驶员踩油门让位（需求三条见 gas_override.py 文件头）。
    # 位置：candidates min() 和 turn_decel 之后、np.clip 之前。
    #   - 它只**抬升下限**（max），所以不可能抢走 turn_decel / FCW / 前车 /
    #     MPC 任何一方"更保守"的结论；反过来 turn_decel 压的是上限、本模块抬的
    #     是下限，两条约束互不覆盖。
    #   - 未接管（reset_state）时不参与，避免把"没接管"搅进相位机，也避免
    #     在没接管时武装松油门窗口。
    if not reset_state:
      lead = sm['radarState'].leadOne

      # ★ 场景闸门（见 crawl_gate.py 文件头）：本车静止 + 前车静止 ⇒ 开门
      #   （latch，覆盖整个"起步→蠕动→再停住"过程）；踩刹车 / 前车起步 /
      #   车速超 3 m/s / 贴到 2 m ⇒ 关门。
      #   必须在两个柔性模块**之前**更新 —— 它们是同一帧的同一份状态。
      crawl = self.crawl_gate.update(
        v_ego=v_ego,
        lead_present=bool(lead.present),
        d_rel=float(lead.dRel),
        v_lead=float(lead.vLead),
        brake_pressed=bool(sm['carState'].brakePressed),
      )
      if self.crawl_gate.last_change:
        # 闸门进出是"这段为什么柔化 / 为什么不柔化"的关键证据，必落盘。
        # ★ `present` / `brk` / `why` 三个字段是 2026-09-26 补的：
        #   上一版只打了 vEgo/dRel/vLead，结果实车出现 20 Hz enter/exit 抖动
        #   时，三个可见值**完全不变** ⇒ 无法判断到底是哪条退出判据在生效。
        #   补上之后，看 `why=` 一眼就知道关门原因（min_gap / brake /
        #   lead_lost / lead_moving / vEgo）。
        cloudlog.info(f"[CrawlGate] {self.crawl_gate.last_change} "
                      f"vEgo={v_ego * 3.6:.1f} dRel={lead.dRel:.1f} "
                      f"vLead={lead.vLead * 3.6:.1f} "
                      f"present={int(bool(lead.present))} "
                      f"brk={int(bool(sm['carState'].brakePressed))} "
                      f"why={self.crawl_gate.last_reason or '-'}")

      gas_res = self.gas_override.update(
        gas_pressed=bool(sm['carState'].gasPressed),
        v_ego=v_ego,
        # 用**原逻辑实际执行**的目标速度（SP 的 SCC/SLA 若在压低速度，这里就是
        # 压低后的值）：这样"滑行到设定速度"收回时，原逻辑刚好也在那里停止制动，
        # 交接处没有阶跃。
        v_cruise=v_cruise,
        accel_coast=accel_coast,
        a_target_in=output_a_target,
        dt=self.dt,
        lead_present=bool(lead.present),
        d_rel=float(lead.dRel),
        v_lead=float(lead.vLead),
        fcw=bool(self.fcw),
        force_decel=bool(sm['controlsState'].forceDecel),
        # e/f 的**场景豁免**：只有"本车静止 + 前车静止"闸门打开时才豁免（见 crawl_gate.py）
        crawl_gate=crawl,
      )
      output_a_target = gas_res.a_target_out
      # 需求 3「不要有空档期」：低速时 LongControl 会因为 should_stop 把状态机
      # 切到 stopping（不再跟随 a_target）。让位期间一并清掉该意图。
      # ★ 2026-09-26 三版：**场景闸门打开期间也清「停车意图」**。
      #
      # 为什么：`should_stop(v_ego, a_target) = v_ego < 0.3 and a_target < 0.1`
      # （见 gas_override.py 文件头）在「本车静止 + 前车静止」时**恒为 True**
      # —— 候选池第 190 行还并入了 e2e 的 `modelV2.action.shouldStop`，前方几米
      # 停着车时它必然置位。而 `longcontrol.py` 的 `stopping` 分支**完全不看
      # a_target**，只把 last_output_accel 以 1 m/s²/s 爬向 `CP.stopAccel`(−2.0)
      # 并锁存 ⇒ 驾驶员踩的那一脚油门（物理油门，本车无 gas interceptor）被
      # −2.0 正好抵消 ⇒ 车 0.0 km/h 纹丝不动。
      #
      # 实车证据（2026-09-26 09:55，swaglog.0000000966，重启后新代码）：
      #   [CrawlGate] enter vEgo=1.0 dRel=3.7 vLead=0.1     ← 闸门正常开门
      #   [GasOverride] pressed  gas=1 vEgo=0.0 aIn=-0.03   ← 让位也正常，上游只给 −0.03
      # 两个柔性模块都在正常工作、上游根本没有硬刹，车却不动 ⇒ 减速度只可能来自
      # 这条状态机锁存。旧做法只在**踩油门那几帧**（gas_override 的
      # `suppress_should_stop`）清，让位窗口一过（松油门 0.5 s）它就回来 ⇒ 正是
      # 用户说的「还是直接刹住」。
      #
      # 为什么安全：清理范围被闸门圈得很死 ——
      #   * 进入要求「本车静止 + 有前车 + 前车静止 + dRel > 2 m」；
      #   * 退出任一成立即恢复：踩刹车 / 前车起步 / v_ego > 3 m/s / **dRel ≤ 2 m**；
      #   * 贴到 2 m 闸门立刻关 ⇒ should_stop 立即恢复 ⇒ stopping 接管 ⇒
      #     **停位就是需求里的最小车距 2 m**，该停的时候一秒不晚；
      #   * 闸门窗口内 a_target 仍由候选池 + gas_override 地板 + 柔化共同决定，
      #     FCW / forceDecel / a、b 两条绝对距离例外（< 2 m、< 4 m 且快 10 km/h）
      #     一个字没动 —— 真正需要刹的时刻一点没让。
      if gas_res.suppress_should_stop or crawl:
        self.output_should_stop = False
      if gas_res.log is not None:
        cloudlog.info(gas_res.log)

      # ---- 闸门 latch 期间「放行蠕动」（第 8 版；论证见 crawl_gate.py / creep_step.py）----
      # 演进史（每一版都是被实车否掉后重写的，别回退旧结论）：
      #   5 版：闸门内「未踩油门 ⇒ 地板 0.0」把上游减速整段清零 ⇒ 从 8 m 滑到 2.0 m。
      #   7 版：改成「温柔减速 + 底线保护」，但实车证明这一档**从未参与运算**
      #         （接线是 max，而上游低速给 0 ~ -0.5 永远更高）。
      #   8 版：删掉未踩油门那一档，闸门内只留踩油门放行；松油门后的定量蠕动
      #         交给 creep_step.py 的位置闭环**主动完成**。
      #   * 踩油门 ⇒ +0.6：放行油门（旧版把 aTarget 发 0，车机 ACC 会抑制油门）
      # 位置：gas_override 之后、stop_soften 之前（同为“只抬下限”，max 幂等）。
      if crawl and bool(sm['carState'].gasPressed):
        # 第 8 版：闸门内**只保留"踩着油门"的放行档**。
        #   为什么删掉"未踩油门"那一档：实车实证（swaglog 1185/1190）它发出的
        #   `aFloor=-0.89` 与同帧 `aOut=+0.00` 并存 —— 接线是
        #   `a_target = max(a_target, floor)`，而上游在低速给的是 0 ~ -0.5，
        #   永远不低于地板 ⇒ 这一档从未真正参与过运算（减法方向是死的）。
        #   真正让车停下的是上游自己的 -0.17 ~ -0.50。
        #   现在松油门后的蠕动改由 creep_step.py 的位置闭环**主动完成**。
        crawl_gap = self.crawl_gate.gap_m
        crawl_floor = CRAWL_GATE_A_BOOST
        crawl_tag = 'boost'
        if output_a_target < crawl_floor:
          output_a_target = crawl_floor
        self._crawl_a_t = getattr(self, '_crawl_a_t', 0.0) + self.dt
        if self._crawl_a_t >= 1.0 or getattr(self, '_crawl_a_tag', '') != crawl_tag:
          self._crawl_a_t = 0.0
          self._crawl_a_tag = crawl_tag
          cloudlog.info(
            f"[CrawlAid] {crawl_tag} vEgo={v_ego * 3.6:.1f} dRel={crawl_gap:.1f} "
            f"gas=1 aFloor={crawl_floor:+.2f} aOut={output_a_target:+.2f}")
      else:
        self._crawl_a_t = 0.0
        self._crawl_a_tag = ''

      # ---- 定量蠕动步（第 8 版；需求与安全论证见 creep_step.py 文件头）----
      # 位置：闸门放行档之后、stop_soften 之前。
      # 接线方向与其它柔性模块相反 ——
      #   * 蠕动推进（a ≥ 0）：max —— 抬下限，放行驾驶员的蠕动；
      #   * 近距硬底线（dRel ≤ 2 m）：min —— 压上限，直接刹死。
      # 安全：本模块只在「闸门内 + 油门下落沿」触发一步，且上游强减速
      # （≤ -1.0）/ FCW / forceDecel 一出现就立刻让位（见 creep_step.py）。
      self.creep_step.update(
        v_ego=v_ego,
        gas_pressed=bool(sm['carState'].gasPressed),
        d_rel=float(lead.dRel),
        lead_present=bool(lead.present),
        v_lead=float(lead.vLead),
        crawl_gate=crawl,
        brake_pressed=bool(sm['carState'].brakePressed),
        fcw=bool(self.fcw),
        force_decel=bool(sm['controlsState'].forceDecel),
        a_target_in=output_a_target,
        dt=self.dt,
      )
      if self.creep_step.hard_stop:
        output_a_target = min(output_a_target, self.creep_step.a_target)
      elif self.creep_step.active:
        output_a_target = max(output_a_target, self.creep_step.a_target)
      if self.creep_step.log is not None:
        cloudlog.info(self.creep_step.log)

      # 「静止起步蠕动」场景下的减速柔化（需求与安全论证见 stop_soften.py 文件头）。
      # 位置：gas_override 之后、np.clip 之前。
      #   - 与 gas_override 同为"只抬下限"，两者互不覆盖（连续两次 max）
      #   - 二版（2026-09-26）触发条件是**场景闸门**（本车静止 + 前车静止），
      #     行进中减速接近静止前车不再柔化 ⇒ 不再有"柔化撤出"造成的阶跃
      #   - 地板含物理下界（恰好能在前车前 2 m 停住）⇒ 不会因为柔化追尾
      soften_res = self.stop_soften.update(
        v_ego=v_ego,
        lead_present=bool(lead.present),
        d_rel=float(lead.dRel),
        v_lead=float(lead.vLead),
        fcw=bool(self.fcw),
        force_decel=bool(sm['controlsState'].forceDecel),
        a_target_in=output_a_target,
        dt=self.dt,
        # 场景闸门：非"本车静止 + 前车静止"一律跳过（见 crawl_gate.py）
        crawl_gate=crawl,
      )
      output_a_target = soften_res.a_target_out
      if soften_res.log is not None:
        cloudlog.info(soften_res.log)

      # ---- 自动靠近守卫（第 6 版；论证见 creep_guard.py 文件头）----
      # 位置：crawl aid / stop_soften **之后**、np.clip 之前。
      #   * 它们只抬下限（max），本模块只压上限（min），方向相反
      #     ⇒ 必须排最后，否则收紧量会被它们的 max 又抬回去；
      #   * 踩油门时本模块完全不介入
      #     ⇒ 「蠕动到 2 m」仍然只由驾驶员的脚触发。
      guard_res = self.creep_guard.update(
        v_ego=v_ego,
        lead_present=bool(lead.present),
        d_rel=float(lead.dRel),
        v_lead=float(lead.vLead),
        gas_pressed=bool(sm['carState'].gasPressed),
        fcw=bool(self.fcw),
        force_decel=bool(sm['controlsState'].forceDecel),
        a_target_in=output_a_target,
        dt=self.dt,
      )
      output_a_target = guard_res.a_target_out
      if guard_res.log is not None:
        cloudlog.info(guard_res.log)

    self.output_a_target = np.clip(output_a_target, ACCEL_MIN, ACCEL_MAX)

    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.output_a_target + a_prev) / 2.0

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks()

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.present
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    pm.send('longitudinalPlan', plan_send)

    self.publish_longitudinal_plan_sp(sm, pm)
