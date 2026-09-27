"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import messaging, custom
from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.sunnypilot.selfdrive.controls.lib.dec.dec import DynamicExperimentalController
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_alerts_helper import E2EAlertsHelper
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.smart_cruise_control import SmartCruiseControl
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver
from openpilot.sunnypilot.selfdrive.controls.lib.traffic_stop import TrafficStopController
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.models.helpers import get_active_bundle

DecState = custom.LongitudinalPlanSP.DynamicExperimentalControl.DynamicExperimentalControlState
LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource

# [LongWin] 探针用：候选来源 → 短名（**绝不 int(枚举)**，那会让 plannerd 崩循环）
_LW_NAMES = {
  LongitudinalPlanSource.cruise: "cruise",
  LongitudinalPlanSource.sccVision: "sccV",
  LongitudinalPlanSource.sccMap: "sccM",
  LongitudinalPlanSource.speedLimitAssist: "sla",
  LongitudinalPlanSource.trafficStop: "tstop",
}
_LW_ORDER = tuple(_LW_NAMES.keys())


class LongitudinalPlannerSP:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, mpc):
    self.events_sp = EventsSP()
    self.resolver = SpeedLimitResolver()
    self.dec = DynamicExperimentalController(CP, mpc)
    self.scc = SmartCruiseControl()
    self.resolver = SpeedLimitResolver()
    self.sla = SpeedLimitAssist(CP, CP_SP)
    # 红灯 / 停止标志「虚拟停止线」辅助（机制与安全边界见 traffic_stop.py 文件头）。
    # 在**这里**实例化（SP 基类），主 planner `LongitudinalPlanner` 直接用
    # `self.traffic_stop`。输出两条：
    #   ① stop_dist_m  → 主 planner 传给 mpc.update(traffic_stop_obstacle_m=...)
    #   ② output_v_target/output_a_target → 下面 targets 字典的候选之一
    self.traffic_stop = TrafficStopController()
    self.generation = int(model_bundle.generation) if (model_bundle := get_active_bundle()) else None
    self.source = LongitudinalPlanSource.cruise
    self.e2e_alerts_helper = E2EAlertsHelper()

    self.output_v_target = 0.
    self.output_a_target = 0.

    # [LongWin] 探针节流（见 update_targets 末段）
    self._lw_t = 999.0
    self._lw_last = None

  def is_e2e(self, sm: messaging.SubMaster) -> bool:
    experimental_mode = sm['selfdriveState'].experimentalMode
    if not self.dec.active():
      return experimental_mode

    return experimental_mode and self.dec.mode() == "blended"

  def update_targets(self, sm: messaging.SubMaster, v_ego: float, a_ego: float, v_cruise: float) -> tuple[float, float]:
    CS = sm['carState']
    v_cruise_cluster_kph = min(CS.vCruiseCluster, V_CRUISE_MAX)
    v_cruise_cluster = v_cruise_cluster_kph * CV.KPH_TO_MS

    long_enabled = sm['carControl'].enabled
    long_override = sm['carControl'].cruiseControl.override

    # Smart Cruise Control
    self.scc.update(sm, long_enabled, long_override, v_ego, a_ego, v_cruise)

    # Speed Limit Resolver
    self.resolver.update(v_ego, sm)

    # Speed Limit Assist
    has_speed_limit = self.resolver.speed_limit_valid or self.resolver.speed_limit_last_valid
    self.sla.update(long_enabled, long_override, v_ego, a_ego, v_cruise_cluster, self.resolver.speed_limit,
                    self.resolver.speed_limit_final_last, has_speed_limit, self.resolver.distance, self.events_sp)

    targets = {
      LongitudinalPlanSource.cruise: (v_cruise, a_ego),
      LongitudinalPlanSource.sccVision: (self.scc.vision.output_v_target, self.scc.vision.output_a_target),
      LongitudinalPlanSource.sccMap: (self.scc.map.output_v_target, self.scc.map.output_a_target),
      LongitudinalPlanSource.speedLimitAssist: (self.sla.output_v_target, self.sla.output_a_target),
      # 红灯辅助：未激活时发 V_TARGET_SENTINEL（远大于任何真实巡航速度）
      # ⇒ min() 永远选不中它 ⇒ 对上面四个来源严格零影响。
      LongitudinalPlanSource.trafficStop: (self.traffic_stop.output_v_target,
                                           self.traffic_stop.output_a_target),
    }

    self.source = min(targets, key=lambda k: targets[k][0])
    self.output_v_target, self.output_a_target = targets[self.source]

    # ── [LongWin] 探针（2026-09-27 加；纯留痕，不参与任何控制判断）───────────
    # 为什么必须有：本行的 `min(targets, key=v_target)` 会把**赢家的 v_target
    # 直接当成新的巡航速度**返回给主 planner（`a_cruise = clip(v_cruise − v_ego)`
    # 随之变负）⇒ 这就是「打直后莫名减速」的来源层。此前判赢家只能拿
    # `[GasOverride]` 里的 vCruise 反推，**没有直接证据**，甚至无法区分
    # 「setpoint 本身被压低」与「某个候选把 v_target 压下来了」。
    # 字段：win= 赢家短名（见 _LW_NAMES）/ vCruise= 进本函数时的巡航设定 /
    #       各候选 `v/a`：v 单位 **km/h**（便于对照车机显示，m/s × 3.6）、
    #       a 单位 m/s²；`inf` = 该来源未激活（发的是 V_TARGET_SENTINEL）。
    # 触发：赢家切换当帧必打 + 否则 1 Hz（两条都要 —— 只看跳变会在
    #       「一直同一个赢家」时完全静默，而那正是需要证据的常态）。
    try:
      self._lw_t += DT_MDL
      if self.source != self._lw_last or self._lw_t >= 1.0:
        self._lw_t = 0.0
        self._lw_last = self.source
        parts = []
        for k in _LW_ORDER:
          v_kph = targets[k][0] * 3.6
          vs = 'inf' if v_kph > 200.0 else f'{v_kph:.1f}'
          parts.append(f"{_LW_NAMES[k]}={vs}/{targets[k][1]:+.2f}")
        cloudlog.info(f"[LongWin] win={_LW_NAMES.get(self.source, '?')} "
                      f"vCruise={v_cruise * 3.6:.0f} | " + " ".join(parts))
    except Exception:
      pass

    return self.output_v_target, self.output_a_target

  def update(self, sm: messaging.SubMaster) -> None:
    self.events_sp.clear()
    self.dec.update(sm)
    self.e2e_alerts_helper.update(sm, self.events_sp)

  def publish_longitudinal_plan_sp(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    plan_sp_send = messaging.new_message('longitudinalPlanSP')

    plan_sp_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlanSP = plan_sp_send.longitudinalPlanSP
    longitudinalPlanSP.longitudinalPlanSource = self.source
    longitudinalPlanSP.vTarget = float(self.output_v_target)
    longitudinalPlanSP.aTarget = float(self.output_a_target)
    longitudinalPlanSP.events = self.events_sp.to_msg()

    # Dynamic Experimental Control
    dec = longitudinalPlanSP.dec
    dec.state = DecState.blended if self.dec.mode() == 'blended' else DecState.acc
    dec.enabled = self.dec.enabled()
    dec.active = self.dec.active()

    # Smart Cruise Control
    smartCruiseControl = longitudinalPlanSP.smartCruiseControl
    # Vision Control
    sccVision = smartCruiseControl.vision
    sccVision.state = self.scc.vision.state
    sccVision.vTarget = float(self.scc.vision.output_v_target)
    sccVision.aTarget = float(self.scc.vision.output_a_target)
    sccVision.currentLateralAccel = float(self.scc.vision.current_lat_acc)
    sccVision.maxPredictedLateralAccel = float(self.scc.vision.max_pred_lat_acc)
    sccVision.enabled = self.scc.vision.is_enabled
    sccVision.active = self.scc.vision.is_active
    # Map Control
    sccMap = smartCruiseControl.map
    sccMap.state = self.scc.map.state
    sccMap.vTarget = float(self.scc.map.output_v_target)
    sccMap.aTarget = float(self.scc.map.output_a_target)
    sccMap.enabled = self.scc.map.is_enabled
    sccMap.active = self.scc.map.is_active

    # Speed Limit
    speedLimit = longitudinalPlanSP.speedLimit
    resolver = speedLimit.resolver
    resolver.speedLimit = float(self.resolver.speed_limit)
    resolver.speedLimitLast = float(self.resolver.speed_limit_last)
    resolver.speedLimitFinal = float(self.resolver.speed_limit_final)
    resolver.speedLimitFinalLast = float(self.resolver.speed_limit_final_last)
    resolver.speedLimitValid = self.resolver.speed_limit_valid
    resolver.speedLimitLastValid = self.resolver.speed_limit_last_valid
    resolver.speedLimitOffset = float(self.resolver.speed_limit_offset)
    resolver.distToSpeedLimit = float(self.resolver.distance)
    resolver.source = self.resolver.source
    assist = speedLimit.assist
    assist.state = self.sla.state
    assist.enabled = self.sla.is_enabled
    assist.active = self.sla.is_active
    assist.vTarget = float(self.sla.output_v_target)
    assist.aTarget = float(self.sla.output_a_target)

    # E2E Alerts
    e2eAlerts = longitudinalPlanSP.e2eAlerts
    e2eAlerts.greenLightAlert = self.e2e_alerts_helper.green_light_alert
    e2eAlerts.leadDepartAlert = self.e2e_alerts_helper.lead_depart_alert

    pm.send('longitudinalPlanSP', plan_sp_send)
