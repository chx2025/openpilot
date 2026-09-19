#!/usr/bin/env python3
import math
import numpy as np
from collections import deque
from typing import Any

import capnp
from openpilot.cereal import messaging, log, custom
from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.common.simple_kalman import KF1D

from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP


# Default lead acceleration decay set to 50% at 1s
_LEAD_ACCEL_TAU = 1.5

# radar tracks
SPEED, ACCEL = 0, 1     # Kalman filter states enum

# stationary qualification parameters
V_EGO_STATIONARY = 4.   # no stationary object flag below this speed

RADAR_TO_CAMERA = 1.52  # RADAR is ~ 1.5m ahead from center of mesh frame

# 转弯判定门槛（对齐 herizon1054/openpilot@dpagel）
# 供 radard_ext 的横向门控使用：方向盘角度或角速度任一超过门槛即视为转弯中。
TURN_STEER_ANGLE_DEG = 15.0
TURN_STEER_RATE_DEG = 10.0

# ---- 静止目标过滤（2026-09-18 改版：雷达不处理静止物体）----
# 背景：雷达对「静止物体」的判定天然不可靠 —— 弯道护栏、路牌、井盖、桥墩、
# 邻车道停着的车，都会被探成几乎静止的 lead。视觉模型对这些目标置信度极低，
# 但雷达报上去的速度/加速度会直接喂进 MPC，就变成幽灵刹车。
#
# 实车实测（C3XL @192.168.20.115，2026-09-18 上午一趟车）：
#   [RadarDSP_GhostDrop] lead1 | dRel 5.8 yRel -2.3 vLeadK -0.1 camProb 0.35
#   [RadarDSP_GhostDrop] lead0 | dRel 2.4 yRel -2.1 vLeadK  0.6 camProb 0.32
#   → dRel 2.4~8.9m（极近）、vLeadK 0~1.2m/s（几乎静止）、camProb 0.22~0.35（视觉不认可）
#   同趟车 FCW triggered 3 次（条件 mpc.crash_cnt>2 且 leadOne.modelProb>0.9），
#   即 MPC 连续 3 帧认定 5s 内要撞 —— 就是体感上的「狠刹一脚」。
#
# 旧版判据要求「方向盘>=60° 或 车速>=70km/h」才启用过滤。实测该工况门太窄：
# 中速（50~65km/h）+ 小转角时不成立，幽灵照旧进 MPC。
#
# 现方案（车主 2026-09-18 决定）：**雷达不处理静止物体** —— 目标只要几乎静止，
# 一律丢弃雷达数据，静止物完全交给视觉 lead（leadsV3）承担。
#   · 不再需要工况门（GHOST_DROP_STEER_ANGLE_DEG / GHOST_DROP_V_EGO_KPH 已删除）
#   · 不再需要 camProb 判据（视觉看不到的静止物正是要丢的那批）
#   · 原厂 potential_low_speed_lead 低速防撞兜底不受影响（v_ego<4m/s 仍采用最近目标）
#
# 丢弃后不会凭空少一个 lead：上层会退回纯视觉 lead（get_RadarState_from_vision），
# 语义是「这次不用雷达测出来的速度和加速度」—— 那才是幽灵刹车的直接来源。
#
# 调参：拥堵跟车发抖（误丢慢速真车）-> 调小 GHOST_DROP_STATIC_KPH（如 3.0）
#       幽灵刹车没消 -> 调大（如 8.0）；彻底关掉把 GHOST_DROP_ENABLE 置 False。
GHOST_DROP_ENABLE = True
GHOST_DROP_STATIC_KPH = 5.0   # 目标绝对速度门槛 (km/h)：低于此值视为「静止物」，一律丢弃


class KalmanParams:
  def __init__(self, dt: float):
    # Lead Kalman Filter params, calculating K from A, C, Q, R requires the control library.
    # hardcoding a lookup table to compute K for values of radar_ts between 0.01s and 0.2s
    assert dt > .01 and dt < .2, "Radar time step must be between .01s and 0.2s"
    self.A = [[1.0, dt], [0.0, 1.0]]
    self.C = [1.0, 0.0]
    #Q = np.matrix([[10., 0.0], [0.0, 100.]])
    #R = 1e3
    #K = np.matrix([[ 0.05705578], [ 0.03073241]])
    dts = [i * 0.01 for i in range(1, 21)]
    K0 = [0.12287673, 0.14556536, 0.16522756, 0.18281627, 0.1988689,  0.21372394,
          0.22761098, 0.24069424, 0.253096,   0.26491023, 0.27621103, 0.28705801,
          0.29750003, 0.30757767, 0.31732515, 0.32677158, 0.33594201, 0.34485814,
          0.35353899, 0.36200124]
    K1 = [0.29666309, 0.29330885, 0.29042818, 0.28787125, 0.28555364, 0.28342219,
          0.28144091, 0.27958406, 0.27783249, 0.27617149, 0.27458948, 0.27307714,
          0.27162685, 0.27023228, 0.26888809, 0.26758976, 0.26633338, 0.26511557,
          0.26393339, 0.26278425]
    self.K = [[np.interp(dt, dts, K0)], [np.interp(dt, dts, K1)]]


class Track:
  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
    self.identifier = identifier
    self.cnt = 0
    self.aLeadTau = FirstOrderFilter(_LEAD_ACCEL_TAU, 0.45, DT_MDL)
    self.K_A = kalman_params.A
    self.K_C = kalman_params.C
    self.K_K = kalman_params.K
    self.kf = KF1D([[v_lead], [0.0]], self.K_A, self.K_C, self.K_K)

  def update(self, d_rel: float, y_rel: float, v_rel: float, v_lead: float, measured: bool = False):
    # relative values, copy
    self.dRel = d_rel   # LONG_DIST
    self.yRel = y_rel   # -LAT_DIST
    self.vRel = v_rel   # REL_SPEED
    self.vLead = v_lead
    # 该雷达点是否为真实量测（而非运动学外推）。
    # 本分支 opendbc 已把 RadarPoint.measured 移入 deprecated group 且无驱动填充它，
    # 实际恒为 False；保留该属性是为了与上游逻辑保持接口一致（见 radard_ext.py）。
    self.measured = measured

    # computed velocity and accelerations
    if self.cnt > 0:
      self.kf.update(self.vLead)

    self.vLeadK = float(self.kf.x[SPEED][0])
    self.aLeadK = float(self.kf.x[ACCEL][0])

    # Learn if constant acceleration
    if abs(self.aLeadK) < 0.5:
      self.aLeadTau.x = _LEAD_ACCEL_TAU
    else:
      self.aLeadTau.update(0.0)

    self.cnt += 1

  def get_RadarState(self, model_prob: float = 0.0):
    return {
      "dRel": float(self.dRel),
      "yRel": float(self.yRel),
      "vRel": float(self.vRel),
      "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK),
      "aLeadK": float(self.aLeadK),
      "aLeadTau": float(self.aLeadTau.x),
      "present": True,
      "modelProb": model_prob,
      "radar": True,
      "radarTrackId": self.identifier,
    }

  def potential_low_speed_lead(self, v_ego: float):
    # stop for stuff in front of you and low speed, even without model confirmation
    # Radar points closer than 0.75, are almost always glitches on toyota radars
    return abs(self.yRel) < 1.0 and (v_ego < V_EGO_STATIONARY) and (0.75 < self.dRel < 25)

  def __str__(self):
    ret = f"x: {self.dRel:4.1f}  y: {self.yRel:4.1f}  v: {self.vRel:4.1f}  a: {self.aLeadK:4.1f}"
    return ret


def is_stationary_track(track: Track) -> bool:
  """静止目标判定：为真即丢弃该雷达轨迹，退回纯视觉 lead（雷达不处理静止物体）。

  track : 已经过 match_vision_to_track 匹配上的雷达轨迹

  为什么不做工况门、也不看视觉置信度：雷达对静止物（护栏/路牌/井盖/邻车道停车）
  的测速本身就不可靠，而「视觉置信度低」恰恰是这类目标的典型特征 —— 拿它当判据
  等于放行。详见文件头说明。

  速度判据用 vLeadK（卡尔曼滤波后的绝对速度）而非单帧 vLead，避免杂波测速抖动
  把静止目标一帧帧地判成运动目标。Kalman 初值直接取 v_lead，所以新轨迹也不会有
  冷启动偏差。
  """
  if not GHOST_DROP_ENABLE:
    return False
  return abs(track.vLeadK) < GHOST_DROP_STATIC_KPH * CV.KPH_TO_MS


def laplacian_pdf(x: float, mu: float, b: float):
  b = max(b, 1e-4)
  return math.exp(-abs(x-mu)/b)


def match_vision_to_track(v_ego: float, lead: capnp._DynamicStructReader, tracks: dict[int, Track]):
  offset_vision_dist = lead.x[0] - RADAR_TO_CAMERA

  def prob(c):
    prob_d = laplacian_pdf(c.dRel, offset_vision_dist, lead.xStd[0])
    prob_y = laplacian_pdf(c.yRel, -lead.y[0], lead.yStd[0])
    prob_v = laplacian_pdf(c.vRel + v_ego, lead.v[0], lead.vStd[0])

    # This isn't exactly right, but it's a good heuristic
    return prob_d * prob_y * prob_v

  track = max(tracks.values(), key=prob)

  # if no 'sane' match is found return -1
  # stationary radar points can be false positives
  dist_sane = abs(track.dRel - offset_vision_dist) < max([(offset_vision_dist)*.25, 5.0])
  vel_sane = (abs(track.vRel + v_ego - lead.v[0]) < 10) or (v_ego + track.vRel > 3)
  if dist_sane and vel_sane:
    return track
  else:
    return None


def get_RadarState_from_vision(lead_msg: capnp._DynamicStructReader, v_ego: float, model_v_ego: float, lead_prob: float):
  lead_v_rel_pred = lead_msg.v[0] - model_v_ego
  return {
    "dRel": float(lead_msg.x[0] - RADAR_TO_CAMERA),
    "yRel": float(-lead_msg.y[0]),
    "vRel": float(lead_v_rel_pred),
    "vLead": float(v_ego + lead_v_rel_pred),
    "vLeadK": float(v_ego + lead_v_rel_pred),
    "aLeadK": float(lead_msg.a[0]),
    "aLeadTau": 0.3,
    "modelProb": float(lead_prob),
    "present": True,
    "radar": False,
    "radarTrackId": -1,
  }


def get_lead(v_ego: float, ready: bool, tracks: dict[int, Track], lead_msg: capnp._DynamicStructReader,
             model_v_ego: float, lead_prob: float, CP: structs.CarParams, CP_SP: structs.CarParamsSP,
             is_turning: bool = False, low_speed_override: bool = True) -> dict[str, Any]:
  # Determine leads, this is where the essential logic happens
  if len(tracks) > 0 and ready and lead_prob > .5:
    track = match_vision_to_track(v_ego, lead_msg, tracks)
    # 静止目标一律丢弃（雷达不处理静止物体），后续退回纯视觉 lead。
    # 详见文件头 GHOST_* 说明与 is_stationary_track。
    if track is not None and is_stationary_track(track):
      track = None
  else:
    track = None

  lead_dict = {'present': False}
  if track is not None:
    lead_dict = track.get_RadarState(lead_prob)
    lead_dict = get_custom_yrel(CP, CP_SP, lead_dict, lead_msg)
  elif (track is None) and ready and (lead_prob > .5):
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego, lead_prob)

  if low_speed_override:
    low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
    if len(low_speed_tracks) > 0:
      closest_track = min(low_speed_tracks, key=lambda c: c.dRel)

      # Only choose new track if it is actually closer than the previous one
      if (not lead_dict['present']) or (closest_track.dRel < lead_dict['dRel']):
        lead_dict = closest_track.get_RadarState()

  return lead_dict


def get_custom_yrel(CP: structs.CarParams, CP_SP: structs.CarParamsSP, lead_dict: dict[str, Any],
                    lead_msg: capnp._DynamicStructReader) -> dict[str, Any]:
  if CP.brand == "hyundai" and (CP_SP.flags & HyundaiFlagsSP.ENHANCED_SCC or
                                CP.flags & (HyundaiFlags.CANFD_CAMERA_SCC | HyundaiFlags.CAMERA_SCC)):
    lead_dict['yRel'] = float(-lead_msg.y[0])

  return lead_dict


class RadarD:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParams, delay: float = 0.0):
    self.CP = CP
    self.CP_SP = CP_SP

    self.current_time = 0.0

    self.tracks: dict[int, Track] = {}
    self.kalman_params = KalmanParams(DT_MDL)
    self.lead_prob_filters = [FirstOrderFilter(0.0, 0.2, DT_MDL) for _ in range(2)]

    self.v_ego = 0.0
    self.v_ego_hist = deque([0.0], maxlen=int(round(delay / DT_MDL))+1)
    self.last_v_ego_frame = -1

    self.radar_state: capnp._DynamicStructBuilder | None = None
    self.radar_state_valid = False

    self.ready = False

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    self.ready = sm.seen['modelV2']
    self.current_time = 1e-9*max(sm.logMonoTime.values())

    if sm.recv_frame['carState'] != self.last_v_ego_frame:
      self.v_ego = sm['carState'].vEgo
      self.v_ego_hist.append(self.v_ego)
      self.last_v_ego_frame = sm.recv_frame['carState']

    # dp: 判断是否正在转弯（方向盘角度或角速度超过门槛），供 radard_ext 的信心度累积
    # 逻辑使用——转弯中保留「必须真实量测」的横向保护（避免旁侧车道目标因外推值
    # 被误判成切入本车道），直行/巡航时放行（避免正常雷达漏拍拖慢插队目标的反应）。
    is_turning = abs(sm['carState'].steeringAngleDeg) >= TURN_STEER_ANGLE_DEG or abs(sm['carState'].steeringRateDeg) >= TURN_STEER_RATE_DEG

    # 静止目标过滤不再需要工况门：目标只要几乎静止就一律丢弃雷达数据，
    # 由视觉 lead 承担（见文件头 GHOST_* 说明与 is_stationary_track）。
    ar_pts = {pt.trackId: [pt.dRel, pt.yRel, pt.vRel, pt.deprecated.measured] for pt in rr.points}

    # *** remove missing points from meta data ***
    for ids in list(self.tracks.keys()):
      if ids not in ar_pts:
        self.tracks.pop(ids, None)

    # *** compute the tracks ***
    for ids in ar_pts:
      rpt = ar_pts[ids]

      # align v_ego by a fixed time to align it with the radar measurement
      v_lead = rpt[2] + self.v_ego_hist[0]

      # create the track if it doesn't exist or it's a new track
      if ids not in self.tracks:
        self.tracks[ids] = Track(ids, v_lead, self.kalman_params)
      self.tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead, rpt[3])

    # *** publish radarState ***
    self.radar_state_valid = sm.all_checks()
    self.radar_state = log.RadarState.new_message()
    self.radar_state.mdMonoTime = sm.logMonoTime['modelV2']
    self.radar_state.radarErrors = rr.errors

    if len(sm['modelV2'].velocity.x):
      model_v_ego = sm['modelV2'].velocity.x[0]
    else:
      model_v_ego = self.v_ego
    leads_v3 = sm['modelV2'].leadsV3
    if len(leads_v3) > 1:
      for i in range(2):
        # Asymmetric filter on lead prob to keep lead when uncertain
        lead_prob = leads_v3[i].prob
        if lead_prob > self.lead_prob_filters[i].x:
          self.lead_prob_filters[i].x = lead_prob
        else:
          self.lead_prob_filters[i].update(lead_prob)

      self.radar_state.leadOne = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[0], model_v_ego, self.lead_prob_filters[0].x,
                                          self.CP, self.CP_SP, is_turning=is_turning, low_speed_override=True)
      self.radar_state.leadTwo = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[1], model_v_ego, self.lead_prob_filters[1].x,
                                          self.CP, self.CP_SP, is_turning=is_turning, low_speed_override=False)

  def publish(self, pm: messaging.PubMaster):
    assert self.radar_state is not None

    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = self.radar_state_valid
    radar_msg.radarState = self.radar_state
    pm.send("radarState", radar_msg)


# fuses camera and radar data for best lead detection
def main() -> None:
  config_realtime_process(5, Priority.CTRL_LOW)

  # wait for stats about the car to come in from controls
  cloudlog.info("radard is waiting for CarParams")
  CP = messaging.log_from_bytes(Params().get("CarParams", block=True), car.CarParams)
  cloudlog.info("radard got CarParams")

  cloudlog.info("radard is waiting for CarParamsSP")
  CP_SP = messaging.log_from_bytes(Params().get("CarParamsSP", block=True), custom.CarParamsSP)
  cloudlog.info("radard got CarParamsSP")

  # *** setup messaging
  sm = messaging.SubMaster(['modelV2', 'carState', 'radarTracks'], poll='modelV2')
  pm = messaging.PubMaster(['radarState'])

  from openpilot.sunnypilot.selfdrive.controls.radard_ext import RadarDSP as RadarD
  RD = RadarD(CP, CP_SP, CP.radarDelay)

  while 1:
    sm.update()

    RD.update(sm, sm['radarTracks'])
    RD.publish(pm)


if __name__ == "__main__":
  main()
