#!/usr/bin/env python3
import math
import numpy as np
from collections import deque
from typing import Any

import capnp
from cereal import messaging, log, car
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.common.simple_kalman import KF1D


# Default lead acceleration decay set to 50% at 1s
_LEAD_ACCEL_TAU = 1.5

# radar tracks
SPEED, ACCEL = 0, 1     # Kalman filter states enum

# stationary qualification parameters
V_EGO_STATIONARY = 4.   # no stationary object flag below this speed

RADAR_TO_CENTER = 2.7   # (deprecated) RADAR is ~ 2.7m ahead from center of car
RADAR_TO_CAMERA = 1.52  # RADAR is ~ 1.5m ahead from center of mesh frame


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
    
    # --- 前車信心度分數 (抗抖動漏桶機制 - 原 radard5 邏輯) ---
    self.is_stopped_car_count = 0
    self.selected_count = 0

    # 🚨 新增：用於記錄旁車切入狀態的專屬變數
    self.y_prev = None          
    self.y_vel = 0.0            
    self.cut_in_count = 0       
    self.is_cutting_in = False  

  def update(self, d_rel: float, y_rel: float, v_rel: float, v_lead: float, measured: float, md: Any = None, v_cruise: float = 0.0):
    # relative values, copy
    self.dRel = d_rel   # LONG_DIST
    self.yRel = y_rel   # -LAT_DIST
    self.vRel = v_rel   # REL_SPEED
    self.vLead = v_lead
    self.measured = measured   # measured or estimate

    # 計算目標橫向移動速度 (y_vel)
    if self.y_prev is not None:
      self.y_vel = (self.yRel - self.y_prev) / DT_MDL
    self.y_prev = self.yRel

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

    # ==========================================
    # 高速旁車切入 (Cut-in) 獨立判斷邏輯
    # ==========================================
    is_cut_in_condition = False
    
    # 條件 1 & 2: 儀表板定速 >= 90 km/h 且 目標距離在 80 米內
    if md is not None and v_cruise >= 90.0 and 0.0 < self.dRel <= 80.0:
      if len(md.laneLines) >= 3 and len(md.laneLineProbs) >= 3:
        # 條件 3: 左、右車道線辨識信心度必須 > 0.3
        if md.laneLineProbs[1] > 0.3 and md.laneLineProbs[2] > 0.3:
          lane_x = list(md.laneLines[1].x)
          if len(lane_x) > 0:
            left_y = np.interp(self.dRel, lane_x, list(md.laneLines[1].y))
            right_y = np.interp(self.dRel, lane_x, list(md.laneLines[2].y))
            
            # 條件 6: 縱向相對速度條件 (對方比你快，或頂多慢 10.8 km/h 內)
            speed_condition = (self.vRel > -3.0)
            
            # 條件 4 & 5 (左側切入): 右邊緣越線 1m 且正在往右逼近
            left_cut_in = (self.yRel > 0) and \
                          ((self.yRel - 1.0) < (left_y - 1.0)) and \
                          (self.y_vel < -0.1) and speed_condition
                          
            # 條件 4 & 5 (右側切入): 左邊緣越線 1m 且正在往左逼近
            right_cut_in = (self.yRel < 0) and \
                           ((self.yRel + 1.0) > (right_y + 1.0)) and \
                           (self.y_vel > 0.1) and speed_condition
            
            if left_cut_in or right_cut_in:
              is_cut_in_condition = True

    # 防雜訊積分累積 (檢測到 +5，未檢測到 -1)
    if is_cut_in_condition:
      self.cut_in_count = min(25, self.cut_in_count + 5)
    else:
      self.cut_in_count = max(0, self.cut_in_count - 1)

    # 積分到達 20 分正式判定為切入車
    self.is_cutting_in = self.cut_in_count >= 20
    # ==========================================

  def get_RadarState(self, model_prob: float = 0.0):
    # 🚨 新增：欺騙系統邏輯。如果是切入目標，橫向參數歸 0 僅觸發煞車，防止方向盤閃躲。
    output_yRel = 0.0 if self.is_cutting_in else float(self.yRel)

    return {
      "dRel": float(self.dRel),
      "yRel": output_yRel,  # 使用修改過的橫向位置
      "vRel": float(self.vRel),
      "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK),
      "aLeadK": float(self.aLeadK),
      "aLeadTau": float(self.aLeadTau.x),
      "status": True,
      "fcw": self.is_potential_fcw(model_prob),
      "modelProb": model_prob,
      "radar": True,
      "radarTrackId": self.identifier,
    }

  def potential_low_speed_lead(self, v_ego: float):
    # 原版低速盲煞防撞：只看正前方 1.0m 內，不使用預測軌跡防止低速抖動誤判
    return abs(self.yRel) < 1.0 and (v_ego < V_EGO_STATIONARY) and (0.75 < self.dRel < 25)

  def is_potential_fcw(self, model_prob: float):
    return model_prob > .9

  def __str__(self):
    ret = f"x: {self.dRel:4.1f}  y: {self.yRel:4.1f}  v: {self.vRel:4.1f}  a: {self.aLeadK:4.1f}"
    return ret


def laplacian_pdf(x: float, mu: float, b: float):
  b = max(b, 1e-4)
  return math.exp(-abs(x-mu)/b)


def match_vision_to_track(v_ego: float, lead: capnp._DynamicStructReader, tracks: dict[int, Track], path_x: list[float], path_y: list[float]):
  offset_vision_dist = lead.x[0] - RADAR_TO_CAMERA

  def prob(c):
    prob_d = laplacian_pdf(c.dRel, offset_vision_dist, lead.xStd[0])
    # 內部匹配仍然使用真實的 c.yRel，保證追蹤精準度不受 output_yRel 的影響
    prob_y = laplacian_pdf(c.yRel, -lead.y[0], lead.yStd[0])
    prob_v = laplacian_pdf(c.vRel + v_ego, lead.v[0], lead.vStd[0])

    return prob_d * prob_y * prob_v

  track = max(tracks.values(), key=prob)

  # 1. 基礎合理性檢查 (原版邏輯 - 動態車)
  dist_sane = abs(track.dRel - offset_vision_dist) < max([(offset_vision_dist)*.25, 5.0])
  vel_sane = (abs(track.vRel + v_ego - lead.v[0]) < 10) or (v_ego + track.vRel > 3)
  
  # 2. 彎道/軌跡靜止車強化邏輯
  model_x = track.dRel + RADAR_TO_CAMERA
  expected_yRel = -np.interp(model_x, path_x, path_y)
  y_sane_on_path = abs(track.yRel - expected_yRel) < 1.2
  
  # === 核心修改：分離「計分」與「選定」，打造完美抗抖動煞車 ===
  
  # 判斷這幀是否為有效前車 (動態車 OR 軌跡上的靜止車)
  is_valid_lead = (dist_sane and vel_sane) or (dist_sane and y_sane_on_path and lead.prob > 0.3)

  # 先更新所有軌跡的信心度分數 (Leak Bucket)
  for c in tracks.values():
    if c is track and is_valid_lead:
      # 看見目標，加分 (上限 25，賦予約 0.3 秒的滯後保持期)
      c.is_stopped_car_count = min(c.is_stopped_car_count + 5, 25)
    else:
      # 沒看見或軌跡飄走，扣分衰減 (最低 0)
      c.is_stopped_car_count = max(0, c.is_stopped_car_count - 1)

  best_track = None

  # 決定要鎖定輸出的目標
  if dist_sane and vel_sane:
    # 常規動態車：最優先，直接鎖定
    best_track = track
  elif track.is_stopped_car_count >= 20:
    # 靜止車：只要分數大於等於門檻 20 就強制鎖定！
    # (完美解決低速煞停頓挫：即使軌跡短暫抖走，分數依然 >= 20，死死咬住煞車)
    best_track = track

  # 更新選中狀態
  for c in tracks.values():
    if best_track is not None and c is best_track:
      c.selected_count += 1
    else:
      c.selected_count = 0

  return best_track


def get_RadarState_from_vision(lead_msg: capnp._DynamicStructReader, v_ego: float, model_v_ego: float):
  lead_v_rel_pred = lead_msg.v[0] - model_v_ego
  return {
    "dRel": float(lead_msg.x[0] - RADAR_TO_CAMERA),
    "yRel": float(-lead_msg.y[0]),
    "vRel": float(lead_v_rel_pred),
    "vLead": float(v_ego + lead_v_rel_pred),
    "vLeadK": float(v_ego + lead_v_rel_pred),
    "aLeadK": float(lead_msg.a[0]),
    "aLeadTau": 0.3,
    "fcw": False,
    "modelProb": float(lead_msg.prob),
    "status": True,
    "radar": False,
    "radarTrackId": -1,
  }


def get_lead(v_ego: float, ready: bool, tracks: dict[int, Track], lead_msg: capnp._DynamicStructReader,
             model_v_ego: float, path_x: list[float], path_y: list[float], low_speed_override: bool = True) -> dict[str, Any]:
  # Determine leads, this is where the essential logic happens
  if len(tracks) > 0 and ready and lead_msg.prob > .5:
    track = match_vision_to_track(v_ego, lead_msg, tracks, path_x, path_y)
  else:
    track = None

  lead_dict = {'status': False}
  if track is not None:
    lead_dict = track.get_RadarState(lead_msg.prob)
  elif (track is None) and ready and (lead_msg.prob > .5):
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego)

  # ==========================================
  # 高速切入車強行覆蓋目標 (Cut-in Override)
  # ==========================================
  cut_in_tracks = [c for c in tracks.values() if c.is_cutting_in]
  if len(cut_in_tracks) > 0:
    closest_cut_in = min(cut_in_tracks, key=lambda c: c.dRel)
    # 條件 8: 只有在目前沒前車，或切入車比前車更近時，才強行切換鎖定
    if (not lead_dict['status']) or (closest_cut_in.dRel < lead_dict.get('dRel', 1000.0)):
      lead_dict = closest_cut_in.get_RadarState(model_prob=0.99)
  # ==========================================

  if low_speed_override:
    # 這裡保留原版，不傳入 path_x, path_y，專注正前方防撞
    low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
    if len(low_speed_tracks) > 0:
      closest_track = min(low_speed_tracks, key=lambda c: c.dRel)

      # Only choose new track if it is actually closer than the previous one
      if (not lead_dict['status']) or (closest_track.dRel < lead_dict['dRel']):
        lead_dict = closest_track.get_RadarState()

  return lead_dict


class RadarD:
  def __init__(self, delay: float = 0.0):
    self.current_time = 0.0

    self.tracks: dict[int, Track] = {}
    self.kalman_params = KalmanParams(DT_MDL)

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

    # 獲取儀表板定速 (km/h) 與模型資料
    v_cruise_kph = sm['carState'].cruiseState.speed * 3.6 if sm.seen['carState'] else 0.0
    md = sm['modelV2'] if sm.seen['modelV2'] else None

    ar_pts = {pt.trackId: [pt.dRel, pt.yRel, pt.vRel, pt.measured] for pt in rr.points}

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
      # 將 md 與定速一併傳入 update
      self.tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead, rpt[3], md, v_cruise_kph)

    # *** publish radarState ***
    self.radar_state_valid = sm.all_checks()
    self.radar_state = log.RadarState.new_message()
    self.radar_state.mdMonoTime = sm.logMonoTime['modelV2']
    self.radar_state.radarErrors = rr.errors
    self.radar_state.carStateMonoTime = sm.logMonoTime['carState']

    # --- 擷取模型預測的行駛軌跡 ---
    if len(sm['modelV2'].position.x) > 0:
      path_x = list(sm['modelV2'].position.x)
      path_y = list(sm['modelV2'].position.y)
    else:
      # 如果剛開機模型還沒準備好，預設為直走
      path_x = [0.0, 100.0]
      path_y = [0.0, 0.0]

    if len(sm['modelV2'].velocity.x):
      model_v_ego = sm['modelV2'].velocity.x[0]
    else:
      model_v_ego = self.v_ego
      
    leads_v3 = sm['modelV2'].leadsV3
    if len(leads_v3) > 1:
      self.radar_state.leadOne = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[0], model_v_ego, path_x, path_y, low_speed_override=True)
      self.radar_state.leadTwo = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[1], model_v_ego, path_x, path_y, low_speed_override=False)

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

  # *** setup messaging
  sm = messaging.SubMaster(['modelV2', 'carState', 'liveTracks'], poll='modelV2')
  pm = messaging.PubMaster(['radarState'])

  RD = RadarD(CP.radarDelay)

  while 1:
    sm.update()

    RD.update(sm, sm['liveTracks'])
    RD.publish(pm)


if __name__ == "__main__":
  main()
