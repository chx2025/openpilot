#!/usr/bin/env python3
import math
import numpy as np
from collections import deque
from typing import Any

# 新增：用於非同步寫入與時間紀錄的模組
import threading
import queue
from datetime import datetime
import os

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


# ==========================================
# 新增：非同步日誌寫入器 (包含 50MB 自動輪替保護機制)
# ==========================================
class AsyncLogger:
  def __init__(self, filepath="/data/radar_target_log.txt", max_size_mb=50):
    self.filepath = filepath
    self.max_size_bytes = max_size_mb * 1024 * 1024
    self.q = queue.Queue(maxsize=1000)
    self.running = True
    self.thread = threading.Thread(target=self._writer_thread)
    self.thread.daemon = True
    self.thread.start()

  def log(self, msg: str):
    if not self.q.full():
      self.q.put_nowait(msg)

  def _writer_thread(self):
    # 啟動時檢查大小
    if os.path.exists(self.filepath) and os.path.getsize(self.filepath) > self.max_size_bytes:
      open(self.filepath, 'w').close()

    with open(self.filepath, 'a', encoding='utf-8') as f:
      f.write(f"\n--- RADAR LOG STARTED AT {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ---\n")
      while self.running:
        try:
          msg = self.q.get(timeout=0.5)
          
          # 寫入前檢查檔案大小，超過限制則清空輪替
          if os.path.exists(self.filepath) and os.path.getsize(self.filepath) > self.max_size_bytes:
            f.close()
            open(self.filepath, 'w').close()
            f = open(self.filepath, 'a', encoding='utf-8')
            f.write(f"\n--- RADAR LOG ROTATED AT {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ---\n")

          f.write(msg + '\n')
          f.flush()
        except queue.Empty:
          pass
# ==========================================


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

    # 用於記錄旁車切入狀態的專屬變數
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
    
    if md is not None and v_cruise >= 90.0 and 0.0 < self.dRel <= 80.0:
      if len(md.laneLines) >= 3 and len(md.laneLineProbs) >= 3:
        if md.laneLineProbs[1] > 0.3 and md.laneLineProbs[2] > 0.3:
          lane_x = list(md.laneLines[1].x)
          if len(lane_x) > 0:
            left_y = np.interp(self.dRel, lane_x, list(md.laneLines[1].y))
            right_y = np.interp(self.dRel, lane_x, list(md.laneLines[2].y))
            
            speed_condition = (self.vRel > -3.0)
            
            left_cut_in = (self.yRel > 0) and \
                          ((self.yRel - 1.0) < (left_y - 1.0)) and \
                          (self.y_vel < -0.1) and speed_condition
                          
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

    self.is_cutting_in = self.cut_in_count >= 20
    # ==========================================

  def get_RadarState(self, model_prob: float = 0.0):
    output_yRel = 0.0 if self.is_cutting_in else float(self.yRel)

    return {
      "dRel": float(self.dRel),
      "yRel": output_yRel,
      "vRel": float(self.vRel),
      "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK),
      "aLeadK": float(self.aLeadK),
      "aLeadTau": float(self.aLeadTau.x),
      "status": True,
      "fcw": self.is_potential_fcw(model_prob),
      "modelProb": float(model_prob),
      "radar": True,
      "radarTrackId": self.identifier,
      
      # 傳出內部信心度積分供 Log 紀錄
      "stoppedConf": self.is_stopped_car_count,
      "cutInConf": self.cut_in_count,
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
  is_valid_lead = (dist_sane and vel_sane) or (dist_sane and y_sane_on_path and lead.prob > 0.3)

  # 先更新所有軌跡的信心度分數 (Leak Bucket)
  for c in tracks.values():
    if c is track and is_valid_lead:
      c.is_stopped_car_count = min(c.is_stopped_car_count + 5, 25)
    else:
      c.is_stopped_car_count = max(0, c.is_stopped_car_count - 1)

  best_track = None

  if dist_sane and vel_sane:
    best_track = track
  elif track.is_stopped_car_count >= 20:
    best_track = track

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
    
    # 純視覺無雷達積分，設為 0
    "stoppedConf": 0,
    "cutInConf": 0,
  }


def get_lead(v_ego: float, ready: bool, tracks: dict[int, Track], lead_msg: capnp._DynamicStructReader,
             model_v_ego: float, path_x: list[float], path_y: list[float], low_speed_override: bool = True) -> dict[str, Any]:
  
  if len(tracks) > 0 and ready and lead_msg.prob > .5:
    track = match_vision_to_track(v_ego, lead_msg, tracks, path_x, path_y)
  else:
    track = None

  lead_dict = {'status': False}
  category = "None"

  # 1. 常規視覺與雷達融合目標
  if track is not None:
    lead_dict = track.get_RadarState(lead_msg.prob)
    category = "[靜止車鎖定]" if track.is_stopped_car_count >= 20 else "[動態追蹤]"
  
  # 2. 純視覺目標
  elif (track is None) and ready and (lead_msg.prob > .5):
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego)
    category = "[純視覺估算]"

  # 3. 高速切入車強行覆蓋目標
  cut_in_tracks = [c for c in tracks.values() if c.is_cutting_in]
  if len(cut_in_tracks) > 0:
    closest_cut_in = min(cut_in_tracks, key=lambda c: c.dRel)
    if (not lead_dict['status']) or (closest_cut_in.dRel < lead_dict.get('dRel', 1000.0)):
      lead_dict = closest_cut_in.get_RadarState(model_prob=0.99)
      category = "[旁車切入防禦]"

  # 4. 低速近距離盲煞防撞
  if low_speed_override:
    low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
    if len(low_speed_tracks) > 0:
      closest_track = min(low_speed_tracks, key=lambda c: c.dRel)
      if (not lead_dict['status']) or (closest_track.dRel < lead_dict.get('dRel', 1000.0)):
        lead_dict = closest_track.get_RadarState()
        category = "[低速盲區煞停]"

  if lead_dict.get('status', False):
    lead_dict['category'] = category

  return lead_dict


class RadarD:
  def __init__(self, delay: float = 0.0):
    self.current_time = 0.0

    self.tracks: dict[int, Track] = {}
    self.kalman_params = KalmanParams(DT_MDL)

    # 初始化非同步 Logger
    self.logger = AsyncLogger("/data/radar_target_log.txt")

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
      v_lead = rpt[2] + self.v_ego_hist[0]

      if ids not in self.tracks:
        self.tracks[ids] = Track(ids, v_lead, self.kalman_params)
      
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
      path_x = [0.0, 100.0]
      path_y = [0.0, 0.0]

    if len(sm['modelV2'].velocity.x):
      model_v_ego = sm['modelV2'].velocity.x[0]
    else:
      model_v_ego = self.v_ego
      
    leads_v3 = sm['modelV2'].leadsV3
    if len(leads_v3) > 1:
      lead1 = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[0], model_v_ego, path_x, path_y, low_speed_override=True)
      lead2 = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[1], model_v_ego, path_x, path_y, low_speed_override=False)
      
      # 🚨 關鍵修正：剔除 Cap'n Proto 模具不認識的自訂欄位，防止當機
      custom_keys = {'category', 'stoppedConf', 'cutInConf'}
      self.radar_state.leadOne = {k: v for k, v in lead1.items() if k not in custom_keys}
      self.radar_state.leadTwo = {k: v for k, v in lead2.items() if k not in custom_keys}

      # --- 將有效目標傳送至非同步 Logger ---
      now_str = datetime.now().strftime('%H:%M:%S.%f')[:-3]
      
      if getattr(lead1, 'status', lead1.get('status', False)):
        cat = lead1.get('category', '[未知]')
        tid = lead1.get('radarTrackId', -1)
        d = lead1.get('dRel', 0.0)
        y = lead1.get('yRel', 0.0)
        v = lead1.get('vRel', 0.0)
        
        m_prob = lead1.get('modelProb', 0.0)
        s_conf = lead1.get('stoppedConf', 0)
        c_conf = lead1.get('cutInConf', 0)

        # 過濾左前與右前的靜止車 (橫向距離 > 1.2m 且為靜止車鎖定)
        is_side_stationary = (cat == "[靜止車鎖定]") and (abs(y) > 1.2)

        if not is_side_stationary:
          log_msg = f"{now_str} | LEAD 1 | 類別: {cat:<8} | 追蹤ID: {tid:>3} | 縱距: {d:>5.1f}m | 橫距: {y:>5.1f}m | 相速: {v:>5.1f}m/s | 視覺: {m_prob:.2f} | 靜止積分: {s_conf:>2}/25 | 切入積分: {c_conf:>2}/25"
          self.logger.log(log_msg)

  def publish(self, pm: messaging.PubMaster):
    assert self.radar_state is not None

    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = self.radar_state_valid
    radar_msg.radarState = self.radar_state
    pm.send("radarState", radar_msg)


def main() -> None:
  config_realtime_process(5, Priority.CTRL_LOW)

  cloudlog.info("radard is waiting for CarParams")
  CP = messaging.log_from_bytes(Params().get("CarParams", block=True), car.CarParams)
  cloudlog.info("radard got CarParams")

  sm = messaging.SubMaster(['modelV2', 'carState', 'liveTracks'], poll='modelV2')
  pm = messaging.PubMaster(['radarState'])

  RD = RadarD(CP.radarDelay)

  while 1:
    sm.update()

    RD.update(sm, sm['liveTracks'])
    RD.publish(pm)


if __name__ == "__main__":
  main()
