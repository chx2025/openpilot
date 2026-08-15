#!/usr/bin/env python3
import capnp
import numpy as np
from typing import Any
from cereal import messaging, car

# ==============================================================================
# 1. 引入整個 radard 模組進行 Monkey Patch (動態替換)
# ==============================================================================
from openpilot.selfdrive.controls import radard

# 2. 正常引入我們需要的元件與原始函數 (移除 DP 中不存在的 structs 與客製函數)
from openpilot.selfdrive.controls.radard import (
    KalmanParams, Track, RadarD, match_vision_to_track,
    get_RadarState_from_vision, RADAR_TO_CAMERA
)

# 3. 引入 cloudlog 用於記錄我們自訂的提早鎖定事件
from openpilot.common.swaglog import cloudlog

# ==============================================================================
# 提早鎖定 (Early Lock) 擴充模組參數設定
# ==============================================================================
LANE_WIDTH_FALLBACK = 1.5           # 預測車道基準單側半寬 (m)
LANE_HYSTERESIS_MARGIN = 0.5        # 邊界外的遲滯容錯預度 (m)
FUZZY_BOUNDS = [0.5, 1.5]           # 物理誤差 (m 或 m/s): 0.5 以內給滿分 1.0，大於 1.5 總分歸零

ALPHA_BASE = 0.2                    # 常規上升學習率
ALPHA_DOWN = 0.1                    # 常規下降與短路過濾時的衰減學習率

BRAKE_THRES_RANGE = [-3.0, -1.2]    # 急煞觸發區間 (m/s²)
MULT_RANGE = [1.2, 1.0]             # 對應威脅倍率
CUTIN_DIST_LIMIT = 40.0             # 評估切入威脅的最大縱向有效距離 (m)
DYNAMIC_SPEED_PCT = 0.2             # 動態相對速度閥值比例

CAM_PROB_SPEED_RANGE = [10.0, 25.0] # 動態相機門檻車速區間
CAM_PROB_RANGE = [0.5, 0.3]         # 動態相機審查門檻
STATIC_EMA_CAP = 0.6                # 目標未達審查門檻時的 EMA 天花板

EMA_VAL_RANGE = [0.4, 0.8]          # 本地 EMA 信心度 X 軸
PROB_THRES_RANGE = [0.5, 0.3]       # 映射出對應的「視覺提早放行門檻」 Y 軸

RELEASE_FRAMES = 5                  # 目標短暫丟失或出界時的 EMA 續命凍結幀數
SELECT_HOLDOVER_FRAMES = 3          # 雷達硬體斷流時，強制維持上一幀鎖定的幀數

MODEL_TAU_MIN_PROB = 0.5            # 啟動驗證的最低視覺機率
MODEL_TAU_BRAKE_A = -0.5            # 啟動驗證的最低急煞門檻 (m/s²)
MODEL_TAU_SUSTAINED = 0.5           # 視覺確認急煞持續
MODEL_TAU_SPURIOUS = 3.0            # 視覺預測即將加速

# 全域快取：僅儲存上一幀鎖定目標的「識別碼(trackId)」，不快取物件本身。
# 注意：絕對不可快取 Track 物件參考！一旦該 trackId 被 RadarD.update() 從
# self.tracks 中移除（雷達硬體真的失去該點），舊物件就不會再被 .update()
# 更新，dRel/vRel/aLeadK 會被凍結在消失前的最後一刻。若之後續命邏輯繼續
# 回傳這顆「冰封」的殭屍物件長達 SELECT_HOLDOVER_FRAMES 幀，等於餵給縱向控制
# 一組過期的相對速度/加速度，正是先前 Prius C 上 ACC 高速來回加減速（振盪）
# 的根因。修正後每一幀都用 trackId 向當下的 tracks dict 重新查詢，確保拿到
# 的永遠是「這一幀」雷達實際更新過的即時物件；只有當 trackId 真的從
# tracks 消失（雷達硬體真正斷流）才允許續命倒數，續命期間也是每幀重新取值。
_LEAD_STATE_CACHE = {
    0: {'track_id': None, 'absent': 0},
    1: {'track_id': None, 'absent': 0}
}


def get_model_lead_tau(lead_msg, lead_prob: float) -> float | None:
  if lead_prob < MODEL_TAU_MIN_PROB or len(lead_msg.a) < 2:
    return None
  
  a0 = float(lead_msg.a[0])
  a1 = float(lead_msg.a[1])
  
  if a0 > MODEL_TAU_BRAKE_A:
    return None
  if a1 < 0.5 * a0:
    return MODEL_TAU_SUSTAINED
  if a1 > 0.1 * a0:
    return MODEL_TAU_SPURIOUS
  
  return None


class TrackDP(Track):
  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
    super().__init__(identifier, v_lead, kalman_params)
    self.ema_confidence = {0: 0.4, 1: 0.4}
    self.holdover_frames = {0: 0, 1: 0}
    self.is_out_of_lane = False

  def _check_spatial_boundaries(self, vision_y: float) -> bool:
    left_bound = vision_y + LANE_WIDTH_FALLBACK
    right_bound = vision_y - LANE_WIDTH_FALLBACK
    current_y = self.yRel

    if not self.is_out_of_lane:
      if current_y > (left_bound + LANE_HYSTERESIS_MARGIN) or current_y < (right_bound - LANE_HYSTERESIS_MARGIN):
        self.is_out_of_lane = True
    else:
      if right_bound <= current_y <= left_bound:
        self.is_out_of_lane = False

    return not self.is_out_of_lane

  def _calculate_fuzzy_score(self, offset_vision_dist: float, vision_y: float, vision_v: float, v_ego: float) -> float:
    err_d = abs(self.dRel - offset_vision_dist)
    err_y = abs(self.yRel - vision_y)
    err_v = abs((self.vRel + v_ego) - vision_v)

    score_d = float(np.interp(err_d, FUZZY_BOUNDS, [1.0, 0.0]))
    score_y = float(np.interp(err_y, FUZZY_BOUNDS, [1.0, 0.0]))
    score_v = float(np.interp(err_v, FUZZY_BOUNDS, [1.0, 0.0]))

    return score_d * score_y * score_v

  def _calculate_threat_multipliers(self, v_ego: float) -> float:
    brake_mult = float(np.interp(self.aLeadK, BRAKE_THRES_RANGE, MULT_RANGE))
    cutin_mult = 1.0
    
    if self.dRel < CUTIN_DIST_LIMIT and abs(self.yRel) > 1.0:
      v_limit = max(1.0, DYNAMIC_SPEED_PCT * v_ego)
      cutin_mult = float(np.interp(self.vRel, [-v_limit, v_limit], MULT_RANGE))

    final_alpha = ALPHA_BASE * brake_mult * cutin_mult
    return min(1.0, final_alpha)

  def _apply_slow_protection(self, v_ego: float, cam_prob: float, current_ema: float) -> float:
    abs_v_lead = abs(self.vRel + v_ego)
    dynamic_v_limit = max(1.0, DYNAMIC_SPEED_PCT * v_ego)

    if abs_v_lead < dynamic_v_limit:
      dynamic_cam_prob_thres = float(np.interp(v_ego, CAM_PROB_SPEED_RANGE, CAM_PROB_RANGE))
      if cam_prob < dynamic_cam_prob_thres:
        return min(current_ema, STATIC_EMA_CAP)

    return current_ema

  def process_track_logic(self, lead_idx: int, lead_msg: capnp._DynamicStructReader, v_ego: float, lead_prob: float):
    offset_vision_dist = lead_msg.x[0] - RADAR_TO_CAMERA
    vision_y = -lead_msg.y[0]
    vision_v = lead_msg.v[0]

    is_invalid = not self.measured or abs(self.yRel - vision_y) > (LANE_WIDTH_FALLBACK + LANE_HYSTERESIS_MARGIN)
    
    fuzzy_score = 0.0
    if not is_invalid:
      is_valid_spatial = self._check_spatial_boundaries(vision_y)
      fuzzy_score = self._calculate_fuzzy_score(offset_vision_dist, vision_y, vision_v, v_ego)
      is_invalid = not is_valid_spatial or fuzzy_score == 0.0

    if is_invalid:
      if self.holdover_frames[lead_idx] > 0:
        self.holdover_frames[lead_idx] -= 1
        return
      else:
        self.ema_confidence[lead_idx] = ALPHA_DOWN * 0.0 + (1 - ALPHA_DOWN) * self.ema_confidence[lead_idx]
        return

    self.holdover_frames[lead_idx] = RELEASE_FRAMES

    final_alpha_up = self._calculate_threat_multipliers(v_ego)
    target_ema = fuzzy_score
    alpha = final_alpha_up if fuzzy_score > 0.5 else ALPHA_DOWN
    
    new_ema = alpha * target_ema + (1 - alpha) * self.ema_confidence[lead_idx]
    new_ema = self._apply_slow_protection(v_ego, lead_prob, new_ema)

    self.ema_confidence[lead_idx] = new_ema


def get_lead_ext(
  v_ego: float,
  ready: bool,
  tracks: dict[int, TrackDP],
  lead_msg: capnp._DynamicStructReader,
  model_v_ego: float,
  lead_prob: float,
  low_speed_override: bool = True,
) -> dict[str, Any]:
  """
  DP 適配版：移除了 CP 與 CP_SP，純粹依靠 DP 的系統參數運作。
  """
  lead_idx = 0 if low_speed_override else 1
  max_ema_confidence = 0.0

  if ready:
    for track in tracks.values():
      track.process_track_logic(lead_idx, lead_msg, v_ego, lead_prob)

  valid_tracks = {k: v for k, v in tracks.items() if not v.is_out_of_lane and v.ema_confidence[lead_idx] > 0.0}

  if len(valid_tracks) > 0:
    max_ema_confidence = max(track.ema_confidence[lead_idx] for track in valid_tracks.values())

  current_prob_thres = float(np.interp(max_ema_confidence, EMA_VAL_RANGE, PROB_THRES_RANGE))

  selected_track = None
  if len(valid_tracks) > 0 and ready and lead_prob > current_prob_thres:
    selected_track = match_vision_to_track(v_ego, lead_msg, valid_tracks)

  # 狀態機記憶：雷達硬體斷流修補
  # 修正：只快取 trackId，續命時一律用 trackId 向本幀的 tracks dict 重新取值，
  # 絕不回傳凍結在舊幀的殭屍物件。若 trackId 已不在 tracks 裡（雷達真的斷流），
  # 代表沒有任何即時資料可續命，立即清除快取，不做無意義的凍結延續。
  cache = _LEAD_STATE_CACHE[lead_idx]
  if selected_track is not None:
    cache['track_id'] = selected_track.identifier
    cache['absent'] = 0
  elif cache['track_id'] is not None and cache['track_id'] in tracks:
    cache['absent'] += 1
    if cache['absent'] <= SELECT_HOLDOVER_FRAMES:
      selected_track = tracks[cache['track_id']]  # 重新查詢，取得本幀已更新的即時物件
    else:
      cache['track_id'] = None
      cache['absent'] = 0
  else:
    # trackId 已從雷達硬體消失（不只是這幀視覺比對失敗），沒有續命的意義
    cache['track_id'] = None
    cache['absent'] = 0

  lead_dict = {'status': False}
  if selected_track is not None:
    lead_dict = selected_track.get_RadarState(lead_prob)

    # 視覺加速度雙重驗證阻尼
    model_tau = get_model_lead_tau(lead_msg, lead_prob)
    if model_tau is not None:
      lead_dict['aLeadTau'] = model_tau

    if current_prob_thres < 0.5 and (0.5 >= lead_prob > current_prob_thres):
      cloudlog.debug(
        f"[RadarD_EarlyLock_DP] 提早鎖定/續命成功！目標 {lead_idx} | "
        f"相機機率: {lead_prob:.2f} (動態門檻: {current_prob_thres:.2f})"
      )

  elif (selected_track is None) and ready and (lead_prob > current_prob_thres):
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego, lead_prob)

  # 原廠底線救援
  if low_speed_override:
    low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
    if len(low_speed_tracks) > 0:
      closest_track = min(low_speed_tracks, key=lambda c: c.dRel)
      if (not lead_dict['status']) or (closest_track.dRel < lead_dict['dRel']):
        lead_dict = closest_track.get_RadarState()

  return lead_dict


# ==============================================================================
# 雙重 Monkey Patching
# ==============================================================================
radard.Track = TrackDP
radard.get_lead = get_lead_ext


class RadarDExt(RadarD):
  """
  DP 版專屬：初始化參數對齊 DP 的單一 delay 參數。
  """
  def __init__(self, delay: float = 0.0):
    super().__init__(delay)

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    super().update(sm, rr)
