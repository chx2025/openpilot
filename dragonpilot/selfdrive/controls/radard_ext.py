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
    KalmanParams, Track, RadarD,
    get_RadarState_from_vision, RADAR_TO_CAMERA, laplacian_pdf
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
# 路徑校正相關門檻 (方案 B + 彎道真前車保護)
# 原本用 LANE_WIDTH_FALLBACK + LANE_HYSTERESIS_MARGIN (2.0m) 當單一硬門檻，
# 直接把超界的 track 從 valid_tracks 剔除，屬於二元判斷。問題是 modelV2 預測
# 路徑本身在急彎/路口/匝道等場景會跟實際道路有落差，若路徑本身誤差夠大，
# 這個硬門檻可能連真正的彎道前車都一起殺掉。改成三層漸進式：
#   1. 小偏差 (< PATH_Y_SOFT_UPPER)：正常評分
#   2. 中偏差 (SOFT_UPPER ~ HARD_EXCLUDE)：不直接剔除，交給 fuzzy score 自然降權
#   3. 大偏差 (> PATH_Y_HARD_EXCLUDE)：才進一步用「未經路徑校正的原始 yRel 誤差」
#      做二次確認 —— 只有路徑相對誤差跟原始誤差「兩者都」超標，才代表這很可能
#      真的是隔壁車道目標，而不是模型路徑預測本身的誤差，此時才真正排除。
PATH_Y_SOFT_UPPER = [0.5, 1.8]      # score_y 用這組較寬容的邊界做漸進降權 (取代 FUZZY_BOUNDS)
PATH_Y_HARD_EXCLUDE_MARGIN = 2.3    # 路徑相對誤差的絕對排除門檻 (近距離基準值)
RAW_Y_FALLBACK_MARGIN = 2.0         # 大偏差時的原始 yRel 誤差二次確認門檻 (近距離基準值)

# 距離縮放：路徑預測與量測本身的不確定性會隨距離增加而放大（模型對遠處路徑
# 曲率的預測誤差、雷達/視覺在遠處的量測雜訊都更大），近距離判斷可以較嚴格，
# 遠距離則不該因為單幀的路徑或原始 Y 誤差偏大就直接排除，避免遠處的彎道
# 真前車被誤殺。用 track 自身的 dRel 對兩個門檻做線性放大。
DIST_MARGIN_NEAR_REF = 15.0         # m，此距離以內用基準門檻，不放大
DIST_MARGIN_FAR_REF = 60.0          # m，此距離以上用最大放大倍率
DIST_MARGIN_FAR_SCALE = 1.6         # 遠距離時門檻放大倍率


def _distance_scaled(base_margin: float, d_rel: float) -> float:
  scale = float(np.interp(d_rel, [DIST_MARGIN_NEAR_REF, DIST_MARGIN_FAR_REF], [1.0, DIST_MARGIN_FAR_SCALE]))
  return base_margin * scale

_LEAD_STATE_CACHE = {
    0: {'track_id': None, 'absent': 0},
    1: {'track_id': None, 'absent': 0}
}

# 全域快取：最新一幀的 modelV2 訊息，用於路徑座標校正 (方案 B)。
# 原因同上：radard.py 的 RadarD.update() 呼叫 get_lead(...) 時簽名是寫死的，
# 無法額外多傳 model_v2 參數。因此改由 RadarDExt.update() 在呼叫 super().update()
# 之前，把當幀 sm['modelV2'] 存進這個全域變數，get_lead_ext 內部再讀取使用，
# 對外的函數簽名維持與原版 radard.get_lead 完全一致，monkey patch 才不會出錯。
_CURRENT_MODEL_V2 = None


def path_y_at_distance(model_v2, d_rel: float) -> float | None:
  """
  方案 B：內插 modelV2.position.x/y，取得模型預測路徑在縱向距離 d_rel 處的橫向座標。

  重要：np.interp() 對超出範圍的輸入不會報錯，而是直接回傳最靠近的端點值
  （相當於假設路徑在模型預測範圍外「維持最後一點的橫向位置不變」）。這在直線
  路段影響不大，但在彎道會是錯的——例如模型只預測到 30m，前車在 50m，
  若直接外插等於假設 30~50m 之間車道沒有繼續彎，這會讓真正的彎道前車被算出
  錯誤的 path-relative Y，進而被降權甚至誤判排除。

  所以這裡明確判斷 d_rel 是否落在模型實際預測範圍內；超出範圍時回傳 None，
  由呼叫端決定要退回「未經路徑校正」的原始座標比較，而不是用不可靠的外插值。
  """
  if model_v2 is None:
    return None
  xs = model_v2.position.x
  ys = model_v2.position.y
  if len(xs) < 2:
    return None
  if d_rel < xs[0] or d_rel > xs[-1]:
    return None
  return float(np.interp(d_rel, xs, ys))


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

  def _check_spatial_boundaries(self, vision_y_rel_path: float, track_y_rel_path: float, hard_exclude_margin: float) -> bool:
    # 注意：這兩個座標參數都已經是「相對於模型預測路徑」的橫向座標(見
    # process_track_logic 裡的 path_y_at_distance 校正，若模型路徑資料不可靠則
    # 退回原始座標)。hard_exclude_margin 是依 track 自身 dRel 距離縮放過的絕對
    # 邊界（見 _distance_scaled），而非固定用近距離的 PATH_Y_HARD_EXCLUDE_MARGIN，
    # 因為路徑預測誤差會隨距離放大，遠處若仍用近距離門檻，容易誤殺彎道真前車；
    # 細緻的漸進降權則交給 _calculate_fuzzy_score 的 score_y 處理。
    left_bound = vision_y_rel_path + hard_exclude_margin
    right_bound = vision_y_rel_path - hard_exclude_margin
    current_y = track_y_rel_path

    if not self.is_out_of_lane:
      if current_y > (left_bound + LANE_HYSTERESIS_MARGIN) or current_y < (right_bound - LANE_HYSTERESIS_MARGIN):
        self.is_out_of_lane = True
    else:
      if right_bound <= current_y <= left_bound:
        self.is_out_of_lane = False

    return not self.is_out_of_lane

  def _calculate_fuzzy_score(self, offset_vision_dist: float, vision_y_rel_path: float, vision_v: float,
                              v_ego: float, track_y_rel_path: float) -> float:
    err_d = abs(self.dRel - offset_vision_dist)
    # err_y 改用路徑相對座標之差，而非原始 yRel 之差，避免過彎時因車頭直線與
    # 實際路徑分岔，導致彎道內真實前車的 y 分數系統性偏低。
    err_y = abs(track_y_rel_path - vision_y_rel_path)
    err_v = abs((self.vRel + v_ego) - vision_v)

    score_d = float(np.interp(err_d, FUZZY_BOUNDS, [1.0, 0.0]))
    # score_y 用較寬容的邊界，讓路徑相對誤差在中段區間漸進降權，
    # 而不是跟 d/v 一樣在 1.5m 就直接歸零，避免彎道中合理的路徑預測誤差
    # 造成真前車信心度斷崖式下跌。
    score_y = float(np.interp(err_y, PATH_Y_SOFT_UPPER, [1.0, 0.0]))
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

  def process_track_logic(self, lead_idx: int, lead_msg: capnp._DynamicStructReader, v_ego: float,
                           lead_prob: float, model_v2=None):
    offset_vision_dist = lead_msg.x[0] - RADAR_TO_CAMERA
    vision_y = -lead_msg.y[0]
    vision_v = lead_msg.v[0]

    # 方案 B 路徑校正：分別在 track 與 vision 各自的縱向距離處查出模型路徑的 y，
    # 因為 self.dRel (雷達量測距離) 與 offset_vision_dist (視覺估計距離) 通常
    # 略有差異，路徑本身在不同距離處的 y 也不同，不能共用同一個路徑 y 值。
    track_path_y = path_y_at_distance(model_v2, self.dRel)
    vision_path_y = path_y_at_distance(model_v2, offset_vision_dist)

    if track_path_y is None or vision_path_y is None:
      # 兩者任一超出模型實際預測範圍（例如遠距離前車，模型只預測到中距離），
      # 代表沒有可靠的路徑校正依據，寧可退回原始（未路徑校正）座標比較，
      # 也不要用 np.interp 端點外插值冒充路徑推論。
      track_y_rel_path = self.yRel
      vision_y_rel_path = vision_y
    else:
      track_y_rel_path = self.yRel - track_path_y
      vision_y_rel_path = vision_y - vision_path_y

    path_err_y = abs(track_y_rel_path - vision_y_rel_path)
    hard_exclude_margin = _distance_scaled(PATH_Y_HARD_EXCLUDE_MARGIN, self.dRel)
    raw_fallback_margin = _distance_scaled(RAW_Y_FALLBACK_MARGIN, self.dRel)

    is_invalid = not self.measured

    # 大偏差二次確認：path-relative 誤差超過（距離縮放後的）絕對排除門檻時，
    # 不要立刻判定 invalid，改用「未經路徑校正的原始 yRel 誤差」再檢查一次。
    # 只有路徑相對誤差跟原始誤差兩者都超標，才代表這很可能真的是隔壁車道目標，
    # 而非單純模型路徑預測本身在這一幀的誤差；門檻隨距離放大，避免遠處的
    # 彎道真前車因單幀誤差被誤判排除。
    if not is_invalid and path_err_y > hard_exclude_margin:
      raw_err_y = abs(self.yRel - vision_y)
      if raw_err_y > raw_fallback_margin:
        is_invalid = True

    fuzzy_score = 0.0
    if not is_invalid:
      is_valid_spatial = self._check_spatial_boundaries(vision_y_rel_path, track_y_rel_path, hard_exclude_margin)
      fuzzy_score = self._calculate_fuzzy_score(offset_vision_dist, vision_y_rel_path, vision_v, v_ego, track_y_rel_path)
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


def match_vision_to_track_ext(v_ego: float, lead: capnp._DynamicStructReader, tracks: dict[int, TrackDP], model_v2=None):
  """
  path-aware 版本的 match_vision_to_track。原版 radard.match_vision_to_track 的
  prob_y 直接用未校正的 c.yRel 跟 -lead.y[0] 算機率，跟 process_track_logic 已經
  改用路徑相對座標篩選 valid_tracks 的邏輯不一致：前面用路徑校正判斷「這是彎道
  合理前車」，後面挑選時卻又用原始座標判斷「Radar y 跟 Vision y 差很多」，會把
  方案 B 的效果部分抵消。這裡讓 prob_y 也改用路徑相對座標，讓篩選跟挑選兩階段
  使用同一套座標系。
  """
  offset_vision_dist = lead.x[0] - RADAR_TO_CAMERA
  vision_y_raw = -lead.y[0]
  vision_path_y = path_y_at_distance(model_v2, offset_vision_dist)

  def prob(c: TrackDP):
    track_path_y = path_y_at_distance(model_v2, c.dRel)
    if track_path_y is None or vision_path_y is None:
      # 任一距離超出模型有效預測範圍時，退回原始座標比較，跟
      # process_track_logic 的處理方式一致，避免用外插值誤導 matching。
      track_y_rel_path = c.yRel
      vision_y_rel_path = vision_y_raw
    else:
      track_y_rel_path = c.yRel - track_path_y
      vision_y_rel_path = vision_y_raw - vision_path_y

    prob_d = laplacian_pdf(c.dRel, offset_vision_dist, lead.xStd[0])
    prob_y = laplacian_pdf(track_y_rel_path, vision_y_rel_path, lead.yStd[0])
    prob_v = laplacian_pdf(c.vRel + v_ego, lead.v[0], lead.vStd[0])

    return prob_d * prob_y * prob_v

  track = max(tracks.values(), key=prob)

  # sanity check 維持跟原版一致，不受路徑校正影響（距離、速度本來就跟路徑幾何無關）
  dist_sane = abs(track.dRel - offset_vision_dist) < max([(offset_vision_dist) * .25, 5.0])
  vel_sane = (abs(track.vRel + v_ego - lead.v[0]) < 10) or (v_ego + track.vRel > 3)
  if dist_sane and vel_sane:
    return track
  else:
    return None


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
      track.process_track_logic(lead_idx, lead_msg, v_ego, lead_prob, _CURRENT_MODEL_V2)

  valid_tracks = {k: v for k, v in tracks.items() if not v.is_out_of_lane and v.ema_confidence[lead_idx] > 0.0}

  if len(valid_tracks) > 0:
    max_ema_confidence = max(track.ema_confidence[lead_idx] for track in valid_tracks.values())

  current_prob_thres = float(np.interp(max_ema_confidence, EMA_VAL_RANGE, PROB_THRES_RANGE))

  selected_track = None
  if len(valid_tracks) > 0 and ready and lead_prob > current_prob_thres:
    selected_track = match_vision_to_track_ext(v_ego, lead_msg, valid_tracks, _CURRENT_MODEL_V2)

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
    global _CURRENT_MODEL_V2
    # 在呼叫父類別 update() 之前先存好本幀的 modelV2，因為 super().update()
    # 內部會呼叫已被 monkey patch 的 get_lead_ext(...)，而其簽名跟原版
    # radard.get_lead 一致、無法額外傳入 model_v2，只能透過這個全域變數取得。
    # sm['modelV2'] 在尚未收到第一幀真實訊息前會是預設空物件
    # (position.x/y 長度為 0)，path_y_at_distance() 已對此做防呆處理，
    # 回傳 0.0 等同於退回原本「相對車頭直線」的行為，不會噴例外。
    _CURRENT_MODEL_V2 = sm['modelV2']
    super().update(sm, rr)
