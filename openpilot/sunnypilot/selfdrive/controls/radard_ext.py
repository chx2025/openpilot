#!/usr/bin/env python3
"""
雷达前车增强：提早锁定 (Early Lock) + 车道门限 + 模糊评分 + EMA 置信度

来源：herizon1054/openpilot@dpagel 的
    dragonpilot/selfdrive/controls/radard_ext.py
    selfdrive/controls/radard.py
适配到本分支（chx2025/openpilot@release260916xl，sunnypilot 系 monorepo）。

本分支与 dp 基底的差异（适配点，逐条对应）：
  1. import 路径：monorepo 用 openpilot.cereal / opendbc.car.structs
  2. LeadData 字段名：本分支是 present，dp 是 status
  3. 保留本分支特有的 get_custom_yrel（hyundai SCC 横向偏移修正）
  4. RadarD.__init__ 需要 CP / CP_SP（本分支签名）
  5. ★ Track.measured 在本分支恒为 False —— opendbc 已把 RadarPoint.measured
     移入 deprecated group 且没有任何驱动填充它（真机实测 1306/1306 采样全 False）。
     因此 dp 原版的 `is_turning and not self.measured` 不能照搬：照搬会让「转弯时
     所有 track 被判无效」，early lock 在转弯工况整体失效。
     转弯门控改为等价实现 —— 转弯时收紧横向模糊容限，见
     TURN_TIGHTEN_ENABLE / TURN_FUZZY_Y_MAX；置 TURN_TIGHTEN_ENABLE = False
     即可完全回退成与直行一致的行为。
  6. 静止目标过滤（本分支新增，非 dp 原有；2026-09-18 改版）：**雷达不处理静止物体**。
     目标只要「绝对速度 < GHOST_DROP_STATIC_KPH(5km/h)」就一律丢弃该雷达轨迹、
     退回纯视觉 lead。不再要求工况门（大转角>=60°/高速>=70km/h），也不再要求
     视觉原始置信度 < 0.35 —— 实测那套门太窄：中速(50~65km/h)+小转角时门不成立，
     幽灵照旧进 MPC（同一趟车 FCW triggered 3 次）。
     判据实现在 radard.is_stationary_track，参数在 radard.py 文件头 GHOST_* 常量段。
     这里负责在 get_lead_ext 的两个关键节点（匹配后 / 缓存续命前）把它拦下来。
  7. ★ 转弯时雷达彻底退出（车主 2026-09-19 实车要求）：
     「当方向盘角度大于 15 度就取消雷达对静止及运动物体的侦测」。
     关键点是**必须早于 ⑥**、且**连同 low_speed_override 低速兜底一起断掉** ——
     ⑥ 那条静止过滤**故意**不覆盖 low_speed_override（v_ego<4m/s 时正前方的
     静止目标极可能是真车，原厂防撞兜底要留着），所以只加 ⑥ 是拦不住
     「低速转弯时雷达把静止物当 lead」这条路的。实车证据见
     radard.py 文件头 TURN_DROP_* 段（[LongDecel] lead=1 vLeadK=-0.0 prob=0.00）。
     开关：radard.TURN_DROP_RADAR_ENABLE / radard.TURN_DROP_STEER_ANGLE_DEG

生效方式（monkey patch，由 radard.py 的 main() 触发）：
    radard.Track    -> TrackSP      车道边界门限 + 模糊评分 + EMA 置信度
    radard.get_lead -> get_lead_ext 动态概率阈值 + 早锁 + 慢车保护 + 选帧保持
radard.py 只在 main() 内把 RadarD 换成 RadarDSP，其余逻辑一行未动。
"""
import capnp
import numpy as np
import time
from typing import Any
from openpilot.cereal import messaging
from opendbc.car import structs
from opendbc.car.structs import car

# 引入整个 radard 模块做 monkey patch
from openpilot.selfdrive.controls import radard
from openpilot.selfdrive.controls.radard import (
    KalmanParams, Track, RadarD, match_vision_to_track,
    get_RadarState_from_vision, get_custom_yrel, RADAR_TO_CAMERA
)
from openpilot.common.constants import CV
from openpilot.common.swaglog import cloudlog

# ==============================================================================
# 提早锁定 (Early Lock) 扩充模块参数设定
# ==============================================================================
LANE_WIDTH_FALLBACK = 1.5           # 预测车道基准单侧半宽 (m)
LANE_HYSTERESIS_MARGIN = 0.5        # 边界外的迟滞容错余度 (m)
FUZZY_BOUNDS = [0.5, 1.5]           # 物理误差 (m 或 m/s): 0.5 以内给满分 1.0，大于 1.5 总分归零

ALPHA_BASE = 0.2                    # 常规上升学习率
ALPHA_DOWN = 0.1                    # 常规下降与短路过滤时的衰减学习率

BRAKE_THRES_RANGE = [-3.0, -1.2]    # 急刹触发区间 (m/s²)
MULT_RANGE = [1.2, 1.0]             # 对应威胁倍率
CUTIN_DIST_LIMIT = 40.0             # 评估切入威胁的最大纵向有效距离 (m)
DYNAMIC_SPEED_PCT = 0.2             # 动态相对速度阈值比例

CAM_PROB_SPEED_RANGE = [10.0, 25.0] # 动态相机门槛车速区间
CAM_PROB_RANGE = [0.5, 0.3]         # 动态相机审查门槛
STATIC_EMA_CAP = 0.6                # 目标未达审查门槛时的 EMA 天花板

EMA_VAL_RANGE = [0.4, 0.8]          # 本地 EMA 信心度 X 轴
PROB_THRES_RANGE = [0.5, 0.3]       # 映射出对应的「视觉提早放行门槛」 Y 轴

RELEASE_FRAMES = 5                  # 目标短暂丢失或出界时的 EMA 续命冻结帧数
SELECT_HOLDOVER_FRAMES = 3          # 雷达硬件断流时，强制维持上一帧锁定的帧数

MODEL_TAU_MIN_PROB = 0.5            # 启动验证的最低视觉机率
MODEL_TAU_BRAKE_A = -0.5            # 启动验证的最低急刹门槛 (m/s²)
MODEL_TAU_SUSTAINED = 0.5           # 视觉确认急刹持续
MODEL_TAU_SPURIOUS = 3.0            # 视觉预测即将加速

# 全局缓存：直接缓存 Track 物件本身。
# dp: 额外加上 last_aLeadK，用来在「冻结中」跟「刚恢复匹配」两种情况下，
# 都对输出的 aLeadK 做变化率限制，避免瞬间跳动触发幽灵刹车
_LEAD_STATE_CACHE = {
    0: {'track': None, 'absent': 0, 'last_aLeadK': None},
    1: {'track': None, 'absent': 0, 'last_aLeadK': None}
}
MAX_ALEADK_DELTA_PER_FRAME = 1.0    # aLeadK 每帧最大允许变化量 (m/s²)，可依实测调整

# 转弯判定门槛定义在 radard.py（TURN_STEER_ANGLE_DEG / TURN_STEER_RATE_DEG），
# 由 radard.RadarD.update() 算好 is_turning 后传进来，这里不再重复定义。

# ---- 转弯门控（等价替代 dp 的 Track.measured 判据）----
# dp 原版转弯时要求雷达点「必须是真实量测」，而本分支 RadarPoint.measured 已废弃、
# 恒为 False，该判据无法使用。这里改用等价的保守手段：转弯时把横向模糊误差的归零
# 阈值从 1.5m 收紧到 TURN_FUZZY_Y_MAX —— err_y 一旦超过它 fuzzy_score 直接归零、
# 目标判为无效，等效于「转弯时只接受与视觉高度吻合的目标」。
# 直行保持原阈值，不拖慢插队目标的信心度累积。
TURN_TIGHTEN_ENABLE = True          # 置 False 即回退成与直行完全一致的行为
TURN_FUZZY_Y_MAX = 1.0              # 转弯时 err_y 归零阈值 (m)；直行为 FUZZY_BOUNDS[1] = 1.5


# ---- 静止目标丢弃日志 ----
# 为什么用 info 而不是 debug：swaglog 落盘的最低级别是 INFO(20)，cloudlog.debug(10)
# 在 /data/log/swaglog.* 里根本查不到。实车验证要靠这份日志留证据，所以只能用 info；
# 既然用了 info 就必须节流 —— 轨迹持续被判静止时是 20Hz，不节流会刷爆日志。
#
# 实车核对：grep RadarDSP_StaticDrop /data/log/swaglog.*
# （2026-09-18 前的旧版本日志名是 RadarDSP_GhostDrop，grep 时可一并带上）
GHOST_LOG_INTERVAL = 3.0            # 同一路 lead 最少间隔 (s)
_GHOST_LOG_TS = {0: -1e9, 1: -1e9}

# ---- 转弯时雷达退出 lead 判定的留证日志（车主 2026-09-19 要求）----
# 判据与理由见 radard.py 文件头 TURN_DROP_* 常量段。这里只负责留一条证据：
# 实车核对：grep RadarDSP_TurnDrop /data/log/swaglog.*
# 节流同样用 GHOST_LOG_INTERVAL —— 转弯可以持续十几秒，20 Hz 不节流会刷爆。
_TURN_DROP_LOG_TS = {0: -1e9, 1: -1e9}


def log_turn_drop(lead_idx: int, v_ego: float, lead_prob: float) -> None:
  now = time.monotonic()
  if now - _TURN_DROP_LOG_TS[lead_idx] < GHOST_LOG_INTERVAL:
    return
  _TURN_DROP_LOG_TS[lead_idx] = now
  cloudlog.info(
    f"[RadarDSP_TurnDrop] lead{lead_idx} | 转弯中雷达退出，退回纯视觉 "
    f"vEgo {v_ego * CV.MS_TO_KPH:.0f}kph leadProb {lead_prob:.2f}"
  )


def log_stationary_drop(lead_idx: int, track, v_ego: float, cam_prob: float) -> None:
  """静止目标被丢弃时留一条证据。

  vEgo   : 用来区分「中速巡航丢掉幽灵」与「拥堵跟车误丢慢车」两种情况
  camProb: 只作参考记录，不再是判据（详见 radard.py 文件头 GHOST_* 说明）
  """
  now = time.monotonic()
  if now - _GHOST_LOG_TS[lead_idx] < GHOST_LOG_INTERVAL:
    return
  _GHOST_LOG_TS[lead_idx] = now
  cloudlog.info(
    f"[RadarDSP_StaticDrop] lead{lead_idx} | dRel {track.dRel:.1f} yRel {track.yRel:.1f} "
    f"vLeadK {track.vLeadK:.1f} camProb {cam_prob:.2f} vEgo {v_ego * CV.MS_TO_KPH:.0f}kph"
  )


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


class TrackSP(Track):
  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
    super().__init__(identifier, v_lead, kalman_params)
    self.ema_confidence = {0: 0.4, 1: 0.4}
    self.holdover_frames = {0: 0, 1: 0}
    self.is_out_of_lane = False
    # 本分支 RadarPoint.measured 已废弃（恒 False），这里给个明确默认值，
    # 避免任何路径上访问到未定义属性。
    self.measured = False

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

  def _calculate_fuzzy_score(self, offset_vision_dist: float, vision_y: float, vision_v: float, v_ego: float,
                             is_turning: bool = False) -> float:
    err_d = abs(self.dRel - offset_vision_dist)
    err_y = abs(self.yRel - vision_y)
    err_v = abs((self.vRel + v_ego) - vision_v)

    # 转弯时收紧横向容限（详见文件头 TURN_TIGHTEN_ENABLE 的说明）
    y_zero = TURN_FUZZY_Y_MAX if (TURN_TIGHTEN_ENABLE and is_turning) else FUZZY_BOUNDS[1]

    score_d = float(np.interp(err_d, FUZZY_BOUNDS, [1.0, 0.0]))
    score_y = float(np.interp(err_y, [FUZZY_BOUNDS[0], y_zero], [1.0, 0.0]))
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
                          lead_prob: float, is_turning: bool = False):
    offset_vision_dist = lead_msg.x[0] - RADAR_TO_CAMERA
    vision_y = -lead_msg.y[0]
    vision_v = lead_msg.v[0]

    # dp 原版（dragonpilot 基底）：
    #   is_invalid = (is_turning and not self.measured) or abs(self.yRel - vision_y) > (LANE_WIDTH_FALLBACK + LANE_HYSTERESIS_MARGIN)
    #
    # 本分支适配：RadarPoint.measured 已被 opendbc 移入 deprecated 且无人填充（实测恒 False），
    # 照搬会让「转弯时所有 track 被判无效」。转弯门控改由 _calculate_fuzzy_score 的
    # 横向容限收紧实现（见该函数与 TURN_TIGHTEN_ENABLE 的说明），此处判据保持与原版一致。
    is_invalid = abs(self.yRel - vision_y) > (LANE_WIDTH_FALLBACK + LANE_HYSTERESIS_MARGIN)

    fuzzy_score = 0.0
    if not is_invalid:
      is_valid_spatial = self._check_spatial_boundaries(vision_y)
      fuzzy_score = self._calculate_fuzzy_score(offset_vision_dist, vision_y, vision_v, v_ego, is_turning)
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
  tracks: dict[int, TrackSP],
  lead_msg: capnp._DynamicStructReader,
  model_v_ego: float,
  lead_prob: float,
  CP: structs.CarParams,
  CP_SP: structs.CarParamsSP,
  is_turning: bool = False,
  low_speed_override: bool = True,
  cam_prob: float = 1.0,
  radar_drop: bool = False,
) -> dict[str, Any]:
  """
  本分支适配版：
    - 保留 CP / CP_SP（供本分支特有的 get_custom_yrel 使用）
    - 新增 is_turning：由 radard.py 依方向盘角度/角速度判断，传给 process_track_logic
    - 新增 radar_drop：由 radard.py 依**方向盘角度**判断（>= TURN_DROP_STEER_ANGLE_DEG），
      为真时雷达彻底退出 lead 判定（静止物 + 运动物都不取），退回纯视觉 lead。
      这是车主 2026-09-19 的实车要求，判据与理由见 radard.py 文件头 TURN_DROP_* 段。
    - 保留 dp 的 aLeadK 变化率限制（防幽灵刹车）
    - cam_prob：仅用于丢弃日志留证，不参与判据
    - 静止目标过滤：见 radard.is_stationary_track（雷达不处理静止物体）。
      注意本函数是 radard.py 里 get_lead 的实际执行体（模块底部 monkey patch），
      所以过滤逻辑必须落在这里，写在 radard.get_lead 里是不会被调用的。
  """
  lead_idx = 0 if low_speed_override else 1
  max_ema_confidence = 0.0

  # ── 转弯时雷达彻底退出（必须放在最前面）───────────────────────────────
  # 放最前面是为了把**三条**「绕回雷达」的路一起断掉，否则本帧不取雷达、
  # 缓存里的旧 track 会在下一帧被补回来，等于没改：
  #   ① 原厂 low_speed_override 低速兜底（函数末尾，v_ego<4m/s 用最近目标）
  #   ② SELECT_HOLDOVER_FRAMES 的选帧冻结续命（缓存里 track 还在）
  #   ③ EMA 置信度累积留下的 valid_tracks
  # 所以这里直接把缓存清空，再直接返回视觉 lead。
  if radar_drop and radard.TURN_DROP_RADAR_ENABLE:
    _LEAD_STATE_CACHE[lead_idx] = {'track': None, 'absent': 0, 'last_aLeadK': None}
    log_turn_drop(lead_idx, v_ego, lead_prob)
    if ready and lead_prob > .5:
      return get_RadarState_from_vision(lead_msg, v_ego, model_v_ego, lead_prob)
    return {'present': False}

  if ready:
    for track in tracks.values():
      track.process_track_logic(lead_idx, lead_msg, v_ego, lead_prob, is_turning)

  valid_tracks = {k: v for k, v in tracks.items() if not v.is_out_of_lane and v.ema_confidence[lead_idx] > 0.0}

  if len(valid_tracks) > 0:
    max_ema_confidence = max(track.ema_confidence[lead_idx] for track in valid_tracks.values())

  current_prob_thres = float(np.interp(max_ema_confidence, EMA_VAL_RANGE, PROB_THRES_RANGE))

  selected_track = None
  if len(valid_tracks) > 0 and ready and lead_prob > current_prob_thres:
    selected_track = match_vision_to_track(v_ego, lead_msg, valid_tracks)
    # 静止目标一律丢弃（雷达不处理静止物体）：本次不用雷达测出来的速度和加速度，
    # 上层会退回纯视觉 lead。判据与调参见 radard.py 文件头 GHOST_* / is_stationary_track。
    if selected_track is not None and radard.is_stationary_track(selected_track):
      log_stationary_drop(lead_idx, selected_track, v_ego, cam_prob)
      selected_track = None

  # 状态机记忆：僵尸物件强制续命（直接缓存物件）
  cache = _LEAD_STATE_CACHE[lead_idx]
  if selected_track is not None:
    cache['track'] = selected_track
    cache['absent'] = 0
  elif cache['track'] is not None:
    # 缓存里的轨迹一旦变成静止目标必须立刻清掉：否则它会在接下来
    # SELECT_HOLDOVER_FRAMES 帧里被「冻结续命」重新送回上层，幽灵刹车照旧。
    if radard.is_stationary_track(cache['track']):
      log_stationary_drop(lead_idx, cache['track'], v_ego, cam_prob)
      cache['track'] = None
      cache['absent'] = 0
      cache['last_aLeadK'] = None
    else:
      cache['absent'] += 1
      if cache['absent'] <= SELECT_HOLDOVER_FRAMES:
        selected_track = cache['track']  # 强制回传上一刻的冻结物件，维持锁定
      else:
        cache['track'] = None
        cache['absent'] = 0
        cache['last_aLeadK'] = None  # lead 真正消失，重置参考基准，避免下一个新目标被错误地拿旧值做限制

  lead_dict = {'present': False}
  if selected_track is not None:
    lead_dict = selected_track.get_RadarState(lead_prob)
    lead_dict = get_custom_yrel(CP, CP_SP, lead_dict, lead_msg)

    # dp: 不管是「冻结续命中」还是「刚恢复匹配、瞬间跳到最新卡尔曼值」，
    # 都对 aLeadK 做变化率限制，避免瞬间跳动被误判成前车突然减速（幽灵刹车）。
    # 只限制 aLeadK，dRel/yRel/vRel 不受影响，维持插队侦测所需的位置即时性。
    if cache['last_aLeadK'] is not None:
      raw_aLeadK = lead_dict['aLeadK']
      delta = float(np.clip(raw_aLeadK - cache['last_aLeadK'], -MAX_ALEADK_DELTA_PER_FRAME, MAX_ALEADK_DELTA_PER_FRAME))
      lead_dict['aLeadK'] = float(cache['last_aLeadK'] + delta)
    cache['last_aLeadK'] = float(lead_dict['aLeadK'])

    # 视觉加速度双重验证阻尼
    model_tau = get_model_lead_tau(lead_msg, lead_prob)
    if model_tau is not None:
      lead_dict['aLeadTau'] = model_tau

    if current_prob_thres < 0.5 and (0.5 >= lead_prob > current_prob_thres):
      cloudlog.debug(
        f"[RadarDSP_EarlyLock] Target {lead_idx} | "
        f"Prob: {lead_prob:.2f} (Thres: {current_prob_thres:.2f}) | Turning: {is_turning}"
      )

  elif (selected_track is None) and ready and (lead_prob > current_prob_thres):
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego, lead_prob)
    _LEAD_STATE_CACHE[lead_idx]['last_aLeadK'] = None  # 纯视觉后备路径不经过雷达物件，重置参考基准

  # 原厂底线救援
  # 注意：这里**故意不套用静止目标过滤**。potential_low_speed_lead 要求 v_ego < 4 m/s
  # 且目标在正前方 0.75~25m，属于低速近距离防撞兜底（掉头、起步、拥堵蠕行）。
  # 这种工况下正前方的静止目标极可能就是真车/真人，丢掉代价远大于幽灵刹车。
  if low_speed_override:
    low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
    if len(low_speed_tracks) > 0:
      closest_track = min(low_speed_tracks, key=lambda c: c.dRel)
      if (not lead_dict['present']) or (closest_track.dRel < lead_dict['dRel']):
        lead_dict = closest_track.get_RadarState()

  return lead_dict


# ==============================================================================
# 双重 Monkey Patching
# ==============================================================================
radard.Track = TrackSP
radard.get_lead = get_lead_ext


class RadarDSP(RadarD):
  """本分支专属：初始化参数对齐 radard.RadarD 的 (CP, CP_SP, delay) 签名。"""

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, delay: float = 0.0):
    super().__init__(CP, CP_SP, delay)

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    super().update(sm, rr)
