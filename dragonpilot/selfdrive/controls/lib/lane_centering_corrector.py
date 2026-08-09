#!/usr/bin/env python3
"""
Lane Centering Corrector (LCC)

根據 modelV2 的車道線 (laneLines) 幾何位置，計算「本車道中心 offset」，
並轉換成一個很小的曲率修正量，疊加在 model 原本輸出的 desired_curvature 上。

設計原則：
  - 完全獨立模組，不觸碰 latcontrol.py / latcontrol_pid.py / latcontrol_torque.py /
    latcontrol_angle.py 任何邏輯，這些檔案維持 upstream 原樣。
  - controlsd.py 只在組出 new_desired_curvature 之後、丟進 clip_curvature() 之前，
    疊加本模組算出的修正量，修正量算錯或關閉時 correction=0，行為等同原本 model。
  - 下游 clip_curvature() 的曲率/側向加速度/側向 jerk 限幅仍然完整作用於
    「model + 修正量」的總和，所以就算本模組給錯值，也會被既有安全限制包住。
  - 車道線信心不足（單線消失、施工、匝道...）時修正量直接淡出到 0，不做外插硬猜。

尚未實際路測，預設 dp_lcc_enabled=False，需在 Params 手動開啟。
"""

import os
import time

import numpy as np

from openpilot.common.params import Params
from openpilot.common.filter_simple import FirstOrderFilter

PARAM_REFRESH_SEC = 2.0

# --- Debug log 設定（獨立 CSV，供路測後回傳分析用） ---
LOG_PATH = "/data/media/0/realdata/lcc_debug.csv"
LOG_INTERVAL_SEC = 0.1  # 穩態最多每 0.1s（10Hz）寫一次，避免頻繁寫入 SD 卡
LOG_COLUMNS = [
  "t", "dt", "v_kph", "state",
  "lat_active", "speed_gate", "sharp_turn", "model_curvature",
  "yield_hold_timer", "engage_ramp_timer", "ramp_factor",
  "weight", "lane_center_error", "error_rate", "p_term", "d_term",
  "raw_correction", "correction",
]

# --- 系統內建參數（不從 Params 讀取，先求穩，之後有需要再開放調參） ---
# 車速遲滯開關（單位 km/h）：
#   - v >= 40 km/h 才「開啟」車道居中修正
#   - v <= 30 km/h 就「關閉」，退回原本 openpilot model 的橫向控制
#   - 30~40 km/h 之間維持前一個狀態，避免在門檻附近來回抖動 on/off
SPEED_ON_KPH = 40.0
SPEED_OFF_KPH = 30.0
KPH_TO_MS = 1000.0 / 3600.0

LOOKAHEAD_DIST_M = 15.0     # 用來算車道中心 offset 的前視距離
FILTER_RC_SEC = 0.5         # 修正量一階低通時間常數，避免抖動

# 一般彎道（高速公路/山路彎）曲率通常遠小於這個值，即使有彎度也照樣居中修正；
# 路口轉彎（例如十字路口右轉）曲率半徑很小、曲率值很大，超過這個門檻就
# 直接停用 LCC，退回原本 model 控制，避免在路口硬要「拉回車道中心」。
#
# 門檻校準依據（市區路口右轉）：
#   - 路緣轉角半徑常見約 5~15m（一般巷道 5~9m，幹道為了讓大車轉彎可到 12~15m）
#   - 車輛實際路徑半徑 = 路緣半徑 + 車道偏移量(約 1.5~2.5m) ≈ 8~17m
#   - 取寬鬆端（較大路徑半徑 17m）反推曲率，確保連轉角較大的幹道路口也能被抓到
#   - 半徑 17m -> 曲率 ≈ 0.059 1/m，取整為 0.06
# 注意：這只是第二道防線，主要保護是車速遲滯開關（LCC 僅 >=40km/h 啟用），
# 正常情況下車輛進入路口右轉前車速早已降到 30km/h 以下、閘門就會先關掉 LCC。
SHARP_TURN_CURVATURE = 0.06  # 1/m，約等於路徑半徑 17m 以下的轉彎

# 車道線信心度：低於 PROB_MIN 直接不修正；高於 PROB_FULL 才給滿權重；
# 中間線性淡入淡出，避免 on/off 硬切
PROB_MIN = 0.3
PROB_FULL = 0.6

MAX_CORRECTION = 0.006  # 修正量曲率上限 (1/m)，寫死在程式碼內
# 2026-08 更新：路測發現持續性左右振盪（約 4~6 秒週期），研判是純 P 控制迴路增益
# 過高（前次從 0.005 調到 0.015 放大了增益）。這裡先大幅調回保守值降低振幅風險，
# 同時新增 KD 阻尼項（見下方 KD_GAIN）從純 P 改為 PD 控制抑制過衝，兩者一起處理。
# 重新啟用測試務必從低速、空曠道路開始，確認不再振盪後再逐步提高。

KD_GAIN = 0.6  # 阻尼增益：誤差變化率的抑制強度，0 = 純 P 控制（跟振盪前一致）

# 「方向燈+出力」讓開判斷的去彈跳時間：條件要連續成立超過這個時間才真的觸發讓開，
# 用來濾掉打方向燈瞬間、手還沒真的出力就先閃一下 steeringPressed 的雜訊/短暫誤觸
YIELD_CONFIRM_SEC = 0.15

# 剛從無效狀態（車速不夠/急彎/解除自駕...）變成有效狀態的那一刻，修正量不會直接套用
# 當下算出來的量，而是從 0 用這段時間線性爬升到全額。避免 engage 瞬間如果車輛本來就
# 不在車道中心，一次給太大修正量在車道線量測延遲下造成來回振盪（「啟動瞬間晃動」）。
ENGAGE_RAMP_SEC = 1.5


def _clip_interp(x, xp, fp):
  """對 modelV2 XYZTData 的 x/y 陣列做前視距離內插，資料異常時回傳 None"""
  if len(xp) < 2:
    return None
  # laneLines 的 x 應該是遞增的（沿著車輛前進方向），不是的話视為異常資料
  if xp[-1] <= xp[0]:
    return None
  if x < xp[0] or x > xp[-1]:
    return None
  return float(np.interp(x, xp, fp))


class LaneCenteringCorrector:
  def __init__(self) -> None:
    self._params = Params()
    self._last_params_read = 0.0

    # --- UI 控制參數（從 Params 讀取） ---
    self._enabled = False

    # --- 狀態 ---
    self._filter = FirstOrderFilter(0.0, FILTER_RC_SEC, 0.01)
    self.correction = 0.0
    self.lane_center_error = 0.0
    self._prev_lane_center_error = 0.0  # 上一幀誤差，用於 D 項計算
    self._prev_error_valid = False  # False 時代表沒有上一幀可比較，D 項該幀直接視為 0
    self.weight = 0.0
    self._active = False
    self._speed_gate = False  # 車速遲滯開關的目前狀態
    self._yield_hold_timer = 0.0  # 「方向燈+出力」持續時間累計，用於去彈跳
    self._engage_ramp_timer = 0.0  # 剛啟用時的漸強爬升時間累計

    # --- debug log 狀態 ---
    self._last_log_time = 0.0
    self._last_logged_state = None
    self._log_header_written = os.path.exists(LOG_PATH)

  def _read_params(self) -> None:
    now = time.monotonic()
    if now - self._last_params_read < PARAM_REFRESH_SEC:
      return
    self._last_params_read = now
    self._enabled = self._params.get_bool("dp_lcc_enabled")

  def reset(self) -> None:
    self._filter.x = 0.0
    self.correction = 0.0
    self.lane_center_error = 0.0
    self._prev_lane_center_error = 0.0
    self._prev_error_valid = False
    self.weight = 0.0
    self._active = False
    self._speed_gate = False
    self._yield_hold_timer = 0.0
    self._engage_ramp_timer = 0.0

  def _log_row(self, state: str, **fields) -> None:
    """寫一行 debug log。狀態轉換一定會寫（不受 throttle 限制），穩態時每
    LOG_INTERVAL_SEC 最多寫一次，避免高頻寫入 SD 卡。失敗直接吞掉，不影響控制迴路。"""
    now = time.monotonic()
    state_changed = state != self._last_logged_state
    if not state_changed and (now - self._last_log_time) < LOG_INTERVAL_SEC:
      return
    self._last_log_time = now
    self._last_logged_state = state
    try:
      row = {"t": f"{time.time():.3f}", "state": state}
      for col in LOG_COLUMNS:
        if col in ("t", "state"):
          continue
        val = fields.get(col, "")
        row[col] = f"{val:.5f}" if isinstance(val, float) else str(val)
      os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)
      write_header = not self._log_header_written
      with open(LOG_PATH, "a", encoding="utf-8") as f:
        if write_header:
          f.write(",".join(LOG_COLUMNS) + "\n")
          self._log_header_written = True
        f.write(",".join(row.get(col, "") for col in LOG_COLUMNS) + "\n")
    except Exception:
      pass

  def update(self, model_v2, v_ego: float, lat_active: bool, dt: float,
             left_blinker: bool = False, right_blinker: bool = False,
             steering_pressed: bool = False) -> float:
    """回傳要疊加到 desired_curvature 的修正量 (1/m)"""
    self._read_params()
    self._filter.dt = dt

    # 車速遲滯開關（km/h）：>=40 開啟、<=30 關閉，30~40 之間維持前一狀態
    v_ego_kph = v_ego / KPH_TO_MS
    if v_ego_kph >= SPEED_ON_KPH:
      self._speed_gate = True
    elif v_ego_kph <= SPEED_OFF_KPH:
      self._speed_gate = False

    # 急彎/路口轉彎判斷：只要 model 本身的曲率已經很大（半徑很小，例如路口右轉），
    # 就直接停用 LCC，退回原本 model 控制，不強行拉回車道中心。
    # 一般高速公路/山路彎道曲率遠小於此門檻，會照樣做居中修正、不會因為有彎度就關掉。
    model_curvature = getattr(model_v2.action, "desiredCurvature", 0.0) if lat_active else 0.0
    is_sharp_turn = abs(model_curvature) > SHARP_TURN_CURVATURE

    hard_invalid = (
      not self._enabled or
      not lat_active or
      not self._speed_gate or
      is_sharp_turn
    )
    if hard_invalid:
      if not self._enabled:
        _state = "DISABLED"
      elif not lat_active:
        _state = "NOT_LAT_ACTIVE"
      elif not self._speed_gate:
        _state = "SPEED_GATE_OFF"
      else:
        _state = "SHARP_TURN"
      self._log_row(_state, dt=dt, v_kph=v_ego_kph, lat_active=lat_active,
                    speed_gate=self._speed_gate, sharp_turn=is_sharp_turn,
                    model_curvature=model_curvature,
                    yield_hold_timer=self._yield_hold_timer,
                    engage_ramp_timer=self._engage_ramp_timer,
                    ramp_factor=0.0, weight=0.0, lane_center_error=0.0, error_rate=0.0,
                    p_term=0.0, d_term=0.0, raw_correction=0.0, correction=0.0)
      self.reset()
      return 0.0

    # 啟用漸強：只有真正「剛從無效狀態轉為有效狀態」才會從 0 開始爬升（因為只有
    # hard_invalid 觸發的 reset() 才會把這個計時器歸零）。方向燈讓開、信心度短暫
    # 偏低等情況不會重置這個計時器，恢復時修正量會直接接回原本力道，不會重新爬升。
    self._engage_ramp_timer += dt
    ramp_factor = float(np.clip(self._engage_ramp_timer / ENGAGE_RAMP_SEC, 0.0, 1.0))

    # 軟性讓開條件：只有「方向燈亮著 + 駕駛手上有出力」持續超過 YIELD_CONFIRM_SEC
    # 才真正判定為讓開，濾掉方向燈剛打瞬間、手還沒真的出力的雜訊/短暫誤觸。
    # 沒打方向燈的話，即使駕駛手上有出力，LCC 仍照常運作（維持「有點黏著」的居中行為），
    # 不影響車速遲滯狀態，純粹是修正量本身淡出到 0。
    raw_yield_condition = (left_blinker or right_blinker) and steering_pressed
    if raw_yield_condition:
      self._yield_hold_timer += dt
    else:
      self._yield_hold_timer = 0.0
    is_yielding = self._yield_hold_timer >= YIELD_CONFIRM_SEC

    if is_yielding:
      self._active = False
      self.weight = 0.0
      self.lane_center_error = 0.0
      self._prev_lane_center_error = 0.0
      self._prev_error_valid = False
      self.correction = self._filter.update(0.0)
      self._log_row("YIELDING", dt=dt, v_kph=v_ego_kph, lat_active=lat_active,
                    speed_gate=self._speed_gate, sharp_turn=is_sharp_turn,
                    model_curvature=model_curvature,
                    yield_hold_timer=self._yield_hold_timer,
                    engage_ramp_timer=self._engage_ramp_timer,
                    ramp_factor=ramp_factor, weight=self.weight,
                    lane_center_error=self.lane_center_error, error_rate=0.0,
                    p_term=0.0, d_term=0.0, raw_correction=0.0, correction=self.correction)
      return self.correction

    lane_lines = model_v2.laneLines
    lane_line_probs = model_v2.laneLineProbs

    if len(lane_lines) < 3 or len(lane_line_probs) < 3:
      self._log_row("NO_LANE_DATA", dt=dt, v_kph=v_ego_kph, lat_active=lat_active,
                    speed_gate=self._speed_gate, sharp_turn=is_sharp_turn,
                    model_curvature=model_curvature,
                    yield_hold_timer=self._yield_hold_timer,
                    engage_ramp_timer=self._engage_ramp_timer,
                    ramp_factor=ramp_factor, weight=0.0, lane_center_error=0.0, error_rate=0.0,
                    p_term=0.0, d_term=0.0, raw_correction=0.0, correction=0.0)
      self.reset()
      return 0.0

    lll_prob = lane_line_probs[1]
    rll_prob = lane_line_probs[2]
    min_prob = min(lll_prob, rll_prob)

    # 信心不足，淡出到 0（不是硬切，靠低通濾波自然收斂）
    self.weight = float(np.clip((min_prob - PROB_MIN) / (PROB_FULL - PROB_MIN), 0.0, 1.0))
    if self.weight <= 0.0:
      self._active = False
      self.correction = self._filter.update(0.0)
      self.lane_center_error = 0.0
      self._prev_lane_center_error = 0.0
      self._prev_error_valid = False
      self._log_row("LOW_CONFIDENCE", dt=dt, v_kph=v_ego_kph, lat_active=lat_active,
                    speed_gate=self._speed_gate, sharp_turn=is_sharp_turn,
                    model_curvature=model_curvature,
                    yield_hold_timer=self._yield_hold_timer,
                    engage_ramp_timer=self._engage_ramp_timer,
                    ramp_factor=ramp_factor, weight=self.weight, lane_center_error=0.0,
                    error_rate=0.0, p_term=0.0, d_term=0.0, raw_correction=0.0,
                    correction=self.correction)
      return self.correction

    lll = lane_lines[1]
    rll = lane_lines[2]
    lll_y = _clip_interp(LOOKAHEAD_DIST_M, lll.x, lll.y)
    rll_y = _clip_interp(LOOKAHEAD_DIST_M, rll.x, rll.y)

    if lll_y is None or rll_y is None:
      self._active = False
      self.correction = self._filter.update(0.0)
      self.lane_center_error = 0.0
      self._prev_lane_center_error = 0.0
      self._prev_error_valid = False
      self._log_row("INTERP_FAIL", dt=dt, v_kph=v_ego_kph, lat_active=lat_active,
                    speed_gate=self._speed_gate, sharp_turn=is_sharp_turn,
                    model_curvature=model_curvature,
                    yield_hold_timer=self._yield_hold_timer,
                    engage_ramp_timer=self._engage_ramp_timer,
                    ramp_factor=ramp_factor, weight=self.weight, lane_center_error=0.0,
                    error_rate=0.0, p_term=0.0, d_term=0.0, raw_correction=0.0,
                    correction=self.correction)
      return self.correction

    # device frame：y 正值為左側。lane_center_error > 0 代表車道中心在車輛左邊，
    # 需要向左修正（正曲率）
    self.lane_center_error = (lll_y + rll_y) / 2.0

    # PD 控制：P 項是 pure-pursuit 近似（曲率 ≈ 2y/L²）；D 項用誤差變化率抑制過衝，
    # 誤差變化太快（代表車輛正在快速接近或衝過中心）時，D 項會反向抵銷一部分修正量，
    # 避免像純 P 控制那樣持續過衝、來回振盪。
    # 剛啟用/剛從讓開或低信心狀態恢復的第一幀，沒有真正有效的「上一幀誤差」可比較，
    # 這種情況 D 項直接視為 0，不能拿歸零的假基準去算變化率（否則會爆出虛假尖峰）。
    if self._prev_error_valid:
      error_rate = (self.lane_center_error - self._prev_lane_center_error) / dt if dt > 1e-3 else 0.0
    else:
      error_rate = 0.0
      self._prev_error_valid = True
    self._prev_lane_center_error = self.lane_center_error

    p_term = 2.0 * self.lane_center_error / (LOOKAHEAD_DIST_M ** 2)
    d_term = KD_GAIN * 2.0 * error_rate / (LOOKAHEAD_DIST_M ** 2)
    raw_correction = p_term - d_term
    raw_correction = float(np.clip(raw_correction, -MAX_CORRECTION, MAX_CORRECTION))
    raw_correction *= self.weight * ramp_factor

    self.correction = self._filter.update(raw_correction)
    self._active = True
    self._log_row("ACTIVE", dt=dt, v_kph=v_ego_kph, lat_active=lat_active,
                  speed_gate=self._speed_gate, sharp_turn=is_sharp_turn,
                  model_curvature=model_curvature,
                  yield_hold_timer=self._yield_hold_timer,
                  engage_ramp_timer=self._engage_ramp_timer,
                  ramp_factor=ramp_factor, weight=self.weight,
                  lane_center_error=self.lane_center_error, error_rate=error_rate,
                  p_term=p_term, d_term=d_term, raw_correction=raw_correction,
                  correction=self.correction)
    return self.correction
