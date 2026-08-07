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

import time

import numpy as np

from openpilot.common.params import Params
from openpilot.common.filter_simple import FirstOrderFilter

PARAM_REFRESH_SEC = 2.0

# --- 系統內建參數（不從 Params 讀取，先求穩，之後有需要再開放調參） ---
MIN_SPEED_MS = 3.0          # 太低速時車道線幾何雜訊大，不修正
LOOKAHEAD_DIST_M = 15.0     # 用來算車道中心 offset 的前視距離
FILTER_RC_SEC = 0.5         # 修正量一階低通時間常數，避免抖動

# 車道線信心度：低於 PROB_MIN 直接不修正；高於 PROB_FULL 才給滿權重；
# 中間線性淡入淡出，避免 on/off 硬切
PROB_MIN = 0.3
PROB_FULL = 0.6

# --- 可透過 Params 調整的參數的預設值 ---
DEFAULT_MAX_CORRECTION = 0.005  # 修正量曲率上限 (1/m)，刻意抓很保守的值


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
    self._max_correction = DEFAULT_MAX_CORRECTION

    # --- 狀態 ---
    self._filter = FirstOrderFilter(0.0, FILTER_RC_SEC, 0.01)
    self.correction = 0.0
    self.lane_center_error = 0.0
    self.weight = 0.0
    self._active = False

  def _read_params(self) -> None:
    now = time.monotonic()
    if now - self._last_params_read < PARAM_REFRESH_SEC:
      return
    self._last_params_read = now
    self._enabled = self._params.get_bool("dp_lcc_enabled")
    self._max_correction = self._get_float("dp_lcc_max_correction", DEFAULT_MAX_CORRECTION)

  def _get_float(self, key: str, default: float) -> float:
    try:
      val = self._params.get(key)
      return float(val) if val is not None else default
    except Exception:
      return default

  def reset(self) -> None:
    self._filter.x = 0.0
    self.correction = 0.0
    self.lane_center_error = 0.0
    self.weight = 0.0
    self._active = False

  def update(self, model_v2, v_ego: float, lat_active: bool, dt: float) -> float:
    """回傳要疊加到 desired_curvature 的修正量 (1/m)"""
    self._read_params()
    self._filter.dt = dt

    is_invalid_condition = (
      not self._enabled or
      not lat_active or
      v_ego < MIN_SPEED_MS
    )

    lane_lines = model_v2.laneLines
    lane_line_probs = model_v2.laneLineProbs

    if is_invalid_condition or len(lane_lines) < 3 or len(lane_line_probs) < 3:
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
      return self.correction

    lll = lane_lines[1]
    rll = lane_lines[2]
    lll_y = _clip_interp(LOOKAHEAD_DIST_M, lll.x, lll.y)
    rll_y = _clip_interp(LOOKAHEAD_DIST_M, rll.x, rll.y)

    if lll_y is None or rll_y is None:
      self._active = False
      self.correction = self._filter.update(0.0)
      self.lane_center_error = 0.0
      return self.correction

    # device frame：y 正值為左側。lane_center_error > 0 代表車道中心在車輛左邊，
    # 需要向左修正（正曲率）
    self.lane_center_error = (lll_y + rll_y) / 2.0

    # pure-pursuit 近似：曲率 ≈ 2y / L^2
    raw_correction = 2.0 * self.lane_center_error / (LOOKAHEAD_DIST_M ** 2)
    raw_correction = float(np.clip(raw_correction, -self._max_correction, self._max_correction))
    raw_correction *= self.weight

    self.correction = self._filter.update(raw_correction)
    self._active = True
    return self.correction
