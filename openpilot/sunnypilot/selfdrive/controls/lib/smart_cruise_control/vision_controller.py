"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V

VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.VisionState

ACTIVE_STATES = (VisionState.entering, VisionState.turning, VisionState.leaving)
ENABLED_STATES = (VisionState.enabled, VisionState.overriding, *ACTIVE_STATES)

# ★ 2026-10-03 实测：本 build 的 capnp 枚举 `str()` 得到的是**数字**（'0'..'5'），
#   不是名字 ⇒ 直接插进日志会变成不可读的裸数字。探针统一走这张表翻译。
#   ★ 只用相等比较，**不调用 `int()`**（不同 build 行为不一致，可能抛异常）。
_STATE_NAMES = {
  'disabled': VisionState.disabled,
  'enabled': VisionState.enabled,
  'overriding': VisionState.overriding,
  'entering': VisionState.entering,
  'turning': VisionState.turning,
  'leaving': VisionState.leaving,
}


def _state_name(state) -> str:
  for name, value in _STATE_NAMES.items():
    if state == value:
      return name
  return f'unknown({state})'

_ENTERING_PRED_LAT_ACC_TH = 1.3  # Predicted Lat Acc threshold to trigger entering turn state.
_ABORT_ENTERING_PRED_LAT_ACC_TH = 1.1  # Predicted Lat Acc threshold to abort entering state if speed drops.

_TURNING_LAT_ACC_TH = 1.6  # Lat Acc threshold to trigger turning state.

_LEAVING_LAT_ACC_TH = 1.3  # Lat Acc threshold to trigger leaving turn state.
_FINISH_LAT_ACC_TH = 1.1  # Lat Acc threshold to trigger the end of the turn cycle.

_A_LAT_REG_MAX = 2.  # Maximum lateral acceleration

_NO_OVERSHOOT_TIME_HORIZON = 4.  # s. Time to use for velocity desired based on a_target when not overshooting.

# Lookup table for the minimum smooth deceleration during the ENTERING state
# depending on the actual maximum absolute lateral acceleration predicted on the turn ahead.
_ENTERING_SMOOTH_DECEL_V = [-0.2, -1.]  # min decel value allowed on ENTERING state
_ENTERING_SMOOTH_DECEL_BP = [1.3, 3.]  # absolute value of lat acc ahead

# Lookup table for the acceleration for the TURNING state
# depending on the current lateral acceleration of the vehicle.
_TURNING_ACC_V = [0.5, 0., -0.4]  # acc value
_TURNING_ACC_BP = [1.5, 2.3, 3.]  # absolute value of current lat acc

_LEAVING_ACC = 0.5  # Conformable acceleration to regain speed while leaving a turn.

# ── 可热调参数（2026-10-03 新增）──────────────────────────────────────────────
# 为什么需要这两个 Param（先读这段再改数）：
#
# 1) `_A_LAT_REG_MAX` 才是 SCC-V **真正**的介入门槛，不是 `_ENTERING_PRED_LAT_ACC_TH`。
#    生效条件是 `output_v_target < v_ego`，即
#        v_ego·√(A_LAT/max_pred) + _NO_OVERSHOOT_TIME_HORIZON·a_target < v_ego
#    当 `max_pred ≤ A_LAT` 时 `√(A_LAT/max_pred) ≥ 1`，而 `a_target` 被
#    `_ENTERING_SMOOTH_DECEL_V` 的下界钳在 −0.2 ⇒ 上式需要
#    `v_ego·(1 − √(A_LAT/max_pred)) > 0.8`，实际不可达。
#    ⇒ **单独调 `_ENTERING_PRED_LAT_ACC_TH` 只会让状态机更常报 entering，车零动作**
#      （已用闭环仿真逐位验证：1.3 与 1.0 结果完全相同）。
#    调 A_LAT 同时改变两件事：
#      · 多早开始压 v_cruise —— `v_trigger ≈ 3.6·√(A_LAT·R)` km/h
#      · 弯中目标速度       —— `v_target = √(A_LAT·R)`
#    ⚠ 只有当 A_LAT 被调到 < `_ENTERING_PRED_LAT_ACC_TH`(1.3) 时，那个门槛才会
#      变成新的瓶颈（届时需同步下调它，并保持 abort < entering）。
#
# 2) `DecelBoost` 按比例加深 ENTERING / TURNING 阶段的减速度。
#    ⚠ 只作用于**负值**（减速度）；正值原样返回 ⇒ 不会让车在弯里更冲（安全性单向）。
#    ⚠ 峰值减速**仍受主 planner 的 `A_CRUISE_MIN = -1.2` 硬钳位限制**
#      （见 `selfdrive/controls/lib/longitudinal_planner.py:25`，且 `j_cruise` 还会
#       限制爬升速率）⇒ 本参数只能让它"更快达到 −1.2 并维持更久"，不能突破 −1.2。
#
# ★ 一键回退：`SCCVisionMaxLatAcc = 2.0` + `SCCVisionDecelBoost = 1.0`
#   ⇒ 与 2026-10-03 之前**逐位相同**。
PARAM_MAX_LAT_ACC = "SCCVisionMaxLatAcc"
PARAM_DECEL_BOOST = "SCCVisionDecelBoost"

MAX_LAT_ACC_DEFAULT = 1.6   # 上游原值 2.0。1.6：R=200 m 的弯从 66.5 → 60.8 km/h 就开始介入
MAX_LAT_ACC_MIN = 1.0
MAX_LAT_ACC_MAX = 3.0

DECEL_BOOST_DEFAULT = 1.0   # 1.0 = 上游原样；>1 加深介入段减速度（只作用于负值）
DECEL_BOOST_MIN = 1.0
DECEL_BOOST_MAX = 2.5


def _read_float(params, key: str, default: float) -> float:
  """降级读取 FLOAT 参数。

  声明了默认值的 FLOAT 键，`get(return_default=True)` 一定返回 float（文件不存在时
  params_get 返回 b''，回落声明默认值再转 float）。任何异常一律回落 default，
  绝不向上抛 —— 本函数跑在 plannerd 关键路径上。
  """
  try:
    value = params.get(key, return_default=True)
  except Exception:
    return default
  try:
    return float(default if value is None else value)
  except (TypeError, ValueError):
    return default


def _apply_decel_boost(a_target: float, boost: float) -> float:
  """只把**负的**（减速）目标按 boost 加深，正值原样返回。

  ⇒ boost 永远不会放大正向加速度 ⇒ 不可能让车在弯里更冲（安全性单向）。
  """
  return a_target * boost if a_target < 0.0 else a_target


class SmartCruiseControlVision:
  v_target: float = 0
  a_target: float = 0.
  v_ego: float = 0.
  a_ego: float = 0.
  output_v_target: float = V_CRUISE_UNSET
  output_a_target: float = 0.

  def __init__(self):
    self.params = Params()
    self.frame = -1
    self.long_enabled = False
    self.long_override = False
    self.is_enabled = False
    self.is_active = False
    self.enabled = self.params.get_bool("SmartCruiseControlVision")
    self.v_cruise_setpoint = 0.

    self.state = VisionState.disabled
    self.current_lat_acc = 0.
    self.max_pred_lat_acc = 0.

    # 可热调参数（1 Hz 轮询，见 _update_params）
    self.max_lat_acc = float(np.clip(_read_float(self.params, PARAM_MAX_LAT_ACC, MAX_LAT_ACC_DEFAULT),
                                     MAX_LAT_ACC_MIN, MAX_LAT_ACC_MAX))
    self.decel_boost = float(np.clip(_read_float(self.params, PARAM_DECEL_BOOST, DECEL_BOOST_DEFAULT),
                                     DECEL_BOOST_MIN, DECEL_BOOST_MAX))

    # [SCCVision] 探针状态（纯留痕；变化检测只放离散量 —— 见 _probe）
    self._pv_t = 0.0
    self._pv_sig = None

  def get_a_target_from_control(self) -> float:
    return self.a_target

  def get_v_target_from_control(self) -> float:
    if self.is_active:
      return max(self.v_target, MIN_V) + self.a_target * _NO_OVERSHOOT_TIME_HORIZON

    return V_CRUISE_UNSET

  def _update_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enabled = self.params.get_bool("SmartCruiseControlVision")
      # ★ 1 Hz 热生效：改完 Param 约 1 秒后被采纳，不需要重启
      self.max_lat_acc = float(np.clip(_read_float(self.params, PARAM_MAX_LAT_ACC, MAX_LAT_ACC_DEFAULT),
                                       MAX_LAT_ACC_MIN, MAX_LAT_ACC_MAX))
      self.decel_boost = float(np.clip(_read_float(self.params, PARAM_DECEL_BOOST, DECEL_BOOST_DEFAULT),
                                       DECEL_BOOST_MIN, DECEL_BOOST_MAX))

  def _probe(self) -> None:
    """[SCCVision] 探针（2026-10-03）—— 纯留痕，不参与任何控制判断。

    为什么必须有：A_LAT 与 boost 都是可热调参数，**必须能从日志反推"当时用的是多少"**，
    否则实车调参过程无法复盘（红灯门槛 p18 的教训：加了 `rel=` 字段才能还原用户调过哪些值）。

    字段：state / maxLat=弯道横向加速度预算 / boost=减速增强倍率 /
          vEgo(km/h) / vTgt=sccV 目标速度(km/h，-1=不适用) / aTgt / out(最终输出，km/h)
    触发：① (state, maxLat, boost, enabled) 变化当帧必打 ② 否则 1 Hz

    ★ 探针铁律：变化检测**只放离散量**（state 名字 / 参数值 / 布尔）。绝不把
      vTgt/aTgt/out 这类连续量放进签名 —— 那会把节流 100% 击穿（每帧一条）。
    ★ 整段 try/except + cloudlog 惰性 import：不许影响控制输出。
    ★ 枚举只走 `_state_name()`（本 build 的 `str(枚举)` 是数字，不可直接插日志）。
    """
    try:
      sig = (self.state, self.max_lat_acc, self.decel_boost, self.enabled)
      self._pv_t += DT_MDL
      if sig != self._pv_sig or self._pv_t >= 1.0:
        self._pv_t = 0.0
        self._pv_sig = sig
        from openpilot.common.swaglog import cloudlog
        # v_target 在直道 = (A_LAT/0)**0.5 = inf、未启用时 = 0 ⇒ 两种都记 -1
        vt_kph = self.v_target * 3.6
        vt_kph = vt_kph if 0.0 < vt_kph <= 999.0 else -1.0
        out_kph = (self.output_v_target * 3.6) if self.is_active else -1.0
        cloudlog.info(f"[SCCVision] {_state_name(self.state)} maxLat={self.max_lat_acc:.2f} "
                      f"boost={self.decel_boost:.2f} vEgo={self.v_ego * 3.6:.1f} vTgt={vt_kph:.1f} "
                      f"aTgt={float(self.a_target):+.2f} out={out_kph:.1f}")
    except Exception:
      pass

  def _update_calculations(self, sm: messaging.SubMaster) -> None:
    if not self.long_enabled:
      return
    else:
      rate_plan = np.array(np.abs(sm['modelV2'].orientationRate.z))
      vel_plan = np.array(sm['modelV2'].velocity.x)

      self.current_lat_acc = self.v_ego ** 2 * abs(sm['controlsState'].curvature)

      # get the maximum lat accel from the model
      predicted_lat_accels = rate_plan * vel_plan
      self.max_pred_lat_acc = np.percentile(predicted_lat_accels, 97)

      # get the maximum curve based on the current velocity
      v_ego = max(self.v_ego, 0.1)  # ensure a value greater than 0 for calculations
      max_curve = self.max_pred_lat_acc / (v_ego**2)

      # Get the target velocity for the maximum curve
      # ★ 2026-10-03：常量 `_A_LAT_REG_MAX` 换成可热调的 `self.max_lat_acc`。
      #   这是 SCC-V 真正的介入门槛（详见 PARAM_MAX_LAT_ACC 处的推导）。
      self.v_target = (self.max_lat_acc / max_curve) ** 0.5

  def _update_state_machine(self) -> tuple[bool, bool]:
    # ENABLED, ENTERING, TURNING, LEAVING, OVERRIDING
    if self.state != VisionState.disabled:
      # longitudinal and feature disable always have priority in a non-disabled state
      if not self.long_enabled or not self.enabled:
        self.state = VisionState.disabled
      elif self.long_override:
        self.state = VisionState.overriding

      else:
        # ENABLED
        if self.state == VisionState.enabled:
          # Do not enter a turn control cycle if the speed is low.
          if self.v_ego <= MIN_V:
            pass
          # If significant lateral acceleration is predicted ahead, then move to Entering turn state.
          elif self.max_pred_lat_acc >= _ENTERING_PRED_LAT_ACC_TH:
            self.state = VisionState.entering

        # OVERRIDING
        elif self.state == VisionState.overriding:
          if not self.long_override:
            self.state = VisionState.enabled

        # ENTERING
        elif self.state == VisionState.entering:
          # Transition to Turning if current lateral acceleration is over the threshold.
          if self.current_lat_acc >= _TURNING_LAT_ACC_TH:
            self.state = VisionState.turning
          # Abort if the predicted lateral acceleration drops
          elif self.max_pred_lat_acc < _ABORT_ENTERING_PRED_LAT_ACC_TH:
            self.state = VisionState.enabled

        # TURNING
        elif self.state == VisionState.turning:
          # Transition to Leaving if current lateral acceleration drops below a threshold.
          if self.current_lat_acc <= _LEAVING_LAT_ACC_TH:
            self.state = VisionState.leaving

        # LEAVING
        elif self.state == VisionState.leaving:
          # Transition back to Turning if current lateral acceleration goes back over the threshold.
          if self.current_lat_acc >= _TURNING_LAT_ACC_TH:
            self.state = VisionState.turning
          # Finish if current lateral acceleration goes below a threshold.
          elif self.current_lat_acc < _FINISH_LAT_ACC_TH:
            self.state = VisionState.enabled

    # DISABLED
    elif self.state == VisionState.disabled:
      if self.long_enabled and self.enabled:
        if self.long_override:
          self.state = VisionState.overriding
        else:
          self.state = VisionState.enabled

    enabled = self.state in ENABLED_STATES
    active = self.state in ACTIVE_STATES

    return enabled, active

  def _update_solution(self) -> float:
    # DISABLED, ENABLED, OVERRIDING
    if self.state not in ACTIVE_STATES:
      # when not overshooting, calculate v_turn as the speed at the prediction horizon when following
      # the smooth deceleration.
      a_target = self.a_ego
    # ENTERING
    elif self.state == VisionState.entering:
      # when not overshooting, target a smooth deceleration in preparation for a sharp turn to come.
      # ★ 2026-10-03：查表结果再过 `_apply_decel_boost`（只加深负值，见其 docstring）。
      a_target = _apply_decel_boost(
        np.interp(self.max_pred_lat_acc, _ENTERING_SMOOTH_DECEL_BP, _ENTERING_SMOOTH_DECEL_V), self.decel_boost)
    # TURNING
    elif self.state == VisionState.turning:
      # When turning, we provide a target acceleration that is comfortable for the lateral acceleration felt.
      a_target = _apply_decel_boost(
        np.interp(self.current_lat_acc, _TURNING_ACC_BP, _TURNING_ACC_V), self.decel_boost)
    # LEAVING
    elif self.state == VisionState.leaving:
      # When leaving, we provide a comfortable acceleration to regain speed.
      a_target = _LEAVING_ACC
    else:
      raise NotImplementedError(f"SCC-V state not supported: {self.state}")

    return a_target

  def update(self, sm: messaging.SubMaster, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float,
             v_cruise_setpoint: float) -> None:
    self.long_enabled = long_enabled
    self.long_override = long_override
    self.v_ego = v_ego
    self.a_ego = a_ego
    self.v_cruise_setpoint = v_cruise_setpoint

    self._update_params()
    self._update_calculations(sm)

    self.is_enabled, self.is_active = self._update_state_machine()
    self.a_target = self._update_solution()

    self.output_v_target = self.get_v_target_from_control()
    self.output_a_target = self.get_a_target_from_control()

    self._probe()

    self.frame += 1
