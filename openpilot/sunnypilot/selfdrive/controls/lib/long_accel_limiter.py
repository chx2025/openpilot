"""输出端纵向 jerk 限制（comfort limiter）。

## 为什么需要它（2026-09-22 实车取证）

`longitudinal_planner` 的最终 `a_target` 是**多条来源取 min()** 的结果：

  candidates = [MPC, cruise(, traffic_stop)(, e2e)]  ->  min()
  再叠加 turn_decel 覆盖 / coast_resume 抬升 / block_accel 归零

上游 MPC 本身输出是连续的（它自带 jerk 约束），但 `min()` 往里面塞进来的
**其他每一路都是不连续的**：布尔门控一帧开一帧关、turn_decel 在方向盘过 15°
时把减速度**一步清零**、coast_resume 抬升时也是阶跃。`min()` 只挑最小值，
**不负责连续性** —— 于是任何一路的跳变都会原样成为 `a_target` 的跳变。

而下游 `LongControl`（`selfdrive/controls/lib/longcontrol.py`）是
**纯 PID + 前馈，没有任何 jerk / 速率限制**（已核对设备源码，全文件无
`jerk` / `RateLimiter` 字样）。所以 `a_target` 的跳变几乎 1:1 传到执行器。

实车实测（2026-09-22 10:22~10:29，`[E2eTrace]` 10 Hz，1598 个相邻样本）：

  中位 jerk            0.05 m/s³   ← 大部分时间其实是平滑的
  jerk >= 5 m/s³ 的次数  6 次
  其中 4 次来自「无证据反闸」的进出（10.0 / 9.7 / 8.9 / 8.0）
  另 2 次来自 turn_decel：方向盘过 15° 松减速（7.2）、进入减速（5.9）

用户体感正是这些：巡航「有东西在打架」、过路口「拉扯感」。

## 判据：**非对称**限速（这一条决定了它为什么是安全的）

  · 往「加强制动」方向（a_target 变小）—— **放得很宽**（`LONG_JERK_BRAKE_M_S3`）。
    绝不因为"要平顺"而延迟制动。6.0 m/s³ 意味着 0.9 m/s² 的加深只要 0.15 s，
    相对制动本身的物理响应（执行器 + 车辆动力学，百毫秒级）可以忽略。
  · 往「放松制动 / 恢复加速」方向（a_target 变大）—— **限得紧**
    （`LONG_JERK_RELEASE_M_S3`）。「刹车被一步抽走」正是拉扯感的来源，
    也是唯一可以慢、且慢了更舒服的方向。
  · 请求进入**紧急制动区间**（<= `LONG_JERK_BYPASS_A`）—— 直接放行并同步状态，
    不做任何限制。FCW / 前车急刹 / traffic_stop 的强制动一律原样通过。

状态同步：带上 wall clock 时间戳，若上一帧距现在超过 `STALE_RESYNC_S`
（未接管一段、模块重启、掉帧）就直接对齐到当前请求值，避免拿着旧值做限制
而在再接管时"卡"一下。**用时间戳而不是外部 reset()**：planner 里
`reset_state` 的语义包含「carState.vCruise == V_CRUISE_UNSET」，该类条件在
实车上可能大面积成立，挂在它上面会导致限制器被逐帧清空而完全失效。

## 回退

`LONG_JERK_LIMIT_ENABLE = False` 一行即可完全恢复原行为（本模块变成直通）。
"""

from __future__ import annotations


# ==== 配置 ================================================================
# 一键回退：False -> update() 直接返回原值，行为与加本模块之前完全一致。
LONG_JERK_LIMIT_ENABLE: bool = True

# 往「放松制动 / 恢复加速」方向的最大 jerk（m/s²/s）。
# 2.0 是"比人开得略紧、但明显消除阶跃"的量级：实测里所有 >=5 m/s³ 的放松
# 事件都会被摊到 >=0.3 s。正常跟车/巡航的相邻帧变化远小于此（中位 0.05 m/s³），
# 所以日常几乎不会触发，只在真正的不连续处介入。
LONG_JERK_RELEASE_M_S3: float = 2.0

# 往「加强制动」方向的最大 jerk（m/s²/s）。
# 取得很宽：0.8 m/s² 的加深只需 0.1 s（约 2 帧@20Hz）。
# 目的是削掉"抓一下"的观感，而不是限制制动能力本身。
# 定 8.0 的依据：回放实测（本趟 1598 对相邻样本）最深制动请求 -1.57 时
# 最大"欠刹"仅 0.802 m/s² -> 折合延迟 100 ms，相对执行器本身的响应时间
# 可忽略；同时 >=8 m/s³ 的制动阶跃在本趟为 0 次，说明它基本不介入日常驾驶。
LONG_JERK_BRAKE_M_S3: float = 8.0

# 请求低于此值时视为紧急制动，直接放行（不做任何限制）。
# 与 ACCEL_MIN(-3.5) 同量级：进入这个区间说明是在真刹，不是舒适性问题。
LONG_JERK_BYPASS_A: float = -2.5

# 距上一帧超过该时长则重新对齐（未接管 / 掉帧），避免用旧状态做限制。
STALE_RESYNC_S: float = 0.5


class LongAccelLimiter:
  """无外部 reset 依赖：靠 `t` 自动判断状态是否过期。"""

  def __init__(self) -> None:
    self._prev: float | None = None
    self._t: float | None = None

  def reset(self) -> None:
    """外部强制对齐（可选；正常靠 `t` 的过期判断即可）。"""
    self._prev = None
    self._t = None

  def update(self, a_target: float, *, t: float, dt: float) -> float:
    """把不连续的请求 `a_target` 限成连续输出。

    Args:
      a_target: 上游（candidates min + 各覆盖）给出的期望加速度。
      t: 当前 wall clock（秒），用于过期判断。
      dt: 期望帧间隔（秒），通常 0.05。
    """
    if not LONG_JERK_LIMIT_ENABLE:
      return a_target

    if dt <= 0.0 or self._prev is None or self._t is None or (t - self._t) > STALE_RESYNC_S:
      self._prev, self._t = a_target, t
      return a_target

    self._t = t

    # 紧急制动：原样放行，并把状态同步到请求值（避免之后回弹时被旧的高值拖住）
    if a_target <= LONG_JERK_BYPASS_A:
      self._prev = a_target
      return a_target

    hi = self._prev + LONG_JERK_RELEASE_M_S3 * dt   # 放松方向的上限
    lo = self._prev - LONG_JERK_BRAKE_M_S3 * dt     # 加强制动的下限
    out = min(max(a_target, lo), hi)
    self._prev = out
    return out
