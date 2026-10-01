"""Tests for GasLongCancel: 踩油门挂起纵向 / 松油门立即恢复（本机 ResumeMs=0）。

Spec (user requirement, 2026-09-28):
  1. 踩油门 -> 取消纵向（把 CC.longActive 压成 False，不是"减小减速"）。
  2. 松开油门 -> 立即恢复纵向（本机配置 0 ms；延时可调 GasLongCancelResumeMs）。
  3. 如果纵向是被 刹车 / 按键 取消掉的，那么之后再踩油门、松油门
     都不能恢复纵向 —— 「踩油门挂起」和「真的取消」必须分清楚。

The controller is a stand-alone sunnypilot module; controlsd wires it as

    if self.gas_long_cancel.update(bool(CS.gasPressed), CC.enabled, CS.vEgo, CS.aEgo):
      CC.longActive = False

so these tests drive `update()` directly and never touch any real Params.
"""
from openpilot.common.realtime import DT_CTRL

from openpilot.sunnypilot.selfdrive.controls.lib import gas_long_cancel
from openpilot.sunnypilot.selfdrive.controls.lib.gas_long_cancel import (
  GAS_LONG_CANCEL_ENABLE, GAS_LONG_CANCEL_PARAM_KEY, GAS_LONG_CANCEL_RESUME_PARAM_KEY,
  GAS_LONG_CANCEL_RESUME_S, GAS_LONG_CANCEL_RESUME_S_MAX, GAS_LONG_CANCEL_RESUME_S_MIN,
  PHASE_HELD, PHASE_OFF, PHASE_READY, PHASE_WAIT, GasLongCancel,
)

RESUME_FRAMES = int(round(GAS_LONG_CANCEL_RESUME_S / DT_CTRL))   # 0.5 s @ 100 Hz = 50


class FakeParams:
  """Minimal stand-in for openpilot.common.params.Params (no C++ dependency)."""

  def __init__(self, values=None, raise_on=None):
    self.values = dict(values or {})
    self.raise_on = set(raise_on or ())

  def get(self, key, block=False, return_default=True):
    if key in self.raise_on:
      raise RuntimeError(f"fake param read failure: {key}")
    return self.values.get(key)


def mk(params=None, enabled=True) -> GasLongCancel:
  return GasLongCancel(params=params, enabled=enabled)


def force_param_refresh(c: GasLongCancel) -> None:
  """Bypass the 1 Hz gate so a param change is visible on the next frame."""
  c._param_t = -gas_long_cancel.GAS_LONG_CANCEL_PARAMS_PERIOD_S


# ---------------------------------------------------------------------------
# 1. 开关语义
# ---------------------------------------------------------------------------
def test_module_level_kill_switch_is_on():
  assert GAS_LONG_CANCEL_ENABLE is True


def test_disabled_by_param_never_suspends():
  c = mk(enabled=False)
  for _ in range(200):
    assert c.update(gas_pressed=True, engaged=True) is False
    assert c.update(gas_pressed=False, engaged=True) is False
  assert c.phase == PHASE_OFF
  assert c.suspended is False


def test_disabled_by_code_switch_never_suspends():
  saved = gas_long_cancel.GAS_LONG_CANCEL_ENABLE
  try:
    gas_long_cancel.GAS_LONG_CANCEL_ENABLE = False
    c = mk(enabled=True)
    for _ in range(50):
      assert c.update(gas_pressed=True, engaged=True) is False
  finally:
    gas_long_cancel.GAS_LONG_CANCEL_ENABLE = saved


def test_never_suspends_while_not_engaged():
  c = mk()
  for _ in range(50):
    assert c.update(gas_pressed=True, engaged=False) is False
  assert c.phase == PHASE_OFF


# ---------------------------------------------------------------------------
# 2. 需求 1：踩油门 -> 立刻挂起
# ---------------------------------------------------------------------------
def test_gas_press_suspends_on_the_very_frame():
  c = mk()
  assert c.update(gas_pressed=False, engaged=True) is False
  assert c.update(gas_pressed=True, engaged=True) is True
  assert c.phase == PHASE_HELD
  assert c.armed is True
  assert c.n_suspend == 1


def test_gas_held_stays_suspended_for_the_whole_hold():
  c = mk()
  c.update(gas_pressed=False, engaged=True)
  c.update(gas_pressed=True, engaged=True)
  for _ in range(300):                       # 3 s 持续踩着
    assert c.update(gas_pressed=True, engaged=True) is True
  assert c.phase == PHASE_HELD


def test_only_rising_edge_arms():
  """engage 的那一刻脚已经压在油门上 -> 不武装，纵向立刻正常接管。

  ★ 必须先喂「未 engaged 但脚已踩着」的帧：模块每帧都被调用（含未 engaged 时），
    prev_gas 始终是真实踏板状态，所以未 engaged 期间踩下的油门**不算上升沿**。
    （第一版用例漏了这一步，把模块的正确行为误判成了 bug。）
  """
  c = mk()
  for _ in range(20):
    assert c.update(gas_pressed=True, engaged=False) is False   # 未 engage，脚先踩下去
  for _ in range(20):
    assert c.update(gas_pressed=True, engaged=True) is False    # 再 engage -> 仍不武装
  assert c.armed is False
  assert c.phase == PHASE_READY


# ---------------------------------------------------------------------------
# 3. 需求 2：松油门 -> 恰好 0.5 s（= RESUME_FRAMES 帧）后恢复
# ---------------------------------------------------------------------------
def test_resume_is_exactly_resume_frames_after_release():
  c = mk()
  c.update(gas_pressed=False, engaged=True)
  c.update(gas_pressed=True, engaged=True)          # 踩下
  c.update(gas_pressed=True, engaged=True)          # 保持 1 帧

  # 松油门后的前 RESUME_FRAMES 帧仍然挂起
  for i in range(RESUME_FRAMES):
    assert c.update(gas_pressed=False, engaged=True) is True, f"frame {i} 应仍挂起"
  assert c.phase == PHASE_WAIT
  assert c.timer == 0

  # 第 RESUME_FRAMES + 1 帧恢复
  assert c.update(gas_pressed=False, engaged=True) is False
  assert c.phase == PHASE_READY
  assert c.armed is False
  assert c.suspended is False
  assert c.n_resume == 1


def test_timer_is_reset_while_gas_is_held():
  """长时间踩着油门，松手后仍然要等满 0.5 s（计时只在松油门后走）。"""
  c = mk()
  c.update(gas_pressed=False, engaged=True)
  c.update(gas_pressed=True, engaged=True)
  for _ in range(500):
    c.update(gas_pressed=True, engaged=True)
  assert c.timer == RESUME_FRAMES
  for _ in range(RESUME_FRAMES):
    assert c.update(gas_pressed=False, engaged=True) is True
  assert c.update(gas_pressed=False, engaged=True) is False


def test_zero_resume_delay_releases_immediately():
  params = FakeParams({GAS_LONG_CANCEL_PARAM_KEY: True, GAS_LONG_CANCEL_RESUME_PARAM_KEY: 0})
  c = mk(params=params)
  c.update(gas_pressed=False, engaged=True)     # 参数在首次 update 时才读进来
  assert c.resume_frames == 0
  assert c.update(gas_pressed=True, engaged=True) is True
  assert c.update(gas_pressed=False, engaged=True) is False
  # ★ 0 ms = 无空窗期：松油门当帧就交回纵向，相位 held -> ready 直跳，绝不出现 wait
  assert c.phase == PHASE_READY
  assert c.n_resume == 1
  assert c.timer == 0


def test_zero_resume_never_enters_wait_phase():
  """★ 本机配置（GasLongCancelResumeMs=0）：松油门当帧接管，wait 相位永不出现。"""
  params = FakeParams({GAS_LONG_CANCEL_PARAM_KEY: True, GAS_LONG_CANCEL_RESUME_PARAM_KEY: 0})
  c = mk(params=params)
  c.update(gas_pressed=False, engaged=True)      # 首帧读参数
  seen = set()
  trues = 0
  for _ in range(3):                             # 踩 3 次
    for _ in range(5):                           # 按下那帧 + 持续踩住 4 帧 = 5 帧挂起
      trues += 1 if c.update(gas_pressed=True, engaged=True) else 0
      seen.add(c.phase)
    trues += 1 if c.update(gas_pressed=False, engaged=True) else 0   # ★ 松手当帧
    seen.add(c.phase)
  assert PHASE_WAIT not in seen, "0 ms 配置下不该出现 wait 相位（空窗期）"
  assert c.n_suspend == 3 and c.n_resume == 3
  assert c.phase == PHASE_READY
  assert trues == 3 * 5, "只有踩住那 5 帧该挂起，松手帧必须立刻交回"


def test_repeated_gas_taps_each_suspend():
  c = mk()
  c.update(gas_pressed=False, engaged=True)
  for tap in range(5):
    assert c.update(gas_pressed=True, engaged=True) is True, f"tap {tap}"
    assert c.update(gas_pressed=False, engaged=True) is True
  assert c.n_suspend == 5


# ---------------------------------------------------------------------------
# 4. 需求 3：刹车 / 按键取消的纵向，不许被"踩油门-松油门"复活
# ---------------------------------------------------------------------------
def test_brake_cancel_clears_arm_and_gas_cannot_resurrect():
  c = mk()
  c.update(gas_pressed=False, engaged=True)
  assert c.update(gas_pressed=True, engaged=True) is True     # 踩油门挂起

  # 刹车（或按键）取消 -> controlsd 侧 CC.enabled 变 False
  assert c.update(gas_pressed=True, engaged=False) is False
  assert c.armed is False
  assert c.n_disarm == 1

  # 之后无论怎么踩油门 / 松油门，只要没重新 engage，一律不挂起（也即"不恢复"）
  for _ in range(10):
    assert c.update(gas_pressed=False, engaged=False) is False
    assert c.update(gas_pressed=True, engaged=False) is False
    assert c.update(gas_pressed=True, engaged=False) is False
    assert c.update(gas_pressed=False, engaged=False) is False
  assert c.armed is False
  assert c.suspended is False
  assert c.phase == PHASE_OFF


def test_gas_press_release_before_engaging_does_not_arm():
  """先踩油门松油门（还没 engage），再 engage -> 不应挂起。"""
  c = mk()
  for _ in range(20):
    c.update(gas_pressed=True, engaged=False)
    c.update(gas_pressed=False, engaged=False)
  # 用户按 RESUME 重新 engage，脚已松开
  for _ in range(20):
    assert c.update(gas_pressed=False, engaged=True) is False
  assert c.phase == PHASE_READY


def test_cancel_while_gas_held_then_reengage_is_normal():
  """踩油门挂起中 -> 刹车取消（脚还踩着）-> 重新 engage（脚还踩着）-> 不应挂起。"""
  c = mk()
  c.update(gas_pressed=False, engaged=True)
  assert c.update(gas_pressed=True, engaged=True) is True
  assert c.update(gas_pressed=True, engaged=False) is False     # 刹车取消，脚仍踩着
  assert c.update(gas_pressed=True, engaged=False) is False
  for _ in range(20):                                           # 重新 engage，脚仍踩着
    assert c.update(gas_pressed=True, engaged=True) is False
  assert c.armed is False


def test_cancel_then_release_gas_then_reengage_then_gas_works():
  """取消 -> 松油门 -> 重新 engage -> 再踩油门：应恢复正常挂起能力。"""
  c = mk()
  c.update(gas_pressed=False, engaged=True)
  c.update(gas_pressed=True, engaged=True)
  c.update(gas_pressed=False, engaged=False)                    # 取消（同时松油门）
  for _ in range(20):
    assert c.update(gas_pressed=False, engaged=True) is False   # 重新 engage
  assert c.update(gas_pressed=True, engaged=True) is True       # 再踩油门 -> 重新挂起


def test_disable_while_suspended_releases_immediately():
  c = mk()
  c.update(gas_pressed=False, engaged=True)
  assert c.update(gas_pressed=True, engaged=True) is True
  assert c.update(gas_pressed=True, engaged=True) is True
  # 挂起途中拨开关到 0（1 Hz 轮询，强制刷新）
  c.enabled = False
  assert c.update(gas_pressed=True, engaged=True) is False
  assert c.suspended is False


# ---------------------------------------------------------------------------
# 5. 参数路径（1 Hz 热刷新；读取失败必须安全）
# ---------------------------------------------------------------------------
def test_param_enable_and_delay_are_applied():
  params = FakeParams({
    GAS_LONG_CANCEL_PARAM_KEY: True,
    GAS_LONG_CANCEL_RESUME_PARAM_KEY: 1000,
  })
  c = mk(params=params, enabled=False)      # 起始关，参数里是开
  c.update(gas_pressed=False, engaged=True)
  assert c.enabled is True
  assert c.resume_frames == int(round(1.0 / DT_CTRL))

  params.values[GAS_LONG_CANCEL_RESUME_PARAM_KEY] = 200
  force_param_refresh(c)
  c.update(gas_pressed=False, engaged=True)
  assert c.resume_frames == int(round(0.2 / DT_CTRL))


def test_param_delay_is_clamped():
  params = FakeParams({GAS_LONG_CANCEL_PARAM_KEY: True, GAS_LONG_CANCEL_RESUME_PARAM_KEY: 99999})
  c = mk(params=params)
  c.update(gas_pressed=False, engaged=True)
  assert c.resume_frames == int(round(GAS_LONG_CANCEL_RESUME_S_MAX / DT_CTRL))

  params.values[GAS_LONG_CANCEL_RESUME_PARAM_KEY] = -500
  force_param_refresh(c)
  c.update(gas_pressed=False, engaged=True)
  assert c.resume_frames == int(round(GAS_LONG_CANCEL_RESUME_S_MIN / DT_CTRL))

  params.values[GAS_LONG_CANCEL_RESUME_PARAM_KEY] = "not-a-number"
  force_param_refresh(c)
  c.update(gas_pressed=False, engaged=True)
  assert c.resume_frames == int(round(GAS_LONG_CANCEL_RESUME_S / DT_CTRL))   # 回落到默认


def test_param_read_failure_keeps_working():
  params = FakeParams({GAS_LONG_CANCEL_PARAM_KEY: True},
                      raise_on=[GAS_LONG_CANCEL_RESUME_PARAM_KEY])
  c = mk(params=params)
  c.update(gas_pressed=False, engaged=True)
  assert c.enabled is True
  assert c.update(gas_pressed=True, engaged=True) is True


def test_params_none_is_allowed():
  c = mk(params=None)
  c.update(gas_pressed=False, engaged=True)
  assert c.update(gas_pressed=True, engaged=True) is True


# ---------------------------------------------------------------------------
# 6. 探针：签名必须是离散量（不能每帧一条）
# ---------------------------------------------------------------------------
def test_probe_does_not_log_every_frame():
  """挂起期间 1 Hz 心跳 + 相位跳变；steady 状态下不能每帧都产日志。"""
  emitted = []
  saved_emit = GasLongCancel._emit
  try:
    GasLongCancel._emit = staticmethod(lambda msg: emitted.append(msg))
    c = mk()
    c.update(gas_pressed=False, engaged=True)
    c.update(gas_pressed=True, engaged=True)
    for _ in range(50):                    # 0.5 s 连续踩着，真实时间 < 1 s ⇒ 期望很少
      c.update(gas_pressed=True, engaged=True)
    assert len(emitted) <= 4, f"探针把节流击穿了，产出了 {len(emitted)} 条：{emitted[:3]}"
  finally:
    # ★ 必须包回 staticmethod：直接赋函数会让它变成实例方法 ⇒ self._emit(msg)
    #   变成两个实参，后续所有用例全被污染（第一版就踩了这个坑）。
    GasLongCancel._emit = staticmethod(saved_emit)


def test_probe_logs_phase_transitions():
  emitted = []
  saved_emit = GasLongCancel._emit
  try:
    GasLongCancel._emit = staticmethod(lambda msg: emitted.append(msg))
    c = mk()
    c.update(gas_pressed=False, engaged=True)
    c.update(gas_pressed=True, engaged=True)
    c.update(gas_pressed=True, engaged=False)      # 取消
    assert len(emitted) >= 2
    assert any(f"phase={PHASE_OFF}" in m for m in emitted)
    assert any(f"phase={PHASE_HELD}" in m for m in emitted)
  finally:
    # ★ 必须包回 staticmethod：直接赋函数会让它变成实例方法 ⇒ self._emit(msg)
    #   变成两个实参，后续所有用例全被污染（第一版就踩了这个坑）。
    GasLongCancel._emit = staticmethod(saved_emit)


def test_emit_swallows_exceptions():
  """cloudlog 不可用时也绝不上抛（controlsd 是 100 Hz 关键进程）。"""
  GasLongCancel._emit("[GasLongCancel] smoke")   # 不应抛异常
# ---------------------------------------------------------------------------
# 裸跑入口（本机没有 pytest）：`python3 test_gas_long_cancel.py` 直接全跑。
# ★ 必须显式调用 —— 只定义 test_* 函数就直接运行文件的话，Python 什么都不执行，
#   表现为「退出码 0 + 零输出」，极容易被当成"全部通过"（2026-09-29 踩过）。
# ---------------------------------------------------------------------------
def _run_all() -> int:
  import traceback as _tb
  names = sorted(n for n in list(globals()) if n.startswith("test_") and callable(globals()[n]))
  failed = 0
  for n in names:
    try:
      globals()[n]()
      print("PASS  %s" % n)
    except Exception:
      failed += 1
      print("FAIL  %s" % n)
      print(_tb.format_exc().rstrip()[-800:])
  print("\n%d/%d passed, %d failed" % (len(names) - failed, len(names), failed))
  return failed


if __name__ == "__main__":
  raise SystemExit(_run_all())
