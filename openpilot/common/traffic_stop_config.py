#!/usr/bin/env python3
"""红灯停车模组（Traffic Light Stop）的配置读写 —— 控制侧与 UI 侧共享。

被三处使用：
  - 控制侧  openpilot/sunnypilot/selfdrive/controls/lib/traffic_stop.py（20 Hz，每 1 s 轮询）
  - 标准 UI openpilot/selfdrive/ui/layouts/settings/toggles.py
  - 紧凑 UI openpilot/selfdrive/ui/mici/layouts/settings/toggles.py

为什么不用 Params
-----------------
Params 的 key 白名单（取值范围会做校验）被编译进 libparams_c.so，
新增 key 必须重编译 C 扩展。本模块改用独立 JSON 文件承载开关，
做到「零编译即可热切换」——这正是本次移植选择轻量版方案的前提。

文件格式
--------
    {
      "enabled": true,                 # UI 开关
      "distance_adjust_m": 0.0,        # 停车点微调 ±5 m
      "tuning": { ... }                # 判据阈值（见 DEFAULT_TUNING），可选
    }

写入协议
--------
  - UI 只改 enabled：**读改写**，其余字段（含 tuning）原样保留
  - **写入原子化**（同目录临时文件 + os.replace）。这一步是必须的：
    控制侧每 1 s 读一次，若直接 open(path, "w") 会先截断文件，
    控制侧正好撞在写入窗口里就会读到空/半截 JSON。
  - 不做文件锁：只有一个 UI 进程写、控制侧只读，且写入是原子的。

读取语义
--------
  - 查找顺序：CONFIG_PATH（运行时，UI 写这里）-> DEFAULT_CONFIG_PATH（仓库内出厂模板）
  - 两处都读不到 / 解析失败 / 字段缺失  -> 各自回到代码默认值
  - 代码默认 enabled=True

为什么要有「出厂模板」
----------------------
CONFIG_PATH 位于 /data，而 /data 会被重装 / 整机重置清空 —— 用户在这里改过的
开关与标定值会**静默消失**，行为回落到代码默认值。2026-09-24 设备重装后正是
如此：文件没了，`enabled` 翻回代码默认的 True，而用户实际要的是关着的。

所以仓库内随代码带一份 `traffic_stop.default.json` 当出厂默认：
  - 运行时文件缺失 -> 按模板取值（重装后行为与用户预期一致）
  - 用户一旦在 UI 里改过，改动写进 CONFIG_PATH，此后以该文件为准（模板不再参与）
  - 模板本身不含 tuning 段时，tuning 全部回落到 StopTuning() 的代码默认值

本模块**只依赖标准库**（不 import numpy / openpilot 内部模块），
这样 UI 进程 import 它不会有任何副作用或依赖链风险。
"""
import json
import os
import tempfile
from typing import NamedTuple

CONFIG_PATH = os.environ.get("TRAFFIC_STOP_CONFIG", "/data/traffic_stop.json")

# 仓库内出厂模板（与 CONFIG_PATH 是两个不同的东西）：
# 运行时文件被重装擦掉时按它取值，避免静默回落到代码默认而改变功能状态。
# `__file__` 用 globals() 取 —— 验证脚本用 exec 执行本模块时会绕过模块机制，
# 那时没有 `__file__`（空串 = 不启用模板回落），真实运行时一定有。
_HERE = os.path.dirname(os.path.abspath(globals().get("__file__"))) if globals().get("__file__") else ""
DEFAULT_CONFIG_PATH = os.path.join(_HERE, "traffic_stop.default.json") if _HERE else ""

DEFAULT_ENABLED = True              # 代码层兜底：模板与运行时文件都读不到时才用它
DEFAULT_DISTANCE_ADJUST_M = 0.0
DISTANCE_ADJUST_LIMIT_M = 5.0       # 与 traffic_stop.py 的停车点微调上限保持一致


class StopTuning(NamedTuple):
  """判据阈值。全部可通过 JSON 的 "tuning" 段热调（1 s 生效，无需重启）。

  这些值之所以做成可调：它们是「模型预测要停」的量化定义，而不同模型
  （TSFDOM / 官方 driving model …）的轨迹末端行为并不一致，
  必须用实车日志标定，不能靠猜。

  [第三轮] 判据从「看速度轨迹形状」改成「看模型预测的**停止位置**」——
  所以字段也重排成「停止点 / 稳定性 / 介入时机 / 刹车力」四组。
  """
  # ① 「模型打算停在哪儿」= 停止点
  stop_terminal_v_ms: float = 0.5        # 轨迹速度低于它就认为「这一段已经静止」
  decel_ratio: float = 0.6               # 「模型在减速」：末速须 < 起点速度 × 该比例
  extrap_frac: float = 0.25              # 视界内没触零时，用轨迹末段多少比例做外推
  min_decel_ms2: float = 1.0             # 外推要求的最小减速度（低于它只算「减速」不算「要停」）
  extrap_max_v_end_ms: float = 2.5       # 只有末速已经这么低了才值得外推（否则可能只是「慢下来继续走」）

  # ② 停止点稳定性（用户第 2 步：停止位置 2 s 内基本没变）
  #
  #   [第七轮 2026-09-19] 容差必须**随车速放大** —— 固定 3.0 m 把高速接近整段挡死。
  #
  #   实车实证（2026-09-19 晚间 50+ km/h 两次红灯，[TrafficStopProbe] 逐帧）：
  #     车速 13.8kph -> 窗口原始极差 5.5m   req=2.92 (>comfort 2.0)  但 stable=0
  #     车速 19.6kph -> 4.3m                req=2.56                 stable=0
  #     车速 27.0kph -> 6.6m                req=2.64                 stable=0
  #     车速 33.9kph -> 6.6m                req=2.29                 stable=0
  #     车速 46.7kph -> 10.9m               req=1.56                 stable=0
  #   ⇒ 整段接近 state=CRUISE / outA=0.00，**一点刹车力都没给**，
  #     车靠模型自己的 ~1.5 m/s² 减速，最终越过停止线约 2 m（用户「都停过线」）。
  #
  #   对照：车速 7.5kph 时原始极差只有 2.4m -> stable=1（唯一一次介入）。
  #   即「原始极差 ≈ 0.8 + 0.22 × v_kph」：**误差与车速成正比**，这不是噪声，
  #   是**模型推理延迟 × 车速**产生的确定性位置误差（50 km/h 时 0.2 s 延迟就是 2.8 m）。
  #   和第五轮那个「自车位移混进停止点」是同一类根因：**判据的量纲选错了**。
  #
  #   修法：tol_eff = stop_pt_tol_m + stop_pt_tol_speed_gain × v_kph
  #     0kph -> 3.0m    20kph -> 7.0m    35kph -> 10.0m    50kph -> 13.0m
  #   增益取 0.20 而不是按「实测极差 ≈ 0.8+0.22v」拟合出的 0.22，也刻意不用
  #   更紧的 0.12 —— 验收标准是**最坏情况必须通过**：判据只剔掉一个离群点，
  #   若极端情况下剔除后极差仍等于原始极差，上表 5 帧也要全部通过：
  #     13.8kph: 5.5 ≤ 5.76 | 19.6: 4.3 ≤ 6.92 | 27.0: 6.6 ≤ 8.40
  #     33.9kph: 6.6 ≤ 9.78 | 46.7: 10.9 ≤ 12.34   （0.12 只能过 2 帧）
  #   高速端由 stop_pt_tol_hard_m 兜住：80kph 时 tol_eff=19m 已超过硬上限 18m。
  #   回退：stop_pt_tol_speed_gain=0 且 stop_pt_tol_hard_m=6.0 即精确回到第六轮行为。
  stop_pt_stable_s: float = 2.0          # 停止点要保持一致多久
  stop_pt_tol_m: float = 3.0             # 静止时的容差（米）；实际容差见 speed_gain
  stop_pt_tol_speed_gain: float = 0.20   # 每 kph 追加的容差（米/kph）—— 模型延迟误差 ∝ v
  stop_pt_tol_hard_m: float = 18.0       # 原始极差硬上限：真在连续漂移的停止点仍然否决
  #   [第八轮 2026-09-20] present_ratio 必须与模型的**真实输出率**相容。
  #   旧值 0.9（要求 3 s 窗口里 90% 的帧都给停止点）与实车不符：实测行驶中
  #   模型 74% 的帧 src=none（压根不给停止点），输出率仅 26% ⇒ **stable 在高速段
  #   数学上不可能成立**（与容差无关，第七轮改容差因此完全无效）⇒ 介入被推迟到
  #   车慢下来、模型输出变密之后（5 次 engage 全在 v ≤ 36.5 kph，3 次 ≤ 20 kph）
  #   ⇒ 用户看到的「介入晚 + 刹不住 + 过线」。
  #   改 0.3 = 判据窗口（2 s / 40 帧）里至少 0.6 s 的稳定停止点输出。
  #   防误触发不再靠这道门，改由下游承担：required > comfort_decel_ms2、
  #   decelerating（模型确实在减速）、横向容差、以及 ⑥ 的 30 m 跟车抑制。
  #   回退：设回 0.9 即精确回到第七轮行为。
  stop_pt_present_ratio: float = 0.3     # 判据窗口内有停止点的帧占比下限

  # ③④ 介入时机：剩下的距离已经不够舒适刹停了
  #
  #   [第八轮 2026-09-20] 2.0 -> 1.5。这是「介入时机」的第一旋钮：
  #   required = v²/(2d) > comfort 才介入 ⇒ 等效介入距离 d_gate = v²/(2·comfort)。
  #     40 kph: 30.9m -> 41.2m     50 kph: 48.2m -> 64.3m
  #   这不是「更凶地刹」，而是**更早地开始轻刹**：介入瞬间 required 恰好 = comfort，
  #   只给 1.5 m/s²（与 MPC 自己正在做的事同量级，体感不突兀），随距离收窄才逼近上限。
  #   实车实证：事件2 在 v=40.4kph/28.6m 时 required 已 2.28（>旧门限 2.0）却因
  #   stable=0 未介入；事件3 在 v=43.8kph/48.5m 时 required=1.56 —— 若门限为 1.5，
  #   **在此就介入**，比原来的 34.3kph/23.0m 提前约 2.5 s。
  #   注意 comfort 同时也是 v_limited（软限速曲线）的系数，降低它会一并让软限速更早生效。
  comfort_decel_ms2: float = 1.5         # 舒适刹停能力；required 超过它才介入
  assist_max_dist_m: float = 150.0       # 安全上限：停止点比这更远就绝不介入
  assist_margin_m: float = 1.0           # 反解时预留的落脚余量

  # ⑤ 辅助刹车力
  #
  #   [第八轮 2026-09-20] 2.16 -> 2.8。旧值只比介入门限 comfort 高 8%，
  #   等于**从介入那一刻起就没有任何余量**：实车 STOPPING 期间 required 实测
  #   2.07 / 2.33 / 2.84 / 4.30，而 cap=2.16 ⇒ 每一帧都欠 0~2 m/s²，
  #   这就是用户说的「刹车力不够」。
  #   2.8 ≈ 0.29 g，仍属平缓（人的紧急制动在 6~8 m/s²），且实际输出是
  #   min(cap, required) —— 距离远时照样只给 required 的轻刹，只有真需要时才逼近 2.8。
  #   与 45 kph 台阶合并看：45 以下 2.8、以上 3.0，台阶落差从 0.84 收到 0.2。
  max_brake_ms2: float = 2.8             # 常规刹车上限（≈0.29 g）
  high_speed_kph: float = 45.0           # 高速区下界
  high_speed_brake_ms2: float = 3.0      # 高速区刹车上限（高速红灯的物理需求更硬）

  # 释放 / 抑制
  release_terminal_v_ms: float = 2.0     # 「模型明确预测还在走」的阈值 -> 释放
  release_confirm_ratio: float = 0.6     # 释放所需的证据占比
  suppress_if_decel_ms2: float = -2.5    # ACC 已经比这更狠地在刹 -> 不叠加（防重复硬刹）

  # ⑥ 跟车让位（用户 2026-09-19 实车要求「30 m 内有前车就不触发」）
  #
  #   [第九轮 2026-09-20] 方案 A：让位判据从「固定 30 m」改成
  #   「前车**确实在虚拟停止线之前**才让位」。
  #
  #   旧判据 `d_rel <= 30 m 一律让位` 有两个方向的错：
  #     错 A（保守过头）：停止线在 12 m、前车在 25 m —— 前车明明在停止线**之后**，
  #           我们却因为「25 < 30」把辅助整个关掉，眼看着越过 12 m 的线。
  #           这正是 >30 kph 时「停过线」的现场工况之一。
  #     错 B（三套门限）：update() 里的 lead_cancels 用 4 m 门限、
  #           stop_sign 里的 ahead_of_lead 用 3 m 门限、入口闸门用 30 m 距离门限 ——
  #           三个表达式互不相同，必然出现「判据说可以介入、状态机说不能让进」
  #           的分裂（第五轮踩过同一个坑）。
  #
  #   新判据（**唯一来源**，三处共用 is_lead_yielding_stop）：
  #       让位 ⟺ d_rel < d_to_stop + lead_yield_margin_m
  #   物理含义：停止点在 d_to_stop 处、前车在 d_rel 处。
  #     d_rel <  d_to_stop  -> 前车先到，虚拟停止线没有意义 -> 交给 ACC 跟前车
  #     d_rel >  d_to_stop  -> 停止线更近，我们本就该停在**前车之前**
  #                            （至少 lead_yield_margin_m 米）-> 保持介入
  #
  #   lead_yield_margin_m 是这一轮的新旋钮：
  #     正值（默认 4.0）= 前车与停止线齐平时也让位（宁可早让，安全侧）；
  #     调大 = 让位更积极；调小/负值 = 要求前车"明确"在停止线之前才让位；
  #     取 -60 等价于「几乎不让位」（仅在极近的前车下让位）。
  lead_yield_margin_m: float = 4.0

  #   下面这项**退为兜底**：模型这帧没给停止点（高速下 74% 的帧如此）就没有
  #   比较基准，退回固定距离门限。只在这一种情况下生效。
  #   0 = 这种兜底也不让位。
  lead_suppress_dist_m: float = 30.0

  # ⑦ [第六轮 2026-09-19] 已停稳分支的「信息量」门槛 + 抖动抑制。
  #   根因：车停着时模型必然预测自己不动，`get_model_stop_point` 的「视界内已静止」
  #   路径会把停止点算成 x[0]≈0，于是「已停稳维持」判据 d_to_stop < 20 m 在
  #   **任何**静止时刻都成立，与前方有没有停止线无关。实车日志实证 engage 时
  #   stopPoint = 0.1 / -0.0 / 0.6 / 0.7 m。
  #   后果：每次停车（跟车、停车位、路边）都新开一次停等，而释放只看「模型还给不给
  #   停止点」，停止点在 0~3 m 之间来回跳 -> 实车出现一秒内 4 次 engage/release。
  #   stop_point_min_hold_m：停止点小于它就认为「没有信息量」，只挡**新介入**，
  #     不影响已经在进行的停等的维持（避免把安全兜底做掉）。
  stop_point_min_hold_m: float = 2.0
  #   stop_reengage_cooldown_s：一次「短命停等」结束后封锁再介入的时长。
  #     只对 stop_reengage_min_episode_s 以内的短事件生效 —— 长停等（真红灯）
  #     结束后的下一次介入不受影响。
  stop_reengage_cooldown_s: float = 6.0
  stop_reengage_min_episode_s: float = 3.0

  probe: bool = True                     # 是否输出 [TrafficStopProbe] 标定日志


DEFAULT_TUNING = dict(StopTuning()._asdict())

# 每个可调项的合法范围（load_tuning 会钳位，防止手改 JSON 写出离谱值）
_TUNING_RANGE = {
  "stop_terminal_v_ms": (0.0, 5.0),
  "decel_ratio": (0.0, 1.0),
  "extrap_frac": (0.05, 0.6),
  "min_decel_ms2": (0.05, 3.0),
  "extrap_max_v_end_ms": (0.5, 8.0),
  "stop_pt_stable_s": (0.2, 3.0),
  "stop_pt_tol_m": (0.5, 20.0),
  "stop_pt_tol_speed_gain": (0.0, 0.5),
  "stop_pt_tol_hard_m": (1.0, 60.0),
  "stop_pt_present_ratio": (0.05, 1.0),   # [第八轮] 下限从 0.3 放开：0.3 已是默认值，想再放松得有空间
  "comfort_decel_ms2": (0.5, 4.0),
  "assist_max_dist_m": (5.0, 250.0),
  "assist_margin_m": (0.0, 10.0),
  "max_brake_ms2": (0.3, 4.0),
  "high_speed_kph": (10.0, 120.0),
  "high_speed_brake_ms2": (0.3, 6.0),
  "release_terminal_v_ms": (0.0, 15.0),
  "release_confirm_ratio": (0.0, 1.0),
  "suppress_if_decel_ms2": (-6.0, 0.0),
  "lead_yield_margin_m": (-60.0, 60.0),   # [第九轮] 让位余量，可为负（= 要求前车明确更近）
  "lead_suppress_dist_m": (0.0, 120.0),
  "stop_point_min_hold_m": (0.0, 10.0),
  "stop_reengage_cooldown_s": (0.0, 30.0),
  "stop_reengage_min_episode_s": (0.0, 15.0),
}


def _read_raw() -> dict:
  """读原始 JSON 字典。**永不抛异常**。

  查找顺序：运行时配置 CONFIG_PATH -> 仓库内出厂模板 DEFAULT_CONFIG_PATH。
  两处都读不到 / 解析失败 / 不是对象 -> 空字典（调用方各自回落到代码默认值）。

  用 TRAFFIC_STOP_CONFIG 显式覆盖过路径时**只看那一个文件**：验证脚本靠这个
  环境变量做隔离，隔离状态下掺进模板会让"文件不存在"的预期值失真。
  """
  if "TRAFFIC_STOP_CONFIG" in os.environ or not DEFAULT_CONFIG_PATH:
    candidates = (CONFIG_PATH,)
  else:
    candidates = (CONFIG_PATH, DEFAULT_CONFIG_PATH)
  for path in candidates:
    try:
      with open(path) as f:
        data = json.load(f)
      if isinstance(data, dict):
        return data
    except Exception:
      continue
  return {}


def load_tuning() -> StopTuning:
  """读 tuning 段，逐项钳位；缺失/非法的项回落到默认值。**永不抛异常**。"""
  raw = _read_raw().get("tuning")
  if not isinstance(raw, dict):
    raw = {}
  values = {}
  for key, default in DEFAULT_TUNING.items():
    value = raw.get(key, default)
    if key == "probe":
      values[key] = bool(value)
      continue
    try:
      value = float(value)
    except (TypeError, ValueError):
      value = float(default)
    lo, hi = _TUNING_RANGE[key]
    values[key] = max(lo, min(hi, value))
  return StopTuning(**values)


def load_config() -> tuple[bool, float]:
  """读开关，返回 (enabled, distance_adjust_m)。**永不抛异常**。"""
  data = _read_raw()
  try:
    enabled = bool(data.get("enabled", DEFAULT_ENABLED))
    adjust = float(data.get("distance_adjust_m", DEFAULT_DISTANCE_ADJUST_M))
    adjust = max(-DISTANCE_ADJUST_LIMIT_M, min(DISTANCE_ADJUST_LIMIT_M, adjust))
    return enabled, adjust
  except Exception:
    # 字段类型不对 —— 一律按默认值处理（默认开启）
    return DEFAULT_ENABLED, DEFAULT_DISTANCE_ADJUST_M


def is_enabled() -> bool:
  """UI 侧初始化开关状态用。"""
  return load_config()[0]


def save_config(enabled: bool, distance_adjust_m: float | None = None) -> bool:
  """原子写入开关。distance_adjust_m 传 None 时保留文件里已有的值。

  **读改写**：先把文件里现有的全部字段读出来，只覆盖这两个键。
  不能只写这两个键 —— 那样 UI 一按开关就会把 tuning 段整体抹掉。

  返回是否写成功 —— UI 侧据此决定要不要把开关状态回滚。
  """
  payload = _read_raw()
  if distance_adjust_m is None:
    try:
      distance_adjust_m = float(payload.get("distance_adjust_m", DEFAULT_DISTANCE_ADJUST_M))
    except (TypeError, ValueError):
      distance_adjust_m = DEFAULT_DISTANCE_ADJUST_M

  payload["enabled"] = bool(enabled)
  payload["distance_adjust_m"] = max(-DISTANCE_ADJUST_LIMIT_M,
                                     min(DISTANCE_ADJUST_LIMIT_M, float(distance_adjust_m)))
  return _atomic_write(payload)


def save_tuning(values: dict) -> bool:
  """合并式更新 tuning 段（其余字段保留）。**原子写**。"""
  payload = _read_raw()
  tuning = payload.get("tuning")
  if not isinstance(tuning, dict):
    tuning = {}
  tuning.update(values)
  payload["tuning"] = tuning
  return _atomic_write(payload)


def _atomic_write(payload: dict) -> bool:
  """同目录临时文件 + fsync + os.replace。返回是否成功。"""
  try:
    directory = os.path.dirname(CONFIG_PATH) or "."
    os.makedirs(directory, exist_ok=True)
    # mkstemp 保证文件名唯一（同机多进程同时写也不会互相覆盖）
    fd, tmp_path = tempfile.mkstemp(dir=directory, prefix=".traffic_stop.", suffix=".tmp")
    try:
      with os.fdopen(fd, "w") as f:
        json.dump(payload, f)
        f.flush()
        os.fsync(f.fileno())
      # 同一文件系统内的 os.replace 是原子的：读端要么看到旧文件、要么看到新文件
      os.replace(tmp_path, CONFIG_PATH)
    except Exception:
      try:
        os.unlink(tmp_path)
      except OSError:
        pass
      raise
    return True
  except Exception:
    return False
