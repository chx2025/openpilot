"""eGPU 状态模型（侧边栏用）。

侧边栏那一格原本显示 SUNNYLINK，本文件把它替换成 eGPU 状态：

  不在线   ->  显示「小模型」三个字   「小模型」
  加载中   ->  显示进度百分比         「45%」
  运行中   ->  显示功率 + GPU 即时温度 「120W 58°C」（2026-09-22 用户要求，原为占用率）

判断优先级（2026-09-17 修订）：
  不在线 > 运行中 > 加载中 > 加载失败 > 未编译 > 链路诊断(USB/PCIe)
「运行中 / 加载中」必须排在链路诊断之前 —— 原因见 build_egpu_sidebar_status 注释，
一句话：模型在跑本身就是链路通畅的最强证据，别让脆弱的链路推断把它顶掉。

纯函数 + 只读数据，不依赖 UI 状态，便于单测。
"""
from __future__ import annotations

from dataclasses import dataclass

from openpilot.common.hardware.usb import CHESTNUT_USB_IDS


# 侧边栏 "小模型" 文案：eGPU 不在线时显示
SMALL_MODEL_TEXT = "小模型"

# 加载进度在拿到真实百分比之前的估算时长（秒）。
# 当前分支的 modeld 只写 ChestnutLoading 布尔量、不写进度参数，
# 所以这里按时长做单调估算，99% 封顶，等真正就绪才跳到"运行中"。
#
# 实测（comma-e80e0b52 / release260916xl，2026-09-17，取自 swaglog 里
# modeld.py 的 "models loaded in ...s"）：
#     大模型加载成功 ≈ 20.0 / 20.4 / 20.5 / 21.6 / 22.2 s（均值 ≈ 20.9）
#     加载失败回退小模型 ≈ 2.3 s
# 之前拍脑袋写的 120.0 会让进度慢 6 倍，20 秒的真实加载只会爬到 17% 就跳走。
#
# 二次校准（2026-09-17）：22.0 -> 17.0，即在实测均值基础上主动提前 5s 收敛。
# 进度只是个估算，宁可早一点到 99% 等就绪，也不要"实际已经加载完、进度条
# 才爬到 91% 就跳走"——后者看起来像卡住了，前者才和真实加载同步。
# 就绪与否仍由 ChestnutLoading / ChestnutActive 决定，不会提前显示"运行中"。
CHESTNUT_LOAD_NOMINAL_S = 17.0


@dataclass(frozen=True)
class EgpuSidebarStatus:
  value: str      # 侧边栏大字（第二行）
  severity: str   # good / warning / danger / progress / disabled
  detail: str     # 中文明细，用于点击弹窗/日志


def build_egpu_sidebar_status(*, present: bool, compiled: bool, link_state: str | None,
                              usb_speed_mbps: int, pcie_ltssm: int | None,
                              loading: bool, active: bool | None,
                              loading_progress: int = 0,
                              model_failed: bool = False,
                              power_w: float = 0.0, gpu_usage_percent: int = 0,
                              gpu_temp_c: float = 0.0,
                              telemetry_valid: bool = False) -> EgpuSidebarStatus:
  """把一个采样时刻的 eGPU 状态压成侧边栏那一格要显示的文案。

  优先级（2026-09-17 修订）：
    不在线 > 运行中 > 加载中 > 加载失败 > 未编译 > 链路诊断

  ⚠️ 为什么「运行中 / 加载中」必须排在链路诊断之前：
  链路判据（usb_speed_mbps / pcie_ltssm / telemetry_valid）都建立在
  `chestnutState` 这个 cereal 服务的采样质量之上，而本分支
  `SubMaster.valid[...]` 恒为 False（实测 3580 次采样 200 次收包 0 次 valid），
  于是遥测被一路判成"无效" -> `check_error` -> LINK ERR。
  更关键的是它排在链首，会把**已经跑起来的大模型**和**正在加载的进度**一起顶掉
  （2026-09-17 实车表现：功率/占用率都在遥测里正常刷新，第四格却显示 LINK ERR，
  加载期也看不到百分比）。

  模型在跑 = 链路通畅的最强证据，所以以它为准；链路诊断只在
  「插着、既不加载也没在跑」时才有意义。
  """
  # --- eGPU 不在线：直接显示「小模型」---
  if not present:
    return EgpuSidebarStatus(SMALL_MODEL_TEXT, "disabled",
                             "未检测到 eGPU，当前运行小模型")

  # --- 运行中：显示功率 + GPU 即时温度（2026-09-22 用户要求，原为功率 + GPU 占用率）---
  # 温度取自 ChestnutState.tempC（cereal/log.capnp:718，单位 °C）。
  # tempC <= 0 视为"没读到"（chestnut 侧 _read_ina 失败时字段会保持 capnp 默认 0），
  # 此时显示 "--" 而不要显示 "0°C" —— 后者会被误读成"凉快"，掩盖遥测失效。
  if active is True:
    if telemetry_valid:
      temp_text = f"{gpu_temp_c:.0f}°C" if gpu_temp_c > 0.0 else "--"
      return EgpuSidebarStatus(f"{power_w:.0f}W {temp_text}", "good",
                               f"eGPU 大模型运行中 · 功耗 {power_w:.0f} W · "
                               f"GPU 温度 {temp_text} · GPU 占用 {gpu_usage_percent}% · USB {usb_speed_mbps} Mbps")
    return EgpuSidebarStatus("RUNNING", "good",
                             f"eGPU 大模型运行中（遥测暂不可用）· USB {usb_speed_mbps} Mbps")

  # --- 加载中：显示进度百分比 ---
  if loading:
    pct = max(0, min(99, int(loading_progress)))
    return EgpuSidebarStatus(f"{pct}%", "progress", f"eGPU 大模型加载中 {pct}%")

  # --- 加载/运行失败（已回退小模型）---
  # 只用 model_failed（= UIState.chestnut_state == FAILED，判据含 modelV2.big），
  # 不直接看 active is False：modeld 启动时会 remove("ChestnutActive")，
  # 那个瞬间 get_bool 返回 False，会被误报成 MODEL ERR。
  if model_failed:
    return EgpuSidebarStatus("MODEL ERR", "danger",
                             "eGPU 链路正常，但大模型加载或运行失败，已回退小模型")

  # --- 插着但大模型还没编译 ---
  if not compiled:
    return EgpuSidebarStatus("NO MODEL", "warning", "eGPU 已连接，但大模型尚未编译")

  # --- 到这里说明「插着、既没加载也没在跑」：此时才做链路诊断 ---
  if link_state == "usb_degraded" or 0 < usb_speed_mbps < 5000:
    return EgpuSidebarStatus(f"USB {usb_speed_mbps}", "danger",
                             f"USB 链路低于 5000 Mbps（当前 {usb_speed_mbps} Mbps）")
  if link_state == "pcie_down":
    ltssm = f"0x{pcie_ltssm:02X}" if pcie_ltssm is not None else "未知"
    return EgpuSidebarStatus("PCIE ERR", "danger",
                             f"USB 正常，但 PCIe 未进入 L0（LTSSM {ltssm}）")
  if link_state == "check_error":
    return EgpuSidebarStatus("LINK ERR", "danger", "无法被动读取 PCIe 链路状态")
  if link_state == "unchecked":
    return EgpuSidebarStatus("NO LINK", "warning",
                             f"eGPU 已连接（USB {usb_speed_mbps} Mbps），但还没收到 PCIe 遥测")
  if link_state != "ready":
    return EgpuSidebarStatus("CHECKING", "warning",
                             f"USB {usb_speed_mbps or '?'} Mbps，正在确认 PCIe 状态")

  return EgpuSidebarStatus("READY", "good", f"eGPU 已就绪 · USB {usb_speed_mbps} Mbps · PCIe L0")


def classify_egpu_link_state(*, present: bool, usb_speed_mbps: int, telemetry_alive: bool,
                             telemetry_valid: bool, pcie_ltssm: int) -> str:
  """把「USB 是否枚举 / 速率 / PCIe 链路」归成一个状态串。

  ⚠️ `telemetry_valid` 目前由调用方直接传 `telemetry_alive`（2026-09-17 修）。
  本分支 `SubMaster.valid[...]` 恒为 False（实测 3580 次采样 200 次收包 0 次 valid），
  拿它判断遥测有效性会让链路状态永远停在 `check_error`。
  `alive=True` 就代表 modeld/hardwared 在正常发布、字段可读。
  保留这个参数是为了不破坏调用约定，将来若真有独立的"数据完整性"信号可以接进来。
  """
  if not present:
    return "disconnected"
  if usb_speed_mbps < 5000:
    return "usb_degraded"
  if not telemetry_alive:
    return "unchecked"
  if not telemetry_valid:
    return "check_error"
  if pcie_ltssm != 0x78:
    return "pcie_down"
  return "ready"


def resolve_egpu_connection(device_state) -> bool:
  """当前物理连接状态（不做行程内的 latch）。"""
  return bool(device_state.chestnutPresent)


def chestnut_usb_speed_mbps(device_state) -> int:
  """取 eGPU(chestnut) 那条 USB 链路的速率（Mbps），没有则 0。"""
  speeds = [int(device.speedMbps) for device in device_state.usbState.devices
            if (int(device.vendorId), int(device.productId)) in CHESTNUT_USB_IDS]
  return max(speeds, default=0)
