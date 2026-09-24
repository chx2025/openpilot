#!/usr/bin/env bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

# models get lower priority than ui
# - ui is ~5ms
# - modeld is 20ms
# - DM is 10ms
# in order to run ui at 60fps (16.67ms), we need to allow
# it to preempt the model workloads. we have enough
# headroom for this until ui is moved to the CPU.
export QCOM_PRIORITY=12

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="19.7"
fi

export STAGING_ROOT="/data/safe_staging"

# eGPU(chestnut) 插着时 SConscript 会尝试编译 big_driving_supercombo.onnx，
# 而仓库未提供该 onnx；下载的模型已自带编译好的 pkl，无需本地重编。
export SKIP_TINYGRAD_COMPILE=1


# 让 tinygrad 的下载/编译缓存落在持久分区，避免每次开机重建
# （/home 是 overlay 临时层，重启即清空）
export TINYGRAD_CACHE_DIR=/data/tg_cache/tinygrad

# chestnut eGPU(ASM24) 功率上限：默认走显卡 SMU PPT 上限(182W)，
# 压到 120W 降低峰值电流，缓解高负载下 USB/PCIe 链路抖动
export AM_POWER_LIMIT=120

# ---- 开机自动校时（版本化，见 system/time_seed.sh 头部注释）----
# 放在仓库内而不是 /data/continue.sh：后者由
# openpilot/selfdrive/ui/installer/continue_openpilot.sh 生成，重装/OTA 会覆盖，
# 钩子会静默消失。脚本内部会等网络（实测约 55s 才就绪）并自带 flock 单实例保护，
# 所以必须 setsid + 后台执行，绝不能阻塞 openpilot 启动。
if [ -x "${DIR:-/data/openpilot}/system/time_seed.sh" ]; then
  setsid "${DIR:-/data/openpilot}/system/time_seed.sh" >>/data/time_seed.log 2>&1 &
fi

# ---- C3XL IFE 硬件缩图（修 CTM 大模型掉帧 / USB 5G 带宽不足）----
# 移植自 onemiless/openpilot@dev-sp-egpu 提交 2a69709f06。
# road 相机在 IFE 硬件里直接出 1344x760（非裁剪，全视场），像素 -43.8%、
# NV12 单帧 -56.2%，把两路 road @20Hz 的 USB 占用从 139.6MB/s 降到 61.3MB/s。
# 需要 /data/hardware_profile == c3xl（源码门控）。
# 回滚：删掉本行 + rm /data/hardware_profile + 重启。
export C3XL_IFE_ROAD_SIZE=1344x760

# ---- chestnut(eGPU) USB 节点权限守护（冷启动 EACCES 竞态）----
# devtmpfs 以 0600 root:root 建 /dev/bus/usb/<bus>/<dev>，udev 要读完描述符才按
# 50-udev-default.rules:72 放宽成 0664；eGPU 底座与车机同时上电时 ASM2464 描述符
# 响应慢，这个窗口能拖到几十秒，而 modeld 以 comma 身份在 T+34s 就 libusb_open
# -> EACCES -> load_big() 只试一次即放弃 -> 静默回退小模型。
# 这里开机即后台轮询、抢在 udev 前把节点 chmod 666；脚本自带自提权。
# 必须放仓库内（与 time_seed.sh 同理）：写 /data/continue.sh 会被重装覆盖。
if [ -x "${DIR:-/data/openpilot}/system/egpu_usb_perm.sh" ]; then
  setsid "${DIR:-/data/openpilot}/system/egpu_usb_perm.sh" >/dev/null 2>&1 &
fi
