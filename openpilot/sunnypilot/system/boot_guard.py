#!/usr/bin/env python3
"""看护 /tmp/booted —— 防止 comma.sh 重复执行「开机重置检查」。

背景（2026-09-19 定位，根因已实测复现）
--------------------------------------
/usr/comma/comma.sh 用 /tmp/booted 作为「本次开机已经查过重置」的哨兵：

    if [ ! -f /tmp/booted ]; then
      touch /tmp/booted
      if [ -f /data/__system_reset__ ]; then ...
      elif (( $(cat /sys/class/input/input2/device/touch_count) > 4 )); then
        $RESET --tap-reset          # ← 弹出触摸屏「System Reset」确认页并阻塞启动
      ...
    fi

该页面的 Confirm 按钮会执行 `sudo rm -rf /data/*` + `mkfs.ext4 userdata`，
即整个 /data（openpilot 仓库、params、全部日志）被清空并格式化。

本机（C3XL）没有 RTC，开机时系统时间停在假时间（如 2026-07-28 15:04）。
comma.sh 在这个假时间下 `touch /tmp/booted`，该文件的 atime/mtime 就被钉死在
那个假日期。等 NTP 把时钟校到真实日期后，systemd-tmpfiles 的规则

    D /tmp 1777 root root 30d      # 清理 /tmp 内 30 天未修改的内容

就认为它「已经 52 天没动过」，在开机后第一次清理（约 boot + 15min）时把它删掉。
此后任何一次 `sudo systemctl restart comma` 都会重新进入上面的分支 ——
只要触摸屏累计被戳过 5 次以上，就会弹出那个确认页并卡住启动。

实测证据：2026-09-19 01:17 开机 → 01:31 文件仍在 → 01:32:00 定时器执行 →
01:33 文件已消失（同目录的 socket 与 USB lock 因 atime 较新而幸存）。

本进程做什么
------------
每 REFRESH_INTERVAL_S 秒把哨兵的时间戳刷新到当前时间（必要时重建），
使 tmpfiles 的年龄判据永远不会命中。

时间线正确性：comma.sh 的重置检查发生在 openpilot 启动**之前**，
所以本进程不可能抢在真正的开机检查前面；它只影响之后的重启。
换句话说，真正的「开机 5 连击重置」功能完全不受影响。

回退
----
从 system/manager/process_config.py 的 procs 列表里删掉 boot_guard 一行即可，
无任何副作用。
"""
import os
import time

from openpilot.common.swaglog import cloudlog

BOOTED_SENTINEL = "/tmp/booted"
REFRESH_INTERVAL_S = 60.0

# 只在异常/状态变化时打日志，避免 20Hz 级的日志噪声
_logged_missing = False


def refresh_sentinel() -> None:
  """刷新 /tmp/booted 的时间戳；不存在就补回来。"""
  global _logged_missing

  if os.path.exists(BOOTED_SENTINEL):
    # utime 同时更新 atime 与 mtime —— tmpfiles 的年龄判据看的就是它们
    os.utime(BOOTED_SENTINEL, None)
    _logged_missing = False
    return

  # 被 tmpfiles 删掉了：补回来，否则下一次 restart comma 会触发重置确认页
  with open(BOOTED_SENTINEL, "w"):
    pass
  os.utime(BOOTED_SENTINEL, None)
  if not _logged_missing:
    cloudlog.warning(f"boot_guard: {BOOTED_SENTINEL} 缺失，已重建"
                     f"（若再被删除说明刷新间隔偏大）")
    _logged_missing = True


def main() -> None:
  cloudlog.info(f"boot_guard: 开始看护 {BOOTED_SENTINEL}，每 {REFRESH_INTERVAL_S:.0f}s 刷新一次")

  # 启动时立刻刷新一次，确保 NTP 校时后的假时间戳不再残留
  try:
    refresh_sentinel()
  except OSError as e:
    cloudlog.warning(f"boot_guard: 首次刷新失败 {type(e).__name__}: {e}")

  while True:
    time.sleep(REFRESH_INTERVAL_S)
    try:
      refresh_sentinel()
    except OSError as e:
      # 不抛出：本进程若反复崩溃会被 manager 反复重启，反而更吵
      cloudlog.warning(f"boot_guard: 刷新失败 {type(e).__name__}: {e}")


if __name__ == "__main__":
  main()
