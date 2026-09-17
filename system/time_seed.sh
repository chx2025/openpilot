#!/usr/bin/env bash
# 开机校时：先布置 NTP，再用 HTTPS Date 头「粗校」兜底
#
# 背景（C3XL / AGNOS，实测）：
#   1. / 是只读挂载（remount rw 无效），/etc 不可写 → timesyncd 的配置只能放
#      /run/systemd/timesyncd.conf.d/（tmpfs），所以每次开机都要重写一遍。
#   2. 出厂默认 NTP 服务器 ntp.ubuntu.com 在本网络被解析成 198.18.0.36（代理 fake-IP），
#      UDP/123 黑洞 → timesyncd 从开机起一直超时，时间永远修不回来。
#      但 ntp.aliyun.com / ntp.tencent.com / cn.pool.ntp.org 都正常（20~290ms）。
#   3. 本机 RTC(qpnp_rtc) 实际是个「开机计时器」而不是挂钟：内核日志
#      `setting system clock to 1970-01-01 00:47:54 UTC (2874)`，读数随开机时长递增，
#      断电不可保留 → 别指望靠它记住时间，只能靠外部源。
#   4. HTTPS(443) 是通的 → 用 HTTP 响应头的 Date: 拿 UTC，秒级精度，做「粗校」兜底最稳。
#   5. 开机后 1~2 秒内网络还没起来（实测取不到 Date 头）→ 必须等网络 + 重试
#      （实测约 55s 网络才就绪）。
#
# 调用点：仓库内 launch_env.sh（版本化，每次开机自动后台执行）
#   —— 不放在 /data/continue.sh：那个文件是由仓库里的
#   openpilot/selfdrive/ui/installer/continue_openpilot.sh 生成/覆盖的，
#   重装或更新就会丢掉钩子。本脚本随仓库走，git pull 即可恢复。
#   手动执行：/data/openpilot/system/time_seed.sh
set -u

LOG=/data/time_seed.log
LOCK=/data/time_seed.lock
NTP_SERVERS="ntp.aliyun.com ntp.tencent.com cn.pool.ntp.org"
NTP_FALLBACK="ntp1.aliyun.com ntp1.tencent.com ntp.sjtu.edu.cn"
MIN_STEP=60                 # 偏差绝对值 >= 该秒数才 step（避免抖动）
MAX_WAIT=90                 # 最多等网络 90s
TIME_URLS="https://www.baidu.com https://api.github.com https://www.aliyun.com"

log() { printf '%s  %s\n' "$(date -u '+%F %T UTC')" "$*" >>"$LOG"; }

# 单实例保护：launch_env.sh 与手动执行可能同时触发，重复跑只会互相踩 NTP 配置
exec 9>"$LOCK" 2>/dev/null || true
if command -v flock >/dev/null 2>&1; then
  if ! flock -n 9; then
    log "=== time_seed already running, skip (uptime $(cut -d' ' -f1 /proc/uptime)s) ==="
    exit 0
  fi
fi

log "=== time_seed start (uptime $(cut -d' ' -f1 /proc/uptime)s) ==="

# ---------- 1) 先布置 NTP：网络一起来就自动开始对时（这条是主力）----------
drop=/run/systemd/timesyncd.conf.d
if sudo -n mkdir -p "$drop" 2>/dev/null \
   && printf '[Time]\nNTP=%s\nFallbackNTP=%s\n' "$NTP_SERVERS" "$NTP_FALLBACK" \
      | sudo -n tee "$drop/10-local-ntp.conf" >/dev/null 2>&1; then
  sudo -n systemctl restart systemd-timesyncd >/dev/null 2>&1 || true
  log "timesyncd -> $NTP_SERVERS (fallback: $NTP_FALLBACK)"
else
  log "cannot write $drop (no sudo?) - NTP override skipped"
fi

# ---------- 2) 粗校兜底：等网络就绪，反复取 HTTPS Date 头 ----------
real=""; src=""; waited=0
while [ "$waited" -lt "$MAX_WAIT" ]; do
  for u in $TIME_URLS; do
    hdr=$(timeout 6 curl -skI "$u" 2>/dev/null \
          | awk 'tolower($1)=="date:" {print $2" "$3" "$4" "$5" "$6; exit}' | tr -d '\r')
    [ -n "$hdr" ] || continue
    e=$(date -u -d "$hdr" +%s 2>/dev/null) || continue
    # 合理性校验：必须晚于 2024-01-01，防代理返回的垃圾头
    [ "${e:-0}" -gt 1704067200 ] && { real="$e"; src="$u"; break 2; }
  done
  sleep 5; waited=$((waited + 5))
done

if [ -n "$real" ]; then
  now=$(date +%s); off=$(( real - now )); absoff=${off#-}
  if [ "$absoff" -ge "$MIN_STEP" ]; then
    if sudo -n date -u -s "@$real" >/dev/null 2>&1; then
      log "coarse step ${off}s (src=$src, waited ${waited}s) -> $(date -u '+%F %T UTC')"
    else
      log "coarse step FAILED (off=${off}s) - sudo 不可用?"
    fi
  else
    log "offset ${off}s < ${MIN_STEP}s (timesyncd 已对齐), no step (waited ${waited}s)"
  fi
else
  log "no usable Date header after ${waited}s - 只能交给 timesyncd"
fi

log "=== time_seed done -> $(date -u '+%F %T UTC') ==="
