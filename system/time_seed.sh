#!/usr/bin/env bash
# 开机校时 + 时间锚点（保证系统时间「永不倒退」）
#
# ============================ 为什么要有「时间锚点」 ============================
# 本机 RTC(qpnp_rtc) 是坏的：内核日志每次开机都是
#   `setting system clock to 1970-01-01 00:00:07`，断电不保留。
# 而网络又不一定连得上（实测 wlan0 要 ~100s 才给默认路由，且并非每次都能连上）。
# 于是自建一个锚点文件，记录「最后一次已知正确的时间」：
#
#   开机第 0 秒（不依赖网络）：读锚点 → 若系统时间比锚点小 → 直接 step 到锚点
#       ⇒ 即使整趟没网，时间也**不会倒退**。最坏只是「比真实时间早」，
#         早的量 = 上次刷新锚点到这次开机之间的间隔（≤ 10 分钟 + 停车时长）。
#   之后常驻，每 10 分钟把当前时间写回锚点
#       ⇒ 锚点始终 ≈ 上次断电时刻，停车越久误差只体现在「停车时长」这一段。
#   一旦 NTP / HTTPS 任何一路校时成功，锚点被抬到真实时间，此后即使断网也从这里起步。
#
# 关键：**没有「熄火时刻」可以抓**。设备是硬掉电（拔钥匙直接断），
#       `/etc/systemd/system/` 里也没有任何 shutdown unit ⇒ 关机钩子不可靠、不存在。
#       所以用「运行期定期刷新」代替「熄火时写一次」。
#
# ============================ 单调性两条铁律 ============================
#   1) step 只向前：任何 step 都要求 candidate > 当前系统时间。
#   2) 写锚点只增不减：只有 new > old 才落盘。
#      唯一例外：NTP 已同步（NTPSynchronized=yes）时，NTP 是权威源，
#      允许把锚点「校正」到系统时间（含变小），并且此时不再做倒退拉回
#      —— 否则一个错误偏大的锚点会把时间永久卡在未来。
#
# ============================ 与系统文件的关系 ============================
#   只新增一个**自己的数据文件** /data/time_anchor（纯文本，一行）。
#   不修改 /data/params、不新增 systemd unit、不改 AGNOS 任何配置。
#   本脚本自身在仓库内（system/time_seed.sh），随 git 走，重装/OTA 后 git pull 即恢复。
#
# 调用点：仓库内 launch_env.sh（setsid 后台执行 ⇒ 本脚本可以常驻做心跳）
#   手动执行：/data/openpilot/system/time_seed.sh        （会常驻，Ctrl-C 退出）
#   联调自测：TIME_SEED_DRY=1 TIME_ANCHOR_FILE=/tmp/ta TIME_SEED_TICK=2 \
#             TIME_SEED_ANCHOR_EVERY=6 timeout 15 /data/openpilot/system/time_seed.sh
set -u

# ---------------- 可调参数（环境变量可覆盖，便于联调）----------------
ANCHOR="${TIME_ANCHOR_FILE:-/data/time_anchor}"   # ★ 自建锚点文件
TICK="${TIME_SEED_TICK:-60}"                      # 巡检间隔(s)：查 NTP 状态 + 防倒退
ANCHOR_EVERY="${TIME_SEED_ANCHOR_EVERY:-600}"     # 刷新锚点间隔(s)：用户要求 10 分钟
DRY="${TIME_SEED_DRY:-0}"                         # 1=不真写盘/不真改时间（联调用）
MAX_WAIT="${TIME_SEED_MAX_WAIT:-300}"             # 等网络做 HTTPS 粗校的上限(s)
HEARTBEAT_LOG="${TIME_SEED_HEARTBEAT_LOG:-600}"   # 心跳日志最小间隔(s)，防日志膨胀

LOG="${TIME_SEED_LOG:-/data/time_seed.log}"
LOCK="${TIME_SEED_LOCK:-/data/time_seed.lock}"
NTP_SERVERS="ntp.aliyun.com ntp.tencent.com cn.pool.ntp.org"
NTP_FALLBACK="ntp1.aliyun.com ntp1.tencent.com ntp.sjtu.edu.cn"
MIN_STEP=60                 # 偏差绝对值 >= 该秒数才 step（避免抖动）
TIME_URLS="https://www.baidu.com https://api.github.com https://www.aliyun.com"

# 锚点合理性判断窗口（与 openpilot 的 system_time_valid 同量级：2024-01-01 ~ 2035-01-01）
T_MIN=1704067200            # 2024-01-01
T_MAX=2051222400            # 2035-01-01

# 日志行自带**开机标识**与 uptime —— 时钟修好之前写下的行，时间戳本身就是错的
# （实测每次开机 RTC 都读出同一个值，导致不同开机的日志行看起来一模一样；
#   我们 2026-09-20 就因此误判过一次「钩子没跑」）。boot_id 每次开机唯一。
BOOT_ID="$(cut -c1-8 /proc/sys/kernel/random/boot_id 2>/dev/null || echo unknown)"
uptime_s() { cut -d' ' -f1 /proc/uptime 2>/dev/null | cut -d. -f1; }
log() {
  printf '%s  [boot:%s up:%ss] %s\n' \
    "$(date -u '+%F %T UTC')" "$BOOT_ID" "$(uptime_s)" "$*" \
    >>"$LOG"
}

# ---------------- 时间设置（DRY 模式只打印）----------------
set_time() {  # $1 = epoch 秒
  if [ "$DRY" = "1" ]; then
    printf '  [DRY] date -u -s @%s  (=%s)\n' "$1" "$(date -u -d "@$1" '+%F %T UTC' 2>/dev/null)"
    return 0
  fi
  sudo -n date -u -s "@$1" >/dev/null 2>&1
}

# ---------------- 锚点读写 ----------------
# 读：输出 epoch（无效/不存在则返回非 0）
anchor_epoch() {
  [ -f "$ANCHOR" ] || return 1
  read -r _e _rest <"$ANCHOR" 2>/dev/null || true
  case "${_e:-}" in ''|*[!0-9]*) return 1 ;; esac
  [ "$_e" -ge "$T_MIN" ] && [ "$_e" -le "$T_MAX" ] || return 1
  printf '%s' "$_e"
}

# 写：$1=epoch  $2=1 表示允许变小（仅在 NTP 权威时用）
anchor_write() {
  local e="$1" force="${2:-0}" cur
  [ "$e" -ge "$T_MIN" ] && [ "$e" -le "$T_MAX" ] || return 1
  cur=$(anchor_epoch) || cur=0
  if [ "$force" != "1" ] && [ "$e" -le "${cur:-0}" ]; then
    return 1                       # 只增不减
  fi
  if [ "$DRY" = "1" ]; then
    printf '  [DRY] anchor_write %s (old=%s) -> %s\n' "$e" "${cur:-none}" "$ANCHOR"
    return 0
  fi
  local t; t="$(mktemp "${ANCHOR}.tmp.XXXXXX" 2>/dev/null)" || return 1
  printf '%s %s boot=%s up=%s\n' \
    "$e" "$(date -u -d "@$e" '+%Y-%m-%dT%H:%M:%SZ')" "$BOOT_ID" "$(uptime_s)" >"$t" \
    && mv -f "$t" "$ANCHOR"
}

# ---------------- 单实例保护 ----------------
exec 9>"$LOCK" 2>/dev/null || true
if command -v flock >/dev/null 2>&1; then
  if ! flock -n 9; then
    log "=== time_seed already running, skip (uptime $(uptime_s)s) ==="
    exit 0
  fi
fi

log "=== time_seed start (uptime $(uptime_s)s, anchor=$ANCHOR tick=${TICK}s refresh=${ANCHOR_EVERY}s) ==="

# ============ 阶段 0：时间锚点兜底（开机第 0 秒，不依赖网络）============
# 这是「时间不倒退」的主保障：哪怕后面全程没网，这一步已经把时间抬到锚点之上。
HAVE_ANCHOR=0
if a=$(anchor_epoch); then
  HAVE_ANCHOR=1
  now=$(date +%s)
  if [ "$a" -gt "$now" ]; then
    lag=$((a - now))
    log "anchor lift: system clock $now -> $a (+${lag}s)  锚点=$(date -u -d "@$a" '+%F %T UTC' 2>/dev/null)"
    if set_time "$a"; then
      log "anchor lift ok -> now $(date -u '+%F %T UTC')"
    else
      log "anchor lift FAILED (need +${lag}s; sudo 不可用?)"
    fi
  else
    log "anchor ok: system clock $now >= anchor $a (skew +$((now - a))s, 无需抬升)"
  fi
else
  log "no usable anchor at $ANCHOR (首次运行 or 文件损坏) -> 不抬升，等外部时间源"
fi

# ============ 阶段 1：布置 NTP（网络一起来就自动对时，主力）============
drop=/run/systemd/timesyncd.conf.d
if [ "$DRY" = "1" ]; then
  log "  [DRY] skip NTP 布置（不重启 timesyncd）"
elif sudo -n mkdir -p "$drop" 2>/dev/null \
   && printf '[Time]\nNTP=%s\nFallbackNTP=%s\n' "$NTP_SERVERS" "$NTP_FALLBACK" \
      | sudo -n tee "$drop/10-local-ntp.conf" >/dev/null 2>&1; then
  sudo -n systemctl restart systemd-timesyncd >/dev/null 2>&1 || true
  log "timesyncd -> $NTP_SERVERS (fallback: $NTP_FALLBACK)"
else
  log "cannot write $drop (no sudo?) - NTP override skipped"
fi

# ============ 阶段 2：HTTPS Date 头粗校（等网络就绪，实测 ~100s 才有默认路由）============
# 用 date 差值做真实预算（原来用固定 +5 计数，而一轮 3 条 curl 各可能占 6 秒，预算会失真）
TRUSTED=0
real=""; src=""; waited=0
t_start=$(date +%s)
while :; do
  waited=$(( $(date +%s) - t_start ))
  [ "$waited" -lt "$MAX_WAIT" ] || break
  for u in $TIME_URLS; do
    hdr=$(timeout 6 curl -skI "$u" 2>/dev/null \
          | awk 'tolower($1)=="date:" {print $2" "$3" "$4" "$5" "$6; exit}' | tr -d '\r')
    [ -n "$hdr" ] || continue
    e=$(date -u -d "$hdr" +%s 2>/dev/null) || continue
    # 合理性校验：必须在锚点窗口内，防代理返回的垃圾头
    [ "${e:-0}" -ge "$T_MIN" ] && [ "${e:-0}" -le "$T_MAX" ] && { real="$e"; src="$u"; break 2; }
  done
  sleep 5
done

if [ -n "$real" ]; then
  now=$(date +%s); off=$(( real - now )); absoff=${off#-}
  if [ "$absoff" -ge "$MIN_STEP" ]; then
    if set_time "$real"; then
      log "coarse step ${off}s (src=$src, waited ${waited}s) -> $(date -u '+%F %T UTC')"
    else
      log "coarse step FAILED (off=${off}s) - sudo 不可用?"
    fi
  else
    log "offset ${off}s < ${MIN_STEP}s (与 HTTPS 一致), no step (waited ${waited}s)"
  fi
  TRUSTED=1
else
  log "no usable Date header after ${waited}s - 交给 timesyncd + 锚点兜底"
fi

# ============ 阶段 3：常驻心跳（巡检 NTP / 防倒退 / 每 10 分钟刷新锚点）============
# 为什么「运行期刷新」而不是「熄火时写一次」：硬掉电没有关机钩子（见文件头）。
# NTP 已同步也算可信（当前设备就是 yes）。
[ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" = "yes" ] && TRUSTED=1

# 可信 ⇒ 立刻建立一次锚点（不等 10 分钟），让兜底马上生效
if [ "$TRUSTED" = "1" ]; then
  now=$(date +%s)
  if anchor_write "$now" 1; then
    log "anchor 建立/校正 -> $(date -u -d "@$now" '+%F %T UTC') (TRUSTED)"
    last_anchor=$(date +%s)
  else
    last_anchor=0
  fi
else
  last_anchor=$(date +%s)   # 尚不可信：不建锚点（防把坏时钟写成锚点）
fi

last_hb_log=0
while :; do
  sleep "$TICK"
  now=$(date +%s)

  # (a) NTP 状态（中途对上也要能捕捉到；systemd 的判据比时间范围判断可靠）
  if [ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" = "yes" ]; then
    TRUSTED=1
  fi

  # (b) 防倒退：系统时间若被谁改小（NTP 抖动 / 人为），用锚点拉回
  #     除非 NTP 已同步 —— 那时 NTP 是权威，改小八成是在纠正错误，不拉回。
  if a=$(anchor_epoch); then
    if [ "$a" -gt "$now" ] && [ "$TRUSTED" != "1" ]; then
      if set_time "$a"; then
        log "anchor pull-back: $now -> $a (检测到时间倒退)"
        now=$(date +%s)
      fi
    fi
  fi

  # (c) 每 ANCHOR_EVERY 秒刷新锚点
  if [ $(( now - last_anchor )) -ge "$ANCHOR_EVERY" ]; then
    if [ "$TRUSTED" = "1" ]; then
      # NTP 权威：允许校正（含变小），避免错误偏大的锚点把时间卡在未来
      if anchor_write "$now" 1; then
        last_anchor=$now
        if [ $(( now - last_hb_log )) -ge "$HEARTBEAT_LOG" ]; then
          log "anchor refresh -> $(date -u '+%F %T UTC')"
          last_hb_log=$now
        fi
      fi
    elif [ "$HAVE_ANCHOR" = "1" ]; then
      # 无网络但已有锚点：锚点是唯一权威，只增不减（这样跨开机不会丢掉已流逝的时间）
      if anchor_write "$now"; then
        last_anchor=$now
      fi
    fi
  fi
done
