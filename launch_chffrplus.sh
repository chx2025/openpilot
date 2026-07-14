#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

source "$DIR/launch_env.sh"

function agnos_init {
  sudo rm -f /data/etc/NetworkManager/system-connections/*.nmmeta
  rm -f /data/scons_cache/config.lock

  sudo abctl --set_success

  sudo chgrp gpu /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0
  sudo chmod 660 /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0

  if [ $(< /VERSION) != "$AGNOS_VERSION" ]; then
    AGNOS_PY="$DIR/system/hardware/tici/agnos.py"
    MANIFEST="$DIR/system/hardware/tici/agnos.json"
    if $AGNOS_PY --verify $MANIFEST; then
      sudo reboot
    fi
    $DIR/system/hardware/tici/updater $AGNOS_PY $MANIFEST
  fi
}

set_tici_hw() {
  grep -q "tici" /sys/firmware/devicetree/base/model 2>/dev/null || return 0
  export TICI_HW=1

  local cache="/persist/dp_dev_panda_mcu_type"
  local attempts=15 confirm=3
  local mcu="" count=0 last="" cur cached

  # 快取極速通道
  cached=$(cat "$cache" 2>/dev/null)
  case "$cached" in
    F4|H7) mcu="$cached"; echo "panda MCU $mcu [cached]" ;;
  esac

  # 慢速偵測通道 (快取不存在時執行)
  if [ -z "$mcu" ]; then
    echo "Querying panda MCU type..."
    for attempt in $(seq 1 "$attempts"); do
      if [ -n "$last" ]; then sleep 1; else sleep 3; fi

      case "$(python -c "from panda_tici import Panda; p = Panda(cli=False); print(p.get_mcu_type()); p.close()" 2>/dev/null)" in
        *McuType.F4*) cur="F4" ;;
        *McuType.H7*) cur="H7" ;;
        *)            cur="" ;;
      esac

      if [ -n "$cur" ] && [ "$cur" = "$last" ]; then
        count=$((count + 1))
      else
        count=1
        last="$cur"
      fi

      if [ -n "$cur" ] && [ "$count" -ge "$confirm" ]; then
        mcu="$cur"
        break
      fi
    done

    # 優雅降級：成功才寫入快取，失敗則不寫入並繼續放行
    if [ -n "$mcu" ]; then
      if sudo mount -o remount,rw /persist 2>/dev/null; then
        echo "$mcu" | sudo tee "$cache" >/dev/null 2>&1
        sudo mount -o remount,ro /persist 2>/dev/null
      fi
    else
      echo "Warning: Panda MCU detection failed after $attempts attempts. Proceeding without cache..."
    fi
  fi

  # 硬體變數指派與防禦性掛載
  if [ "$mcu" = "F4" ]; then
    mount_nvme
    export TICI_DOS=1
    set_aux_panda
  elif [ "$mcu" = "H7" ]; then
    export TICI_TRES=1
  else
    # 就算 MCU 偵測失敗，依舊嘗試掛載 NVMe，避免 F4 硬體失去錄影空間
    mount_nvme
  fi
}

set_aux_panda() {
  local mode="/sys/devices/platform/soc/a600000.ssusb/mode"
  [ -e "$mode" ] || return 0

  echo host | sudo tee "$mode" >/dev/null 2>&1
  for _ in $(seq 1 6); do
    sleep 0.5
    if [ "$(lsusb 2>/dev/null | grep -c 'comma.ai panda')" -ge 2 ]; then
      return 0
    fi
  done
  echo none | sudo tee "$mode" >/dev/null 2>&1
}

mount_nvme() {
  # 0.2秒極速高頻輪詢掛載
  for i in $(seq 1 50); do
    [ -b /dev/nvme0n1p1 ] && break
    sleep 0.2
  done

  if [ ! -b /dev/nvme0n1p1 ]; then return 0; fi
  if ! mountpoint -q /data/media/0/realdata; then mount /dev/nvme0n1p1 /data/media/0/realdata; fi

  if mountpoint -q /data/media/0/realdata; then
    OWNER="$(stat -c '%U' /data/media/0/realdata)"
    GROUP="$(stat -c '%G' /data/media/0/realdata)"
    PERM="$(stat -c '%a' /data/media/0/realdata)"
    if [ "$OWNER" != "comma" ] || [ "$GROUP" != "comma" ]; then chown comma:comma /data/media/0/realdata; fi
    if [ "$PERM" != "755" ]; then chmod 755 /data/media/0/realdata; fi
  fi
}

set_lite_hw() {
  if grep -q "tici" /sys/firmware/devicetree/base/model 2>/dev/null; then
    output=$(i2cget -y 0 0x10 0x00 2>/dev/null)
    if [ -z "$output" ]; then export LITE=1; fi
  fi
}

set_model_fingerprint() {
  local model
  model=$(cat /data/params/d/dp_dev_model_selected 2>/dev/null)
  if [ -n "$model" ] && [ "$model" != "0" ]; then
    export FINGERPRINT="$model"
    export SKIP_FW_QUERY=1
  fi
}

function launch {
  [ -f "$DIR/.git/index.lock" ] && rm -f $DIR/.git/index.lock

  # Git 智慧更新機制：只比對 Commit Hash，無阻擋標記即自動覆蓋更新
  if [ ! -f "/data/.skip_overlay_check" ]; then
    LOCAL_COMMIT=$(git -C "$DIR" rev-parse HEAD 2>/dev/null)
    STAGING_COMMIT=$(git -C "${STAGING_ROOT}/finalized" rev-parse HEAD 2>/dev/null)

    if [ -n "$STAGING_COMMIT" ] && [ "$LOCAL_COMMIT" != "$STAGING_COMMIT" ]; then
      if [ -f "${STAGING_ROOT}/finalized/.overlay_consistent" ]; then
        if [ ! -d /data/safe_staging/old_openpilot ]; then
          echo "偵測到遠端新版本 ($STAGING_COMMIT)，執行更新替換..."
          LAUNCHER_LOCATION="${BASH_SOURCE[0]}"
          mv $DIR /data/safe_staging/old_openpilot
          mv "${STAGING_ROOT}/finalized" $DIR
          cd $DIR
          unset AGNOS_VERSION
          exec "${LAUNCHER_LOCATION}"
        fi
      fi
    fi
  fi

  ln -sfn $(pwd) /data/pythonpath
  export PYTHONPATH="$PWD"

  if [ -f /AGNOS ]; then
    set_tici_hw
    set_lite_hw
    agnos_init
    set_model_fingerprint
  fi

  tmux capture-pane -pq -S-1000 > /tmp/launch_log

  cd system/manager

  # Git 智慧免編譯機制：只在程式碼有真正變動時才呼叫 build.py
  CURRENT_COMMIT=$(git -C "$DIR" rev-parse HEAD 2>/dev/null)
  CACHED_COMMIT=$(cat /data/.build_commit_cache 2>/dev/null)

  if [ -n "$CURRENT_COMMIT" ] && [ "$CURRENT_COMMIT" = "$CACHED_COMMIT" ]; then
    echo "Git 版本未變更，安全跳過編譯階段"
  else
    echo "開始完整編譯..."
    ./build.py
    # 確認編譯成功後，才寫入新的快取
    if [ $? -eq 0 ] && [ -n "$CURRENT_COMMIT" ]; then
      echo "$CURRENT_COMMIT" > /data/.build_commit_cache
    fi
  fi

  ./manager.py

  while true; do sleep 1; done
}

launch
