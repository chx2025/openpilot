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
  local model
  # devicetree 的 model 帶結尾 NUL：「comma tici」(C3) / 「comma tizi」(C3X)
  model=$(tr -d '\000' < /sys/firmware/devicetree/base/model 2>/dev/null)

  case "$model" in
    *tizi)
      # dp - C3X 強制 H7，完全不信任 /persist 上的快取。
      # C3X 只有 red panda（H7），F4 只存在於 comma two / EON。但是
      # /persist/dp_dev_panda_mcu_type 會跨刷機保留：只要曾經被寫成 F4，之後每次開機
      # 都走 TICI_DOS=1 -> pandad_tici（C2 的 panda 庫），再由 set_aux_panda 把內建
      # panda 從 USB 匯流排上摘掉，症狀就是「panda 不見了」；而且重裝系統也不會復原，
      # 因為快取在 /persist 而不是 /data。這裡直接短路成 H7，不讀快取也不做慢速探測。
      export TICI_HW=1
      echo "C3X (tizi): forcing panda MCU H7, /persist cache ignored"
      export TICI_TRES=1
      # NVMe 掛載只在真的有裝置時才做，避免沒有 NVMe 時白等 mount_nvme 的輪詢
      [ -b /dev/nvme0n1p1 ] && mount_nvme
      return 0
      ;;
    *tici) ;;
    *) return 0 ;;
  esac

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

    # 優雅降級：成功才寫入快取，失敗則不寫入並繼續放行（不 exit，維持快速開機）
    if [ -n "$mcu" ]; then
      if sudo mount -o remount,rw /persist 2>/dev/null; then
        echo "$mcu" | sudo tee "$cache" >/dev/null 2>&1
        sudo mount -o remount,ro /persist 2>/dev/null
      fi
    else
      echo "WARNING: Panda MCU detection failed after $attempts attempts. TICI_DOS/TICI_TRES not set, proceeding anyway."
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

  # dp - egpu: 该端口同时承载 eGPU 底座（ASM2464，3801）。开启 eGPU 时
  # 必须保持 host 模式让底座枚举，绝不能在检测不到第二个 panda 时 echo none
  # 把端口切回 device 模式（会把已枚举的 eGPU 一起切断，见 dmesg usb 4-1 disconnect）。
  # dp(2026-10-22): 檔名不存在時（首次開機，manager 還沒把預設值寫進 /data/params）
  # 一律按宣告的預設值 "1" 處理。原本讀到空字串會讓這道保護失效，直接走到
  # 下面的 echo none，把內建 panda 從 USB 匯流排摘掉。
  local egpu=$(cat /data/params/d/dp_egpu_enabled 2>/dev/null)
  [ -z "$egpu" ] && egpu=1
  if [ "$egpu" = "1" ]; then
    echo host | sudo tee "$mode" >/dev/null 2>&1
    return 0
  fi

  echo host | sudo tee "$mode" >/dev/null 2>&1
  for _ in $(seq 1 6); do
    sleep 0.5
    if [ "$(lsusb 2>/dev/null | grep -c 'comma.ai panda')" -ge 2 ]; then
      return 0
    fi
    # dp - egpu: 检测到 eGPU 底座（3801:0001 或 ADD1:0002）也保持 host
    if lsusb 2>/dev/null | grep -qiE '3801:0001|3801:0002'; then
      return 0
    fi
  done
  # dp(2026-10-22): 以前這裡是 `echo none`，把沒有第二個 panda 的內建 panda 從
  # USB 匯流排上摘掉 —— 摘掉之後 pandad 找不到裝置，症狀是「panda 不見了」，
  # 而且看起來像硬體故障。內建 panda 永遠不該被摘掉，找不到第二個就留在 host
  # 模式即可（多一個裝置不會有害）。
  echo "dp: no second panda on the aux port, keeping it in host mode (no echo none)"
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
    # dp(2026-10-22): 只接受 opendbc 裡真的存在的車型名。原本只要參數非空就
    # export FINGERPRINT + SKIP_FW_QUERY=1：一個打錯或殘留的名字會讓整台車用錯的
    # 車型上車（按鍵全錯、車機報錯），症狀看起來像硬體故障，而且 /data 重裝前
    # 一直復現。這裡用一次 grep 對照 values.py 的車型定義，成本可忽略。
    case "$model" in
      ""|*[!A-Z0-9_]*)
        echo "WARNING: dp_dev_model_selected='$model' is malformed, ignoring"
        return 0
        ;;
    esac
    if ! grep -rqsE "^[[:space:]]*${model} = " "$DIR"/opendbc_repo/opendbc/car/*/values.py; then
      echo "WARNING: dp_dev_model_selected='$model' is not a known platform, ignoring"
      return 0
    fi
    export FINGERPRINT="$model"
    export SKIP_FW_QUERY=1
    echo "dp: forcing fingerprint $model (SKIP_FW_QUERY=1)"
  fi
}

function launch {
  [ -f "$DIR/.git/index.lock" ] && rm -f $DIR/.git/index.lock

  # Git 智慧更新機制：commit hash 比對 (快) + 本地修改保護 (近乎零成本的 stat 比對)
  #
  # LOCAL_MODIFIED 檢查沿用原生機制：只要 .git 底下有任何檔案比 .overlay_init 新，
  # 就代表使用者在本地做了修改（不論是否已 commit），此時一律跳過覆蓋更新，
  # 避免自動更新把還在寫的東西沖掉。這個檢查只是一次 find+grep，幾乎不花時間，
  # 不會拖慢開機。
  if [ ! -f "/data/.skip_overlay_check" ]; then
    LOCAL_MODIFIED=0
    if [ -f "${DIR}/.overlay_init" ]; then
      find ${DIR}/.git -newer ${DIR}/.overlay_init 2>/dev/null | grep -q '.' && LOCAL_MODIFIED=1
    fi

    if [ "$LOCAL_MODIFIED" -eq 1 ]; then
      echo "${DIR} 有本地修改（含未 commit），跳過覆蓋更新"
    else
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
  fi

  ln -sfn $(pwd) /data/pythonpath
  export PYTHONPATH="$PWD"

  # dp - jetlink: the vendored package, importable as `jetlink` from the tree root
  if [ -d jetlink_repo/jetlink ]; then
    ln -sfn jetlink_repo/jetlink jetlink
  fi

  if [ -f /AGNOS ]; then
    set_tici_hw
    set_lite_hw
    agnos_init
    set_model_fingerprint
  fi

  tmux capture-pane -pq -S-1000 > /tmp/launch_log

  cd system/manager

  # Git 智慧免編譯機制：commit hash 比對 (快) + working tree dirty 檢查 (近乎零成本)
  #
  # 只比對 commit hash 會漏掉「還沒 commit 就重開機測試」的情況（開發時常態）。
  # `git status --porcelain` 是單一指令、只掃差異，通常是毫秒等級，遠比 build.py
  # 本身快上百倍，加這個才能保證「你剛改的程式碼」一定會被編到，同時不變動未修改
  # 版本的快速跳過路徑。非 git 環境（純 prebuilt image）則退回原生 prebuilt flag，
  # 不會每次強制全編。
  CURRENT_COMMIT=$(git -C "$DIR" rev-parse HEAD 2>/dev/null)

  if [ -z "$CURRENT_COMMIT" ]; then
    if [ ! -f $DIR/prebuilt ]; then
      ./build.py || return 1
    fi
  else
    DIRTY=$(git -C "$DIR" status --porcelain --untracked-files=normal 2>/dev/null)
    CACHED_COMMIT=$(cat /data/.build_commit_cache 2>/dev/null)

    if [ -z "$DIRTY" ] && [ "$CURRENT_COMMIT" = "$CACHED_COMMIT" ]; then
      echo "Git 版本未變更且無未提交修改，安全跳過編譯階段"
    else
      echo "偵測到程式碼變更（commit 或未提交修改），開始編譯..."
      ./build.py
      BUILD_RESULT=$?
      if [ "$BUILD_RESULT" -eq 0 ]; then
        # 只在乾淨狀態下寫入快取；有 dirty 改動時故意不寫，
        # 確保下次開機（不論改動有沒有 commit）都會重新判斷。
        if [ -z "$DIRTY" ]; then
          echo "$CURRENT_COMMIT" > /data/.build_commit_cache
        else
          rm -f /data/.build_commit_cache
        fi
      else
        echo "Build failed ($BUILD_RESULT); refusing to start manager with incomplete build"
        return "$BUILD_RESULT"
      fi
    fi
  fi

  ./manager.py

  while true; do sleep 1; done
}

launch
