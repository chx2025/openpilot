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

ensure_usb_host() {
  # dp - USB-C 口（dwc3, a600000.ssusb）始终保持 host 模式。
  #
  # 这个口同时承载 eGPU 底座（ASM2464, VID 3801）和外接 panda。旧代码在没有
  # 第二个 panda 时会 `echo none` 把口切回 device 模式，等于把 eGPU 和外接 panda
  # 一起摘掉。而内建 panda 走的是 SPI（panda_tici.Panda 先试 USB，失败转
  # spi_connect），跟这个口毫无关系 —— 所以 echo none 既救不了内建 panda，又会
  # 弄断底座的 USB 链路，纯粹是负收益。这里只保证 host，不做任何关闭动作。
  local mode="/sys/devices/platform/soc/a600000.ssusb/mode"
  [ -e "$mode" ] || return 0
  echo host | sudo tee "$mode" >/dev/null 2>&1
}

set_tici_hw() {
  local model
  # devicetree 的 model 帶結尾 NUL：「comma tici」(C3) / 「comma tizi」(C3X)。
  # 順手把兩端空白去掉，免得某個 DT 版本多一個尾隨空格就整段判定失配。
  model=$(tr -d '\000' < /sys/firmware/devicetree/base/model 2>/dev/null | sed 's/^[[:space:]]*//; s/[[:space:]]*$//')

  # dp(2026-10-22): 平台由機型串決定，不再靠探測 panda MCU 類型。
  #
  # 舊邏輯是「F4 -> TICI_DOS / H7 -> TICI_TRES」，把 panda 的 MCU 型號當成平台
  # 判據，兩頭都是錯的：
  #   1. TICI_DOS 真正的語義是「這台是 comma 3」—— 它決定 manager 載入哪套 panda
  #      庫（panda_tici 對 panda）、pandad 跑 pandad_tici 還是 pandad，modem 也靠它
  #      選 QMI 而不是 PPP。TICI_TRES 只影響 home.py 的版本字串（Lite / XLite）。
  #      拿 MCU 型號去推這兩個變數，等於用「panda 是哪個型號」回答「這台是什麼機器」。
  #   2. 判據來自 /persist/dp_dev_panda_mcu_type，而 /persist 跨重裝保留。只要曾經
  #      插過 external white panda（F4），探測就會把它寫進快取，之後每次開機都按
  #      F4 走 TICI_DOS，重裝 /data 也清不掉。
  # 現在 C3 / C3X 各自短路，完全不讀快取、也不做那 15 次 × 3 秒的慢速探測。
  case "$model" in
    *tizi)
      export TICI_HW=1
      export TICI_TRES=1
      unset TICI_DOS
      echo "dp: C3X (tizi) -> TICI_TRES=1, standard panda library"
      ;;
    *tici)
      export TICI_HW=1
      export TICI_DOS=1
      unset TICI_TRES
      echo "dp: C3 (tici) -> TICI_DOS=1, panda_tici library"
      ;;
    *)
      return 0
      ;;
  esac

  ensure_usb_host

  # 舊快取已廢棄：只在存在時提一下，方便日後看開機日誌的人知道那個 F4 從哪來。
  # 不刪它 —— 刪要 remount /persist rw，為了清一行歷史狀態不值得多擔一次風險。
  if [ -f /persist/dp_dev_panda_mcu_type ]; then
    echo "dp: NOTE /persist/dp_dev_panda_mcu_type=$(cat /persist/dp_dev_panda_mcu_type 2>/dev/null) is stale, ignored"
  fi

  # NVMe 只在真的有裝置時才掛，避免沒有 NVMe 的機器白等 mount_nvme 的輪詢
  [ -b /dev/nvme0n1p1 ] && mount_nvme
  return 0
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

seed_ssh_keys() {
  # dp - 重裝後的 SSH 救命通道。
  #
  # 車機只允許 publickey 登入，而它信任的公鑰清單就是 /data/params/d/GithubSshKeys
  # 這個 param 檔（sshd 透過 /usr/comma/setup_keys 讀它）。重裝會清空 /data，
  # 這個檔一起消失，於是形成死鎖：
  #   裝完卡在 comma logo → 進不了 UI、填不了 GitHub 使用者名稱 → SSH 也被拒 → 只能線刷。
  #
  # 這裡在 build.py 之前把倉庫自帶的公鑰種進該 param，所以不管安裝卡在哪一步，
  # SSH 都還在。只寫公鑰（本身是公開資料），param 已有內容則完全不動 ——
  # 使用者在 UI 裡填過的使用者名稱優先。
  local src="$DIR/system/hardware/tici/authorized_keys"
  local dst="/data/params/d/GithubSshKeys"

  [ -s "$src" ] || return 0
  [ -s "$dst" ] && return 0

  mkdir -p /data/params/d
  # 跟 Params.put() 一樣走「暫存檔 + rename」，避免 sshd 剛好讀到寫一半的內容。
  # 權限對齊 params 自己的慣例（mkstemp 出來的 0600）：sshd 以 root 讀得到，
  # 又不會被其他人改寫。
  if cp "$src" "$dst.tmp" 2>/dev/null; then
    chmod 600 "$dst.tmp" 2>/dev/null
    if mv -f "$dst.tmp" "$dst" 2>/dev/null; then
      echo "dp: seeded SSH authorized keys from system/hardware/tici/authorized_keys"
      return 0
    fi
  fi
  rm -f "$dst.tmp" 2>/dev/null
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

  # dp - 放在 git 覆蓋更新之後、build.py 之前：git 分支可能整個換掉，
  # 但 seed 用的是剛落地的這份 $DIR，而且一定要早於編譯階段。
  seed_ssh_keys

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
