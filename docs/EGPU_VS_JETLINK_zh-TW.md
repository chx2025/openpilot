# eGPU 與 Jetlink：架構差異與掉幀差異說明

適用版本：openpilot-dp0111pre + jetlink v11 overlay + eGPU 移植 v3（含 `AM_POWER_LIMIT=100`、**不含** e9fa764）
硬體：comma 3X、純油車 12 V 供電
日期：2026-10-15

本文所有行為描述都對照過原始碼，引用格式為 `檔案:行號`。實測數字只有 AmyJeanes 公開的 3X 量測；本移植**尚未在實車上量過幀時間**，下文會明確標示哪些是推論。

---

## 1. 一句話結論

Jetlink 也會慢，但它把慢的幀**藏起來**（hold 重播上一幀），並且在變慢時**同一個行程內立刻換回已經暖機的小模型**。
eGPU 路徑是**同步等待**：每一幀都要等 AMD 算完才能處理下一幀，慢了就直接變成 modeld 掉幀；而且只有在**拋出例外**時才會退場，單純變慢沒有任何保護。

---

## 2. 兩條路徑的架構對照

| 項目 | eGPU（本移植） | Jetlink（v11 overlay） |
|---|---|---|
| 大模型在哪裡算 | 插在 comma 上的 USB eGPU（chestnut / ASM2464 + AMD） | 外接裝置（Jetson / Mac / iPhone）上的 jetlink-server |
| 誰供電 | comma 的 12 V 電源，經 USB-C 供給 eGPU | 外接裝置自己的電源 |
| 功耗限制 | `AM_POWER_LIMIT=100`，時脈交給 SMU 管（約 31 W、約 5 A）`dragonpilot/egpu/modeld_egpu.py` | 外接裝置自行決定，comma 端只做 warp 與收發 |
| 執行程序 | 獨立的 `modeld_egpu`（NativeProcess、全新 interpreter、vendored tinygrad 9cd40014），**取代** stock modeld | 在 stock modeld 內部，由 jetlink 的 joining model 包住小模型 |
| 小模型是否同時存在 | **否**。整個 process 只有大模型（`selfdrive/modeld/modeld.py:247-250`） | **是**。小模型永遠先開、先駕駛，大模型就緒後才換上 |
| warp | QCOM 上的 warp JIT，**每顆鏡頭各跑一次**，各自 `.numpy()` 讀回 host（`dragonpilot/egpu/runner.py:129-136`） | QCOM 上的 warp JIT，**兩顆鏡頭一次 replay**；GPU warp 時 CPU 同時打包其他輸入（`jetlink_repo/jetlink/openpilot/model_state.py` `prepare()`） |
| 跨 USB 傳輸 | warp 後的裁切影像加上數值，打包成一個 host buffer，**一次上傳**到 AMD | warp 後的裁切影像加上數值，透過 USB gadget 傳到外接裝置 |
| 歷史佇列 / state | 在 AMD 上（JIT 內部） | 在外接裝置上 |
| 一幀的等待方式 | **同步**：上傳 → 跑模型 → `.numpy()` 讀回，全部做完才往下 | **分段**：`prepare()` → `send()` → 先發布健康訊息 → `end()` 最多等到 `HOLD_FRAME` |
| 逾時處理 | 無。慢多少，就等多少 | 超過 46 ms 就 **hold**（重播上一幀輸出） |
| 變慢的退場 | **無**（只有例外才退場） | hold 過多，或掉幀比例 > 0.75%，就換回小模型 |
| 失敗的退場 | 設 `dp_egpu_failed` → process 結束 → manager 啟動 stock modeld（**跨 process，要重新載入小模型**） | 同一個 process 內換回已暖機的小模型（只要一幀） |
| 退場後能否回到大模型 | 本趟行程不行（到下一趟） | 可以：重新加入，並通過 proving frames 驗證 |

---

## 3. 為什麼掉幀表現不同

### 3.1 modeld 怎麼算「掉幀」

`selfdrive/modeld/modeld.py:357-365`：

```python
vipc_dropped_frames = max(0, meta_main.frame_id - last_vipc_frame_id - 1)
frames_dropped = frame_dropped_filter.update(min(vipc_dropped_frames, 10))   # FirstOrderFilter, rc = 10 s
frame_drop_ratio = frames_dropped / (1 + frames_dropped)
```

- 相機以 20 Hz 出幀，所以每幀的預算是 50 ms。
- modeld 處理一幀超過 50 ms，下一次 `recv` 拿到的 frame_id 就會跳號，跳過的幀就算成掉幀。
- `frameDropPerc = frame_drop_ratio × 100`（`selfdrive/modeld/fill_model_msg.py:64`）。
- `selfdrived` 在 `frameDropPerc > 20` 時發出 `modeldLagging`（`selfdrive/selfdrived/selfdrived.py:448`）。

所以關鍵只有一件事：**modeld 每一圈是否在 50 ms 內完成。**

### 3.2 eGPU：一幀的時間全部串在關鍵路徑上

`dragonpilot/egpu/runner.py:127-149` 每一幀依序做：

1. 道路鏡頭 warp（QCOM）→ `.numpy()`，**同步一次**
2. 廣角鏡頭 warp（QCOM）→ `.numpy()`，**再同步一次**
3. 打包 desire / traffic_convention / action_t
4. `input_device.copy_from(input_host)`：USB 上傳
5. `run_model(...)`：AMD 推論
6. `outputs.numpy()`：USB 讀回，**同步**
7. 解析輸出

這些步驟**沒有任何重疊**，總時間 = 1 到 7 的和，再加上 modeld 本身的工作（發布 modelV2、drivingModelData、cameraOdometry 等）。

AmyJeanes 在 3X 上量到的 AMD 大模型時間（不是本移植的量測）：

| 設定 | 每幀時間 | 輸入電流 |
|---|---|---|
| `AM_POWER_LIMIT=100`，SMU 管時脈（**本移植目前的設定**） | 約 42 ms | 約 5 A |
| e9fa764 鎖最高時脈（已取消） | 約 36.5 ms | 突波約 12 A |
| 不設 `AM_POWER_LIMIT` | 最高時脈、無功耗上限 | 最高 |

42 ms 加上兩次 QCOM warp、USB 往返和 modeld 的其他工作，**平均值就已經很接近 50 ms**。以下任何一項都會讓單幀超過預算，直接變成掉幀：

- USB 傳輸抖動（ASM2464 / USB 3 的排程）
- SMU 因溫度或功耗降頻（power limit 底下時脈本來就是浮動的）
- comma CPU 排程延遲（UI、loggerd、camerad 同時在跑）
- 第一次遇到新的 camera buffer 時的 `Tensor.from_blob`（`runner.py:131-132`；ring buffer 填滿後就不會再發生）

而且 modeld 的 egpu 分支**每一幀都跑完整推論**，就算上一幀已經掉了也一樣（`selfdrive/modeld/modeld.py:391-392`；歷史在 JIT 內，不能用 `prepare_only` 跳過）。所以一旦開始落後，就沒有餘裕追回來。

### 3.3 Jetlink：一樣會慢，但有三層機制把慢的幀擋住

**第一層：hold，單幀不讓它變成掉幀**

`jetlink_repo/jetlink/openpilot/model_state.py:45`：

```python
HOLD_FRAME = 0.046
```

- `end()` 最多只等到這一幀開始後的 46 ms。外接裝置還沒回覆，就**重播上一幀的輸出**（`model_state.py:275-281`）。
- modeld 照常在預算內發布，**相機幀不會掉**。
- 原始碼註解的說法是：held frame 只是晚一幀的規劃，和 10 秒的規劃長度、致動延遲相比可以忽略；dropped frame 則一樣是舊規劃，還會累計到 `modeldLagging`。
- `send()` 用 `skip_if_busy`：連線還在消化前面的幀時，這一幀就不送，直接 hold。

**第二層：hold 過多，判定為「落後」，換回小模型**

`model_state.py:54-56`、`326-343`：

```python
HOLDS_IN_A_ROW = 5      # 連續 5 幀 hold
HOLDS_ALLOWED = 20      # 或 10 秒內超過 20 幀 hold
HOLD_WINDOW = 10.0
PROVING_FRAMES = 20     # 剛換上大模型的前 20 幀內，任何一幀 hold 都算落後
```

**第三層：modeld 的掉幀比例超過 0.75%，換回小模型**

`jetlink_repo/jetlink/openpilot/joining.py:102`、`326-330`：

```python
DROP_LIMIT = 0.0075
...
elif self._active is not self._small and self._frame_drop_ratio > DROP_LIMIT:
    self._demote(BEHIND)
```

以 rc = 10 s 的濾波器來算，大約是「掉 1 幀可以原諒，約 6.5 秒內再掉第 2 幀就換回小模型」（原始碼註解）。

**換手本身不算掉幀**

`selfdrive/modeld/modeld.py:402-405`：`handovers` 一有變化，就把 `frame_drop_ratio` 與濾波器歸零。換手那一幀的停頓不會累計。

**換回去的成本只有一幀**

小模型一直留在同一個 process 裡，也已經暖機。除此之外，`SMALL_WARMUP_FRAMES = 3`：每次 modeld 啟動，都由小模型先跑前幾幀，因為小模型在新 process 第一次執行要約 1.3 秒。

### 3.4 小結：同樣的落後，兩邊的結果

| 情境 | eGPU | Jetlink |
|---|---|---|
| 偶爾一幀 55 ms | 掉 1 幀 | hold 1 幀，**不掉幀** |
| 平均 48 ms，尾端常超過 50 ms | 持續零星掉幀；`frameDropPerc` 慢慢累積，可能觸發 `modeldLagging` | 零星 hold；hold 太多就換回小模型，**`modeldLagging` 不會出現** |
| 平均 60 ms | 每一幀都超時，約每 6 幀就掉 1 幀 → `modeldLagging` | 每幀都 hold → 5 幀內換回小模型 |
| 推論拋出例外 | process 結束 → manager 啟動 stock modeld（跨 process，中間有空窗） | 同一個 process 內換回小模型，只要一幀 |

所以「jetlink 不會掉幀」的正確說法是：**jetlink 不讓大模型的慢反映成 modeld 的掉幀**。它要嘛用 hold 把慢的幀蓋掉，要嘛把駕駛權還給小模型。

---

## 4. 硬體面：為什麼 eGPU 本來就比較吃緊

1. **供電**：eGPU 的電來自 comma 的 12 V 供電。純油車的電源無法承受 e9fa764 鎖時脈時約 12 A 的突波，所以只能用 `AM_POWER_LIMIT=100` 讓 SMU 管時脈。代價是時脈浮動，幀時間的尾端也比較長。
2. **comma 本身的負擔**：eGPU 路徑的 warp、USB 驅動（tinygrad 的 USB+AMD 是 userspace 驅動，由 comma CPU 執行）、上傳與讀回，全部由 comma 的 CPU/GPU 執行。Jetlink 的外接裝置則把整個推論和佇列都移到別處。
3. **兩次 QCOM 同步**：本移植的 warp 是一顆鏡頭一次 JIT 呼叫，各自讀回 host；jetlink 是兩顆鏡頭一起 replay，而且 GPU warp 時 CPU 同時在打包輸入。這部分**是本移植可以再優化的地方**（推論，尚未實測量化）。

### 4.1 相機解析度與視角：3X 與 comma four (mici)

以下數值出自 `common/transformations/camera.py:49-52` 與 `system/camerad/cameras/nv12_info.py`：

| | 3X (tici, AR0231/OX03C10) | comma four (mici, OS04C10) |
|---|---|---|
| 道路鏡頭解析度 | **1928×1208**（不是 1920×1080） | 1344×760 |
| 道路鏡頭焦距 (px) | 2648 → 水平視角約 40° | 1141.5 → 水平視角約 61° |
| 廣角鏡頭 | 1928×1208，f = 567 | 1344×760，f = 425.25 |
| 一幀 NV12 buffer | **4.80 MB** | 2.43 MB |
| 兩顆鏡頭整幀 × 20 Hz | 約 192 MB/s | 約 97 MB/s |
| warp 後送進模型的裁切 | 2×6×128×256 = **393 KB**（兩台相同） | 393 KB |

- **視角不同**：warp 的轉換矩陣 = 校正 × 該鏡頭的內參，所以兩台的畫面都會被投影到同一個 512×256 的模型視野。**模型看到的輸入大小和視野相同，推論的運算量也相同。**
- **資料量大約多 2 倍**：只有在**整幀要跨 USB** 時才有影響。sunnypilot-master 預設 `FRAME_DEV=CPU`，在 eGPU 上做 warp，3X 每秒要送約 192 MB，這正是 AmyJeanes 173d90b（commaai#38862）把 warp 移到 QCOM 的原因。本移植和 jetlink 都在 comma 的 QCOM 上做 warp，**跨 USB 的只有 393 KB 裁切**，兩台一樣大。
- **對 QCOM warp 本身**：warp 是依輸出像素去取樣（nearest neighbour gather），運算量取決於輸出的 512×256，不是輸入。輸入較大主要影響記憶體快取（推論，未實測）。
- **結論**：在**同一台 3X 上**比較 eGPU 和 jetlink，兩邊做的是同一個 QCOM warp、送同樣 393 KB，所以解析度和視角**不是兩者掉幀差異的原因**。但 comma／sunnypilot 在 comma four 上的「不掉幀」說法不能直接套到 3X：3X 的整幀大 2 倍，而且 comma four 的上游預設路徑不同。Amy 的約 42 ms 是在 3X 上量的，可以直接參考。

---

## 5. 與上游 / 參考實作的分歧（明確標示）

| # | 項目 | 參考實作 | 本移植 | 說明 |
|---|---|---|---|---|
| D1 | 大模型變慢的退場 | sunnypilot modeld_v2：無；jetlink：hold 加 0.75% 退場 | 無 | 移植時比照 sunnypilot，**這就是掉幀差異的主因** |
| D2 | 功耗 | AmyJeanes e9fa764：鎖最高時脈 | `AM_POWER_LIMIT=100`，不鎖時脈 | 12 V 純油車無法負荷，刻意不採用 |
| D3 | warp | AmyJeanes 173d90b：warp 與模型融合成一個 JIT；jetlink：兩顆鏡頭一次 replay | 獨立的 QCOM warp JIT，每顆鏡頭各跑一次 | 比照 sunnypilot-master 拆成兩個 JIT；代價是多一次同步 |
| D4 | 失敗後的退場 | jetlink：同 process 換回小模型 | 跨 process，由 manager 啟動 stock modeld | eGPU 使用 vendored tinygrad，與 dp 小模型的 tinygrad 不能放在同一個 process |
| D5 | `modeldLagging` 門檻的認知 | jetlink `joining.py:93-101` 註解寫「frameDropPerc 超過 1% 就 soft-disable」 | dp0111pre `selfdrived.py:448` 的門檻是 **> 20** | **jetlink 的註解與 dp 的原始碼不符**。實際上 jetlink 的 0.75% 遠比 dp 的 lagging 門檻嚴格，等於非常早就退場 |

---

## 6. 改善方向

| 方案 | 效果 | 成本 / 風險 | 建議 |
|---|---|---|---|
| A. 在 eGPU 加 jetlink 式保護：單幀超過約 46 ms 就 hold 上一幀，連續 hold 或掉幀比例 > 0.75% 就本趟改回 stock modeld | 偶發慢幀不再變成掉幀；持續落後時主動退場，不等 `modeldLagging` | 不增加耗電。困難點：AMD 推論是同步呼叫，要 hold 就得把推論移到背景執行緒（或分成 submit / wait 兩段），改動集中在 `runner.py` 和 modeld 的 egpu 分支 | **首選** |
| B. 兩顆鏡頭的 warp 合併成一個 QCOM JIT（frames=2），只同步一次 | 省下一次 QCOM 同步加讀回（量值待實測） | 要改 `compile.py` 的 warp 產物和 `runner.py`；cache key 要升版 | 可以和 A 一起做 |
| C. 改用較小的大模型 | 同一顆 GPU，時間餘裕直接變大 | 模型能力不同 | 視需求 |
| D. AmyJeanes c432bda（reprojectd） | Amy 報告在 3X 上約 41.7 ms 且不掉幀 | 要移植 reprojectcalibd，並確認和 vendored tinygrad 相容，改動範圍大 | 評估中 |
| E. 提高 power limit / 鎖時脈 | 更快 | 電流增加，**純油車 12 V 不建議** | 不採用 |

---

## 7. 尚未驗證的事項

- 本移植在實車上的幀時間分布（p50 / p99）。
- 兩次 QCOM warp 同步的實際耗時。
- `AM_POWER_LIMIT=100` 在夏季車內溫度下的降頻程度。
- 從 `dp_egpu_failed` 到 stock modeld 開始發布之間，實際的空窗長度。

建議上路時記錄 `modelV2.modelExecutionTime` 與 `frameDropPerc`，用來決定 A / B 的優先順序。
