# v5.2 eGPU 修復（不含 cluster）

以提供的 openpilot-dpegpu00(2).zip 為底，選取 openpilot-dpegpu00-cluster.zip 的獨立修復。沒有整包替換成 cluster 分支。

## 修復項目

- 亮度：get_screen_brightness() 限制回報 0–100。tici 原有 40% 軟體亮度縮放、mici 100% 均保留。開機原始背光 664、最大值 1023 時，原公式回報 162，超過 screenBrightnessPercent 的 Int8 範圍；修正版回報 100，避免寫入 deviceState 時例外。
- hardwared：工作執行緒死亡時明確失敗，manager 啟用既有重啟機制。新增首次發布 deviceState 後 3 秒 offroad 等待，仍須通過原有點火與啟動條件。新增 Startup waiting 診斷。
- USB 識別：原版 tinygrad AMD 後端支援 ADD1:0001、3801:0001、ADD1:0002，與上層偵測一致。
- 模型啟動：暖機重設後同步 AMD、warp 裝置與 NPY；第一張真實相機輸出發布後才切換即時排程並標示 running。
- 故障處理：onroad USB 消失、等待首筆輸出達 150 秒、running 後輸出停滯達 5 秒，標示本趟失敗，走既有小模型 fallback。輪詢約 1 Hz，這些門檻不是可繼續駕駛控制的容許時間，也不是無縫接管保證。
- 退回小模型後禁止本趟再次選擇內建 USB 大模型或 Jetlink，避免重新連線後又回到故障路徑。
- 記錄首次推論階段及例外 traceback，方便定位 NPY hang；並不宣稱根治所有硬體／驅動停滯。
- 建置：主機後端探測限制 CUDA/QCOM/CPU 並設 30 秒逾時；建置失敗不啟動 manager；忽略已知產生檔，避免無謂重建。

## 明確排除

未加入 selfdrive/dp/cluster、cluster_d、ClusterHud 參數、TR 顯示模式、cluster 編碼器、cluster 資源或共用 USB 顯示鎖。common/params_keys.h、UI 顯示邏輯與翻譯、loggerd/SConscript 保持 v5.2 原樣。保留原始 big_driving_vision.onnx.chunkmanifest；參考版的刪除不移植。其他縱向、橫向、ACM/AEM、雷達及模型權重均未修改。

## 驗證

- 參考版 4 組測試移入：60 passed。
- 修改的 Python 與 SConscript AST 語法檢查通過；launch_chffrplus.sh 通過 bash -n。
- 稽核非選取檔案與原始 v5.2 位元組一致，沒有加入 cluster 依賴。
- 此環境沒有 comma/eGPU，未執行完整裝置編譯或實車測試。

## 使用

此為完整原始碼壓縮檔，根目錄維持 openpilot-dpegpu00/。在原 v5.2 分支上依相對位置覆蓋修改檔並正常編譯重啟；若只想更換必要檔案，可依下方清單取出。勿直接套用到已加入其他修改的版本而不比對差異。docs/EGPU_V52_FIX.patch 提供精確修改差異。

裝置驗證：冷開機先確認 deviceState 持續發布且亮度介於 0–100；首次 offroad 等待至少 3 秒後，正常點火應能通過啟動條件；確認日誌出現 first camera frame published, big model running，並持續有 modelV2/modelExt。若仍停在首頁，檢查 Startup waiting 的 blockers 與 hardwared 例外。eGPU 故障退回請在靜止測試環境驗證，不能把桌上測試視為道路安全驗證。

## 修改／新增程式檔

- `.gitignore`
- `dragonpilot/egpu/constants.py`
- `dragonpilot/egpu/manager.py`
- `dragonpilot/egpu/modeld_egpu.py`
- `dragonpilot/egpu/runner.py`
- `launch_chffrplus.sh`
- `selfdrive/modeld/SConscript`
- `selfdrive/modeld/helpers.py`
- `selfdrive/modeld/modeld.py`
- `selfdrive/modeld/tests/test_egpu_disconnect.py`
- `selfdrive/modeld/tests/test_egpu_startup.py`
- `selfdrive/modeld/tests/test_startup_guards.py`
- `selfdrive/modeld/tests/test_usbgpu_device_ids.py`
- `system/hardware/hardwared.py`
- `system/hardware/tici/hardware.py`
- `system/manager/process_config.py`
- `tinygrad_repo/tinygrad/runtime/ops_amd.py`
