# eGPU v5.2 修復版 + Jetlink 0.8.7 整合

底版：本次已修復的 eGPU v5.2（不含 cluster）。Jetlink 來源：提供的 openpilot-dp0111pre-jetlink-v12.zip，程式版本為 0.8.7。

## 整合內容

- 更新 Jetlink 套件、API 3 adapter、新版 driving_warp 建置與載入格式。
- modeld 將其 SubMaster 交給 adapter，Jetlink 每幀讀取即時控制狀態。保留 ALKA、接管與無效訊息時禁止切換的條件。
- 加入來源版 bigModelLoading／Ready／Available／LinkLost 提示定義。
- 保留 eGPU loader、獨立 tinygrad、單趟模型路由、暖機同步、首次真實輸出後才標示 running、超時及故障退回小模型。
- 保留 eGPU 全部參數、設定頁、狀態顯示、USB ID ADD1:0002 及 USB 優先條件。
- 保留亮度 0–100 回報限制、hardwared 異常重啟、首次 3 秒 offroad 等待與啟動診斷；建置保護不變。
- 不加入 cluster。車輛控制、ACM/AEM、雷達、模型權重不變。

## 使用注意

這是完整原始碼包，根目錄 openpilot-dpegpu00/。需要正常重新編譯，讓新版 Jetlink driving_warp pickle 生成；舊 warp_*.pkl 格式不能供 0.8.7 使用。請勿沿用 prebuilt 或以快取跳過這次建置。warp 建置若失敗，來源版設計會讓 Jetlink 留在小模型，需查看建置日誌與 Jetlink 提示。

eGPU 與 Jetlink 供選擇使用，不同時接管。只要 eGPU 功能開啟且 dp_egpu_seen 為真，Jetlink daemon 就不啟動，以免 eGPU 枚舉暫時消失時搶用 USB。改用手機時請先在 offroad 關閉 eGPU 功能、拔除 eGPU，再選擇 Jetlink 模式並連接手機；大模型切換仍受原控制狀態限制。eGPU 本趟失敗後維持小模型，不改接手機模型；下趟才重新評估。

## 驗證範圍

81 項測試通過：原 60 項 eGPU／亮度／啟動測試，加 21 項 API 3 介面、USB 優先、裝置識別、控制狀態與整合檢查。修改 Python 語法及 launch 腳本語法通過。eGPU 專屬目錄與硬體修復檔案保持與修復版一致。

本環境沒有 comma／eGPU／手機連線，未執行完整裝置編譯、QCOM warp 建置或實車測試。需要驗證冷開機、正常 onroad、Jetlink warp 成功、手機接入與 eGPU 故障退回。

## 相對 eGPU 修復版的修改檔案

- `docs/JETLINK_PORT_zh-TW.md`
- `dragonpilot/jetlink_adapter/SConscript`
- `dragonpilot/jetlink_adapter/__init__.py`
- `dragonpilot/jetlink_adapter/build_warp.py`
- `jetlink_repo/jetlink/__init__.py`
- `jetlink_repo/jetlink/comma/port.py`
- `jetlink_repo/jetlink/openpilot/__init__.py`
- `jetlink_repo/jetlink/openpilot/interface.py`
- `jetlink_repo/jetlink/openpilot/joining.py`
- `jetlink_repo/jetlink/openpilot/model_state.py`
- `jetlink_repo/jetlink/openpilot/models.py`
- `jetlink_repo/jetlink/openpilot/status.py`
- `jetlink_repo/jetlink/openpilot/warp.py`
- `jetlink_repo/jetlink/queues.py`
- `jetlink_repo/jetlink/registry/catalog.py`
- `jetlink_repo/jetlink/registry/lfs.py`
- `selfdrive/modeld/modeld.py`
- `selfdrive/modeld/tests/test_jetlink_egpu_integration.py`
- `selfdrive/selfdrived/events.py`
