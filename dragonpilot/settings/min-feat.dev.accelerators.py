# Browser settings entries: all changes go through the offroad-guarded API.
ITEMS = [
  {"section": "Device", "key": "DashyEgpuSettings", "type": "link_item",
   "condition": "DASHY", "title": lambda: "eGPU 設定",
   "description": lambda: "啟用、模型下載與編譯、狀態及快取管理。",
   "action": "/pages/accelerators.html#egpu"},
  {"section": "Device", "key": "DashyJetlinkSettings", "type": "link_item",
   "condition": "DASHY", "title": lambda: "Jetlink 設定",
   "description": lambda: "USB／iOS 連線、模型選擇及外部裝置狀態。",
   "action": "/pages/accelerators.html#jetlink"},
]
