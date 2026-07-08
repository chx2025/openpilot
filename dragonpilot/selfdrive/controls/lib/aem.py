"""
Copyright (c) 2025, Rick Lan

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, and/or sublicense,
for non-commercial purposes only, subject to the following conditions:

- The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.
- Commercial use (e.g. use in a product, service, or activity intended to
  generate revenue) is prohibited without explicit written permission from
  the copyright holder.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

import numpy as np
import json
import os

class AEM:
  def __init__(self):
    self._active = False
    
    # 預先載入並儲存座標以便進行高速向量化運算
    self.signals_lat = np.array([])
    self.signals_lon = np.array([])
    self._load_export_data()

  def _load_export_data(self):
    """
    從同目錄下的 export.json 讀取路口與紅綠燈節點
    """
    try:
      file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'export.json')
      
      lats = []
      lons = []
      if os.path.exists(file_path):
        with open(file_path, 'r', encoding='utf-8') as f:
          data = json.load(f)
          for element in data.get('elements', []):
            if element.get('type') == 'node' and 'lat' in element and 'lon' in element:
              lats.append(element['lat'])
              lons.append(element['lon'])
              
        self.signals_lat = np.array(lats)
        self.signals_lon = np.array(lons)
        print(f"[AEM] 成功載入 {len(lats)} 個座標點。")
      else:
        print(f"[AEM] 警告：找不到 export.json 檔案，路徑: {file_path}")
    except Exception as e:
      print(f"[AEM] 載入 export.json 失敗: {e}")

  def _haversine_distances(self, lat1, lon1):
    """
    使用向量化計算車輛當前位置與所有載入座標的距離 (公尺)
    """
    if len(self.signals_lat) == 0:
      return np.array([])
      
    R = 6371000.0  # 地球半徑 (公尺)
    phi1 = np.radians(lat1)
    phi2 = np.radians(self.signals_lat)
    dphi = np.radians(self.signals_lat - lat1)
    dlambda = np.radians(self.signals_lon - lon1)
    
    a = np.sin(dphi / 2.0)**2 + np.cos(phi1) * np.cos(phi2) * np.sin(dlambda / 2.0)**2
    
    a = np.clip(a, 0.0, 1.0)
    c = 2.0 * R * np.arctan2(np.sqrt(a), np.sqrt(1.0 - a))
    return c

  def update_states(self, sm):
    """
    從 SubMaster 直接獲取定速與 GPS 訊號並更新 AEM 狀態
    :param sm: SubMaster 訊息中心
    """
    # 1. 讀取並處理巡航定速設定值
    try:
      v_cruise_kph = sm['carState'].vCruise
      # V_CRUISE_UNSET 通常為 255，將其重設為 0
      if v_cruise_kph == 255:
        v_cruise_kph = 0.0
    except Exception:
      v_cruise_kph = 0.0

    # 2. 讀取並驗證 GPS 座標 (完全看齊 tdxd 雙重保障與精度檢查邏輯)
    current_lat = None
    current_lon = None
    MAX_HORIZONTAL_ACCURACY = 50.0

    try:
      is_gps_ready = False
      
      # 優先嘗試讀取 liveGPS
      if sm.valid.get('liveGPS', False):
        live_gps = sm['liveGPS']
        if live_gps.gpsOK and (live_gps.horizontalAccuracy <= 0 or live_gps.horizontalAccuracy <= MAX_HORIZONTAL_ACCURACY):
          current_lat = live_gps.latitude
          current_lon = live_gps.longitude
          if current_lat != 0.0 and current_lon != 0.0:
            is_gps_ready = True
      
      # 若 liveGPS 未就緒，退而求其次讀取 gpsLocationExternal (備援)
      if not is_gps_ready and sm.valid.get('gpsLocationExternal', False):
        raw_gps = sm['gpsLocationExternal']
        acc = getattr(raw_gps, 'horizontalAccuracy', 0.0)
        if acc <= 0 or acc <= MAX_HORIZONTAL_ACCURACY:
          current_lat = raw_gps.latitude
          current_lon = raw_gps.longitude
          if current_lat == 0.0 and current_lon == 0.0:
            current_lat = None
            current_lon = None
    except Exception:
      pass

    # 3. 安全防呆：若 GPS 資料無效或圖資未成功載入，維持一般 ACC 模式
    if current_lat is None or current_lon is None or len(self.signals_lat) == 0:
      self._active = False
      return

    # 4. 條件一：巡航定速必須低於 75 km/h
    if v_cruise_kph >= 75.0:
      self._active = False
      return

    # 5. 條件二：計算 GPS 座標是否進入任何圖資座標的 100 公尺內
    distances = self._haversine_distances(current_lat, current_lon)
    
    # 若矩陣中存在任何一個距離小於等於 100 公尺，則啟用實驗模式 (blended)
    if np.any(distances <= 100.0):
      self._active = True
    else:
      self._active = False

  def get_mode(self, current_mode):
    """
    獲取當前應使用的模式
    """
    if self._active:
      return 'blended'  
    else:
      return 'acc'
