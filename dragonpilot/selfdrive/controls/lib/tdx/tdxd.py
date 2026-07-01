#!/usr/bin/env python3
import os
import sys
import time
import math
import json
import re
import requests
import xml.etree.ElementTree as ET
import urllib3

# 忽略因為 verify=False 產生的 SSL 警告，適合 C3X 嵌入式環境
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

# 依據你的檔案位置，設定正確的 openpilot 路徑以載入 messaging
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), "../../../../"))
import cereal.messaging as messaging

# ==========================================
# 工具函數：開機自動下載 TDX 線形圖資 (針對 Comma 3X 網路延遲最佳化)
# ==========================================
def download_freeway_shapes_guest(filepath):
    print(f"[系統] 偵測到開機啟動，準備透過訪客額度下載最新 TDX 圖資...")
    url = "https://tdx.transportdata.tw/api/basic/v2/Road/Traffic/SectionShape/Freeway?$format=JSON"
    headers = {'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) openpilot/tdxd'}
    
    # C3X 開機時網路連線需要時間，給予 5 次重試機會 (共約 100 秒緩衝)
    for attempt in range(5):
        try:
            print(f"[系統] 第 {attempt+1}/5 次嘗試連線 TDX...")
            response = requests.get(url, headers=headers, timeout=15, verify=False)
            response.raise_for_status()
            data = response.json()
            
            # 存檔覆蓋舊圖資 (請確保腳本放在 /data 目錄下才有寫入權限)
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
                
            print(f"[系統] ✅ 圖資下載成功！已更新 {filepath} (共 {len(data)} 筆路段)")
            return # 下載成功就跳出函數，繼續執行主程式
            
        except Exception as e:
            print(f"[警告] ⚠️ 嘗試失敗 ({e})。可能是 C3X 網路尚未就緒，20秒後重試...")
            time.sleep(20)
            
    print(f"[警告] ❌ 網路連線逾時，放棄下載，將沿用本地舊有圖資檔案。")


# ==========================================
# 工具函數：WKT 解析、距離與方位角運算 (升級版)
# ==========================================
def _parse_wkt_linestring(wkt_str):
    inner = re.search(r'LINESTRING\s*\((.+)\)', wkt_str, re.IGNORECASE)
    if not inner:
        return []
    pairs = inner.group(1).split(',')
    return [tuple(map(float, p.strip().split())) for p in pairs]

def _segment_bearing(lon1, lat1, lon2, lat2):
    # 計算兩點間的局部航向角
    dy = lat2 - lat1
    dx = (lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2))
    return math.degrees(math.atan2(dx, dy)) % 360

def _point_to_segment_dist_sq_and_bearing(px, py, ax, ay, bx, by):
    # 回傳：點到線段的最短距離平方, 該線段的方位角
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq == 0:
        return (px - ax) ** 2 + (py - ay) ** 2, 0.0
    
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len_sq))
    proj_x = ax + t * dx
    proj_y = ay + t * dy
    dist_sq = (px - proj_x) ** 2 + (py - proj_y) ** 2
    bearing = _segment_bearing(ax, ay, bx, by)
    return dist_sq, bearing

def _angle_diff(a, b):
    diff = abs(a - b) % 360
    return diff if diff <= 180 else 360 - diff

def _get_distance_meters(lon1, lat1, lon2, lat2):
    # 簡單的經緯度轉公尺 (適用於短距離)
    dx = (lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2)) * 111320.0
    dy = (lat2 - lat1) * 111320.0
    return math.sqrt(dx*dx + dy*dy)

def bearing_to_direction(bearing_deg):
    b = bearing_deg % 360
    if b < 45 or b >= 315:
        return '北向'
    elif b < 135:
        return '東向'
    elif b < 225:
        return '南向'
    else:
        return '西向'

# ==========================================
# 1. 核心圖資配對引擎 (網格索引 + 局部切線 + 拓樸推演)
# ==========================================
class LocalMapMatcher:
    def __init__(self, filepath):
        self.sections = {}          # sec_id -> dict
        self.spatial_grid = {}      # (lat_idx, lon_idx) -> list of sec_ids
        self.grid_size = 0.01       # 約 1.1 公里一個網格
        self.topology_next = {}     # sec_id -> next_sec_id (連接圖資)
        
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                data = json.load(f)
                raw_shapes = data.get("SectionShapes", data) if isinstance(data, dict) else data
                
                print("[圖資] 正在建立空間網格與幾何拓樸 (純 Python 最佳化)...")
                
                # 第一階段：解析與網格化
                for item in raw_shapes:
                    geom_wkt = item.get('Geometry')
                    sec_id = item.get('SectionID')
                    if not geom_wkt or not sec_id: continue
                        
                    coords = _parse_wkt_linestring(geom_wkt)
                    if len(coords) < 2: continue
                        
                    # 計算總長度與局部線段
                    segments = []
                    total_length = 0.0
                    for i in range(len(coords) - 1):
                        p1, p2 = coords[i], coords[i+1]
                        dist = _get_distance_meters(p1[0], p1[1], p2[0], p2[1])
                        total_length += dist
                        segments.append({'p1': p1, 'p2': p2, 'dist': dist})
                        
                    self.sections[sec_id] = {
                        'id': sec_id,
                        'coords': coords,
                        'segments': segments,
                        'total_length': total_length,
                        'start_pt': coords[0],
                        'end_pt': coords[-1]
                    }
                    
                    # 放入空間網格 (Bounding Box)
                    min_lon = min(p[0] for p in coords)
                    max_lon = max(p[0] for p in coords)
                    min_lat = min(p[1] for p in coords)
                    max_lat = max(p[1] for p in coords)
                    
                    min_gx, max_gx = int(min_lon / self.grid_size), int(max_lon / self.grid_size)
                    min_gy, max_gy = int(min_lat / self.grid_size), int(max_lat / self.grid_size)
                    
                    for gx in range(min_gx, max_gx + 1):
                        for gy in range(min_gy, max_gy + 1):
                            grid_key = (gy, gx)
                            if grid_key not in self.spatial_grid:
                                self.spatial_grid[grid_key] = []
                            self.spatial_grid[grid_key].append(sec_id)
                
                # 第二階段：建立道路連通拓樸 (Topology) - 尋找下一段路
                for sec_a, data_a in self.sections.items():
                    end_pt = data_a['end_pt']
                    end_bearing = _segment_bearing(data_a['segments'][-1]['p1'][0], data_a['segments'][-1]['p1'][1], 
                                                   data_a['segments'][-1]['p2'][0], data_a['segments'][-1]['p2'][1])
                    best_next = None
                    min_gap = float('inf')
                    
                    for sec_b, data_b in self.sections.items():
                        if sec_a == sec_b: continue
                        start_pt = data_b['start_pt']
                        gap = _get_distance_meters(end_pt[0], end_pt[1], start_pt[0], start_pt[1])
                        
                        if gap < 50.0 and gap < min_gap: # 容許 50 公尺圖資斷點
                            start_bearing = _segment_bearing(data_b['segments'][0]['p1'][0], data_b['segments'][0]['p1'][1], 
                                                             data_b['segments'][0]['p2'][0], data_b['segments'][0]['p2'][1])
                            if _angle_diff(end_bearing, start_bearing) < 45: # 確保沒有接錯到迴轉道
                                best_next = sec_b
                                min_gap = gap
                                
                    if best_next:
                        self.topology_next[sec_a] = best_next

                print(f"[圖資] ✅ 載入 {len(self.sections)} 筆，建立 {len(self.topology_next)} 筆道路連結！")
        except Exception as e:
            print(f"[圖資] 讀取 freeway_shapes.json 失敗: {e}")

    def _get_candidates_from_grid(self, lat, lon, radius_km=3):
        candidates = set()
        gy_center, gx_center = int(lat / self.grid_size), int(lon / self.grid_size)
        span = math.ceil((radius_km / 111.0) / self.grid_size) 
        
        for gy in range(gy_center - span, gy_center + span + 1):
            for gx in range(gx_center - span, gx_center + span + 1):
                candidates.update(self.spatial_grid.get((gy, gx), []))
        return candidates

    def find_current_section(self, lat, lon, threshold_meters=300, bearing_deg=None):
        candidates = self._get_candidates_from_grid(lat, lon, radius_km=2)
        if not candidates:
            return None
        
        best_match = None
        min_dist = float('inf')
        
        for sec_id in candidates:
            sec_data = self.sections[sec_id]
            
            for seg in sec_data['segments']:
                dist_sq, local_bearing = _point_to_segment_dist_sq_and_bearing(
                    lon, lat, seg['p1'][0], seg['p1'][1], seg['p2'][0], seg['p2'][1])
                
                dist_meters = math.sqrt(dist_sq) * 111320.0
                
                if dist_meters < min_dist:
                    # [修正] 將限制放寬至 90 度，確保彎曲路段不會被誤濾掉
                    if bearing_deg is not None and _angle_diff(bearing_deg, local_bearing) > 60:
                        continue
                    
                    min_dist = dist_meters
                    best_match = sec_id
                    
        if best_match and min_dist < threshold_meters:
            return best_match
        return None

    def find_ahead_section(self, current_sec_id, target_distance_m=3000):
        if current_sec_id not in self.sections:
            return None
            
        accumulated_dist = 0.0
        current = current_sec_id
        
        for _ in range(20): 
            sec_len = self.sections[current]['total_length']
            if accumulated_dist + sec_len >= target_distance_m:
                return current
                
            accumulated_dist += sec_len
            
            if current in self.topology_next:
                current = self.topology_next[current]
            else:
                return current
                
        return current


# ==========================================
# 2. 高公局 XML 雙核心抓取器
# ==========================================
class FreewayDataClient:
    def __init__(self):
        self.events_url = "https://tisvcloud.freeway.gov.tw/history/motc20/LiveEvents.xml"
        self.traffic_url = "https://tisvcloud.freeway.gov.tw/history/motc20/LiveTraffic.xml"
        self.headers = {'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64)'}
        self.cached_events = []
        self.cached_speeds = {}

    def update_data(self, matcher):
        print("[網路] 🔄 背景更新高公局 LiveEvents & LiveTraffic...")
        try:
            res_evt = requests.get(self.events_url, headers=self.headers, timeout=5, verify=False)
            res_evt.encoding = 'utf-8'
            xml_evt = re.sub(r'\sxmlns="[^"]+"', '', res_evt.text, count=1)
            root_evt = ET.fromstring(xml_evt)

            new_events = []
            for event in root_evt.findall('.//LiveEvent'):
                event_type = (event.findtext('EventType') or '0').strip()
                desc = event.findtext('Description', '未知事件')
                positions = event.findtext('Positions')
                direction = (event.findtext('.//Direction') or '').strip()

                if positions and positions.startswith('POINT('):
                    coords_str = positions.replace('POINT(', '').replace(')', '').strip().split()
                    if len(coords_str) == 2:
                        evt_lon, evt_lat = float(coords_str[0]), float(coords_str[1])
                        # [修正] 移除不合理的 pseudo_bearing，直接用座標尋找，後續再透過 UI 過濾反向事件
                        sec_id = matcher.find_current_section(evt_lat, evt_lon, threshold_meters=1000)
                        if sec_id:
                            new_events.append({
                                'EventType': event_type,
                                'Description': desc,
                                'SectionID': str(sec_id).strip(),
                                'Direction': direction,
                            })
            self.cached_events = new_events

            res_spd = requests.get(self.traffic_url, headers=self.headers, timeout=5, verify=False)
            res_spd.encoding = 'utf-8'
            xml_spd = re.sub(r' xmlns="[^"]+"', '', res_spd.text, count=1)
            xml_spd = re.sub(r' xmlns:xsi="[^"]+"', '', xml_spd, count=1)
            xml_spd = re.sub(r' xsi:[^=]+="[^"]+"', '', xml_spd)
            root_spd = ET.fromstring(xml_spd)

            new_speeds = {}
            for traffic in root_spd.findall('.//LiveTraffic'):
                link_id = (traffic.findtext('SectionID') or '').strip()
                speed_str = (traffic.findtext('TravelSpeed') or '').strip()
                if link_id and speed_str:
                    spd = int(speed_str)
                    if spd > 0:
                        new_speeds[link_id] = spd
            self.cached_speeds = new_speeds
            print(f"[網路] ✅ 快取 {len(self.cached_events)} 筆事件, {len(self.cached_speeds)} 筆有效車速。")
        except Exception as e:
            print(f"[網路] 更新失敗: {e}")

# ==========================================
# 3. 主循環
# ==========================================
def main():
    print("🚀 啟動 tdxd 路況發布系統 (C3X 網格拓樸完美版)...")
    pm = messaging.PubMaster(['tdx'])

    BASE_DIR = os.path.dirname(os.path.realpath(__file__))
    JSON_PATH = os.path.join(BASE_DIR, "freeway_shapes.json")
    
    # ----------------------------------------------------
    # 開機網路緩衝與圖資下載
    # ----------------------------------------------------
    download_freeway_shapes_guest(JSON_PATH)

    # 讀取並預先解析圖資
    matcher = LocalMapMatcher(JSON_PATH)
    client = FreewayDataClient()
    sm = messaging.SubMaster(['liveGPS', 'gpsLocationExternal'],
                              ignore_alive=['liveGPS', 'gpsLocationExternal'])

    MAX_HORIZONTAL_ACCURACY = 50.0

    last_api_call = 0
    UPDATE_INTERVAL = 30 # 網路更新頻率

    # --- 平滑與效能控制變數 ---
    last_calc_time = 0           # 控制 GPS 運算頻率
    CALC_INTERVAL = 1.0          # 降頻：每 1.0 秒才做一次圖資比對
    
    ahead_section_history = []   # 暫存陣列，用來儲存最近幾次的運算結果
    SMOOTH_COUNT = 3             # 平滑門檻：連續 3 次一樣才信任
    stable_ahead_section = None  # 最終決定發布給 UI 的前方路段
    stable_current_section = None# 最終決定發布給 UI 的目前路段

    # --- 目前路段去抖動 (簡單版) ---
    CURRENT_SECTION_MISS_LIMIT = 2   # 連續 2 次才真正判定離開
    current_section_miss_count = 0
    
    # ==========================================
    # 測試點設定 (可自行開關) 切換回真實的車輛 GPS，只要把 TEST_MODE = True 改成 TEST_MODE = False
    # ==========================================
    TEST_MODE = False
    TEST_LAT = 24.860332
    TEST_LON = 121.218465
    TEST_BEARING = 65.0   # [修正] 符合該地真實的道路傾角
    # ==========================================

    while True:
        if TEST_MODE:
            time.sleep(0.1) 
        else:
            sm.update()
            
        current_time = time.time()

        # 1. 網路資料更新 (30秒一次)
        if current_time - last_api_call >= UPDATE_INTERVAL:
            client.update_data(matcher)
            last_api_call = current_time

        gps_source = None
        is_gps_ready = False

        if TEST_MODE:
            is_gps_ready = True
            lat, lon, bearing = TEST_LAT, TEST_LON, TEST_BEARING
            gps_source = 'TEST_MODE'
        else:
            if sm.updated.get('liveGPS', False):
                live_gps = sm['liveGPS']
                if live_gps.gpsOK and (live_gps.horizontalAccuracy <= 0 or
                                        live_gps.horizontalAccuracy <= MAX_HORIZONTAL_ACCURACY):
                    lat = live_gps.latitude
                    lon = live_gps.longitude
                    bearing = live_gps.bearingDeg
                    is_gps_ready = True
                    gps_source = 'liveGPS'

            if not is_gps_ready and sm.updated.get('gpsLocationExternal', False):
                raw_gps = sm['gpsLocationExternal']
                acc = getattr(raw_gps, 'horizontalAccuracy', 0.0)
                if acc <= 0 or acc <= MAX_HORIZONTAL_ACCURACY:
                    lat = raw_gps.latitude
                    lon = raw_gps.longitude
                    bearing = raw_gps.bearingDeg
                    is_gps_ready = True
                    gps_source = 'gpsLocationExternal(備援)'

        # 2. 核心運算：降頻與圖資比對
        if is_gps_ready and (current_time - last_calc_time >= CALC_INTERVAL):
            last_calc_time = current_time
            my_direction = bearing_to_direction(bearing)

            # 進行圖資比對
            raw_current_section = matcher.find_current_section(lat, lon, threshold_meters=300, bearing_deg=bearing)

            # --- 目前路段去抖動判定 ---
            if raw_current_section is not None:
                stable_current_section = raw_current_section
                current_section_miss_count = 0
            else:
                current_section_miss_count += 1
                if current_section_miss_count >= CURRENT_SECTION_MISS_LIMIT:
                    stable_current_section = None

            raw_ahead_section = None
            if stable_current_section is not None:
                raw_ahead_section = matcher.find_ahead_section(stable_current_section, target_distance_m=3000)
                
                ahead_section_history.append(raw_ahead_section)
                if len(ahead_section_history) > SMOOTH_COUNT:
                    ahead_section_history.pop(0)

                if len(ahead_section_history) == SMOOTH_COUNT and len(set(ahead_section_history)) == 1:
                    stable_ahead_section = ahead_section_history[0]
            else:
                ahead_section_history.clear()
                stable_ahead_section = None

            # 3. 打包與發布訊息
            msg = messaging.new_message('tdx')
            traffic = msg.tdx.init('trafficStatus')
            
            ahead_speed = client.cached_speeds.get(str(stable_ahead_section).strip(), -1) if stable_ahead_section else -1
            display_speed = ahead_speed

            if display_speed > 0:
                traffic.sectionId = str(stable_ahead_section)
                traffic.speed = int(display_speed)
                if display_speed >= 80:
                    traffic.status = "GREEN"
                elif display_speed >= 40:
                    traffic.status = "YELLOW"
                else:
                    traffic.status = "RED"
            else:
                traffic.sectionId = ""
                traffic.speed = -1
                traffic.status = "GREEN"

            # 事件設定 
            current_event = None
            ahead_event = None

            for evt in client.cached_events:
                evt_sid = evt['SectionID']
                evt_dir = evt['Direction']
                evt_desc = evt['Description']
                evt_type = evt.get('EventType', '0')

                if evt_dir:
                    OPPOSITE = {'南向': '北向', '北向': '南向', '東向': '西向', '西向': '東向'}
                    # 若事件方向剛好是我目前方向的反向，則忽略
                    if OPPOSITE.get(my_direction) == evt_dir:
                        continue
                
                formatted_evt = f"{evt_type}|{evt_desc}"

                if stable_current_section and evt_sid == str(stable_current_section).strip():
                    current_event = (current_event + "/" + formatted_evt) if current_event else formatted_evt

                if stable_ahead_section and evt_sid == str(stable_ahead_section).strip():
                    ahead_event = (ahead_event + "/" + formatted_evt) if ahead_event else formatted_evt

            # 每 10 秒切換顯示「目前路段」與「前方路段」的事件
            cycle_state = int(current_time // 10) % 2
            event = msg.tdx.init('roadEvent')
            event.isActive = False
            event.description = ""
            event.sectionId = ""

            if cycle_state == 0:
                if current_event:
                    event.description = f"目前:{current_event}"
                    event.isActive = True
                elif ahead_event:
                    event.description = f"前方:{ahead_event}"
                    event.isActive = True
            else:
                if ahead_event:
                    event.description = f"前方:{ahead_event}"
                    event.isActive = True
                elif current_event:
                    event.description = f"目前:{current_event}"
                    event.isActive = True

            pm.send('tdx', msg) 

            # Console Log
            print(f"GPS: {gps_source} | 航向: {bearing:.1f}°({my_direction})")
            print(f"路段判定 -> 目前: {stable_current_section or '無'} | 前方: {stable_ahead_section or '無'}")
            
            if traffic.speed > 0:
                print(f" => 前方車速顯示: {traffic.speed} km/h")
            if event.isActive:
                print(f" => 事件警告: {event.description}")
            print("-" * 40)

if __name__ == "__main__":
    main()
