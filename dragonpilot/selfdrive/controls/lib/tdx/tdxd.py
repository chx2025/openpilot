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
# 工具函數：WKT 解析 & 點到線段距離
# ==========================================
def _parse_wkt_linestring(wkt_str):
    inner = re.search(r'LINESTRING\s*\((.+)\)', wkt_str, re.IGNORECASE)
    if not inner:
        raise ValueError(f"不支援的 WKT 格式: {wkt_str}")
    pairs = inner.group(1).split(',')
    return [tuple(map(float, p.strip().split())) for p in pairs]

def _point_to_segment_dist_sq(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return (px - ax) ** 2 + (py - ay) ** 2
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return (px - ax - t * dx) ** 2 + (py - ay - t * dy) ** 2

def _point_to_linestring_dist(lon, lat, coords):
    min_dist_sq = float('inf')
    for i in range(len(coords) - 1):
        d2 = _point_to_segment_dist_sq(lon, lat, coords[i][0], coords[i][1],
                                        coords[i+1][0], coords[i+1][1])
        if d2 < min_dist_sq:
            min_dist_sq = d2
    return math.sqrt(min_dist_sq)

def _linestring_bearing(coords):
    if len(coords) < 2:
        return 0.0
    lon1, lat1 = coords[0]
    lon2, lat2 = coords[-1]
    dy = lat2 - lat1
    dx = (lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2))
    return math.degrees(math.atan2(dx, dy)) % 360

def _angle_diff(a, b):
    diff = abs(a - b) % 360
    return diff if diff <= 180 else 360 - diff

# ==========================================
# 分段直線投射（每 250m 一段）
# ==========================================
def get_ahead_points(lat, lon, bearing_deg, total_m=3000, step_m=250):
    R = 6378137.0
    bearing_rad = math.radians(bearing_deg)
    lat_rad = math.radians(lat)
    points = []
    for dist in range(step_m, total_m + step_m, step_m):
        la = lat + (dist / R) * (180.0 / math.pi) * math.cos(bearing_rad)
        lo = lon + (dist / R) * (180.0 / math.pi) * math.sin(bearing_rad) / math.cos(lat_rad)
        points.append((la, lo))
    return points

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
# 1. 核心圖資配對引擎 (記憶體預先解析極速版)
# ==========================================
class LocalMapMatcher:
    def __init__(self, filepath):
        self.processed_shapes = []
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                data = json.load(f)
                raw_shapes = data.get("SectionShapes", data) if isinstance(data, dict) else data
                
                print("[系統] 正在預先解析圖資進記憶體 (徹底降低 CPU 負擔)...")
                # 開機時只做一次：把字串全部轉成數字 list，並算好航向角
                for item in raw_shapes:
                    geom_wkt = item.get('Geometry')
                    sec_id = item.get('SectionID')
                    if geom_wkt and sec_id:
                        try:
                            coords = _parse_wkt_linestring(geom_wkt)
                            self.processed_shapes.append({
                                'id': sec_id,
                                'coords': coords,
                                'bearing': _linestring_bearing(coords)
                            })
                        except:
                            continue
                print(f"[系統] 圖資載入完成！共 {len(self.processed_shapes)} 筆路段準備就緒。")
        except Exception as e:
            print(f"[圖資] 讀取 freeway_shapes.json 失敗: {e}")

    def find_current_section(self, lat, lon, threshold_meters=3000, bearing_deg=None):
        if not self.processed_shapes:
            return None
        
        best_match = None
        min_dist = float('inf')
        
        for item in self.processed_shapes:
            if bearing_deg is not None:
                if _angle_diff(bearing_deg, item['bearing']) > 90:
                    continue
                    
            # 單純做數學距離運算，不碰字串解析，速度極快
            dist_meters = _point_to_linestring_dist(lon, lat, item['coords']) * 111000
            if dist_meters < min_dist:
                min_dist = dist_meters
                best_match = item['id']
                
        if best_match and min_dist < threshold_meters:
            return best_match
        return None

    def find_ahead_section(self, lat, lon, bearing_deg, total_m=3000, step_m=250, threshold_meters=500):
        if not self.processed_shapes:
            return None
            
        points = get_ahead_points(lat, lon, bearing_deg, total_m, step_m)
        for (la, lo) in points:
            best_match = None
            min_dist = float('inf')
            
            for item in self.processed_shapes:
                if _angle_diff(bearing_deg, item['bearing']) > 90:
                    continue
                    
                dist_meters = _point_to_linestring_dist(lo, la, item['coords']) * 111000
                if dist_meters < min_dist:
                    min_dist = dist_meters
                    best_match = item['id']
                    
            if best_match and min_dist < threshold_meters:
                return best_match
        return None

# ==========================================
# 2. 高公局 XML 雙核心抓取器
# ==========================================
EVENT_TYPE_LABEL = {
    '1': '⚠️事故', '2': '🚧施工', '3': '🔴壅塞',
    '4': '🚦管制', '5': '🌧️天氣', '8': '⚠️異常'
}

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
                event_type = (event.findtext('EventType') or '').strip()
                label = EVENT_TYPE_LABEL.get(event_type, '📢')
                desc = event.findtext('Description', '未知事件')
                positions = event.findtext('Positions')
                direction = (event.findtext('.//Direction') or '').strip()

                if positions and positions.startswith('POINT('):
                    coords_str = positions.replace('POINT(', '').replace(')', '').strip().split()
                    if len(coords_str) == 2:
                        evt_lon, evt_lat = float(coords_str[0]), float(coords_str[1])
                        # 利用預先解析好的 matcher 加速找尋事件路段
                        sec_id = matcher.find_current_section(evt_lat, evt_lon, threshold_meters=2000)
                        if sec_id:
                            new_events.append({
                                'Description': f"{label} {desc}",
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
    print("🚀 啟動 tdxd 路況發布系統 (C3X 終極最佳化版)...")
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
    # 修改：網路更新頻率調整為 30 秒一次
    UPDATE_INTERVAL = 30 

    # --- 平滑與效能控制變數 ---
    last_calc_time = 0           # 控制 GPS 運算頻率
    CALC_INTERVAL = 1.0          # 降頻：每 1.0 秒才做一次圖資比對
    
    ahead_section_history = []   # 暫存陣列，用來儲存最近幾次的運算結果
    SMOOTH_COUNT = 3             # 平滑門檻：連續 3 次一樣才信任
    stable_ahead_section = None  # 最終決定發布給 UI 的前方路段
    stable_current_section = None# 最終決定發布給 UI 的當下路段

    TEST_MODE = False
    TEST_LAT = 23.089022
    TEST_LON = 120.250816
    TEST_BEARING = 0.0

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

        # 2. 核心運算：加上 1 秒鐘的時間限制 (降頻)
        if is_gps_ready and (current_time - last_calc_time >= CALC_INTERVAL):
            last_calc_time = current_time
            my_direction = bearing_to_direction(bearing)

            # 進行圖資比對 (現在是極速的記憶體運算)
            raw_current_section = matcher.find_current_section(lat, lon, threshold_meters=300, bearing_deg=bearing)
            raw_ahead_section = matcher.find_ahead_section(lat, lon, bearing, total_m=3000, step_m=250, threshold_meters=500)

            # --- 平滑過濾機制 (Debouncing) ---
            stable_current_section = raw_current_section 
            
            ahead_section_history.append(raw_ahead_section)
            if len(ahead_section_history) > SMOOTH_COUNT:
                ahead_section_history.pop(0)

            if len(ahead_section_history) == SMOOTH_COUNT and len(set(ahead_section_history)) == 1:
                stable_ahead_section = ahead_section_history[0]

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

                if evt_dir:
                    OPPOSITE = {'南向': '北向', '北向': '南向', '東向': '西向', '西向': '東向'}
                    # 若事件方向剛好是我目前方向的反向，則忽略
                    if OPPOSITE.get(my_direction) == evt_dir:
                        continue

                if stable_current_section and evt_sid == str(stable_current_section).strip():
                    current_event = (current_event + " / " + evt_desc) if current_event else evt_desc

                if stable_ahead_section and evt_sid == str(stable_ahead_section).strip():
                    ahead_event = (ahead_event + " / " + evt_desc) if ahead_event else evt_desc

            # 使用目前時間每 30 秒切換顯示「當下路段」與「前方路段」的事件
            cycle_state = int(current_time // 30) % 2
            event = msg.tdx.init('roadEvent')
            event.isActive = False
            event.description = ""
            event.sectionId = ""

            if cycle_state == 0:
                if current_event:
                    event.description = f"當下:{current_event}"
                    event.isActive = True
                elif ahead_event:
                    event.description = f"前方:{ahead_event}"
                    event.isActive = True
            else:
                if ahead_event:
                    event.description = f"前方:{ahead_event}"
                    event.isActive = True
                elif current_event:
                    event.description = f"當下:{current_event}"
                    event.isActive = True

            pm.send('tdx', msg) 

            # Console Log
            print(f"GPS: {gps_source} | 航向: {bearing:.1f}°({my_direction})")
            print(f"路段判定 -> 當下: {stable_current_section or '無'} | 前方(過濾後): {stable_ahead_section or '無'} | 前方(原始): {raw_ahead_section or '無'}")
            
            if traffic.speed > 0:
                print(f" => 前方車速顯示: {traffic.speed} km/h")
            if event.isActive:
                print(f" => 事件警告: {event.description}")
            print("-" * 40)

if __name__ == "__main__":
    main()
