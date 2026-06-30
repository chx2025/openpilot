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

urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), "../../../../"))
import cereal.messaging as messaging

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
    """計算 WKT 折線的整體走向角度（度），取首尾兩點"""
    if len(coords) < 2:
        return 0.0
    lon1, lat1 = coords[0]
    lon2, lat2 = coords[-1]
    dy = lat2 - lat1
    dx = (lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2))
    return math.degrees(math.atan2(dx, dy)) % 360

def _angle_diff(a, b):
    """兩個角度之間的最小差值（0~180）"""
    diff = abs(a - b) % 360
    return diff if diff <= 180 else 360 - diff

# ==========================================
# 修正 2：分段直線投射（每 500m 一段）
# ==========================================
def get_ahead_points(lat, lon, bearing_deg, total_m=3000, step_m=500):
    """沿車頭方向每 step_m 投射一個點，回傳所有中繼點（含終點）"""
    R = 6378137.0
    bearing_rad = math.radians(bearing_deg)
    lat_rad = math.radians(lat)
    points = []
    for dist in range(step_m, total_m + step_m, step_m):
        la = lat + (dist / R) * (180.0 / math.pi) * math.cos(bearing_rad)
        lo = lon + (dist / R) * (180.0 / math.pi) * math.sin(bearing_rad) / math.cos(lat_rad)
        points.append((la, lo))
    return points

# ==========================================
# 修正 1：bearing 轉行駛方向文字
# ==========================================
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
# 1. 核心圖資配對引擎
# ==========================================
class LocalMapMatcher:
    def __init__(self, filepath):
        self.shapes_data = []
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                data = json.load(f)
                self.shapes_data = data.get("SectionShapes", data) if isinstance(data, dict) else data
        except Exception as e:
            print(f"[圖資] 讀取 freeway_shapes.json 失敗: {e}")

    def find_current_section(self, lat, lon, threshold_meters=3000, bearing_deg=None):
        if not self.shapes_data:
            return None
        best_match = None
        min_dist = float('inf')
        for item in self.shapes_data:
            geom_wkt = item.get('Geometry')
            sec_id = item.get('SectionID')
            if not geom_wkt or not sec_id:
                continue
            try:
                coords = _parse_wkt_linestring(geom_wkt)
                if bearing_deg is not None:
                    seg_bearing = _linestring_bearing(coords)
                    if _angle_diff(bearing_deg, seg_bearing) > 90:
                        continue
                dist_meters = _point_to_linestring_dist(lon, lat, coords) * 111000
                if dist_meters < min_dist:
                    min_dist = dist_meters
                    best_match = sec_id
            except:
                continue
        if best_match and min_dist < threshold_meters:
            return best_match
        return None

    def find_ahead_section(self, lat, lon, bearing_deg, total_m=3000, step_m=500, threshold_meters=500):
        if not self.shapes_data:
            return None
        points = get_ahead_points(lat, lon, bearing_deg, total_m, step_m)
        for (la, lo) in points:
            best_match = None
            min_dist = float('inf')
            for item in self.shapes_data:
                geom_wkt = item.get('Geometry')
                sec_id = item.get('SectionID')
                if not geom_wkt or not sec_id:
                    continue
                try:
                    coords = _parse_wkt_linestring(geom_wkt)
                    seg_bearing = _linestring_bearing(coords)
                    if _angle_diff(bearing_deg, seg_bearing) > 90:
                        continue
                    dist_meters = _point_to_linestring_dist(lo, la, coords) * 111000
                    if dist_meters < min_dist:
                        min_dist = dist_meters
                        best_match = sec_id
                except:
                    continue
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
            # --- 事件 ---
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
                        sec_id = matcher.find_current_section(evt_lat, evt_lon, threshold_meters=2000)
                        if sec_id:
                            new_events.append({
                                'Description': f"{label} {desc}",
                                'SectionID': str(sec_id).strip(),
                                'Direction': direction,
                            })
            self.cached_events = new_events

            # --- 車速 ---
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
                    # 嚴格過濾：小於等於 0 視為無效資料，不寫入快取
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
    print("🚀 啟動 tdxd 路況發布系統...")
    pm = messaging.PubMaster(['tdx'])

    BASE_DIR = os.path.dirname(os.path.realpath(__file__))
    JSON_PATH = os.path.join(BASE_DIR, "freeway_shapes.json")
    matcher = LocalMapMatcher(JSON_PATH)
    client = FreewayDataClient()
    sm = messaging.SubMaster(['gpsLocationExternal'])

    last_api_call = 0
    UPDATE_INTERVAL = 60

    # ==========================================
    # 測試點設定 (可自行開關) 切換回真實的車輛 GPS，只要把 TEST_MODE = True 改成 TEST_MODE = False
    # ==========================================
    TEST_MODE = False
    TEST_LAT = 23.089022
    TEST_LON = 120.250816
    TEST_BEARING = 0.0
    # ==========================================

    while True:
        if TEST_MODE:
            time.sleep(1)
        else:
            sm.update()
            
        current_time = time.time()

        if current_time - last_api_call >= UPDATE_INTERVAL:
            client.update_data(matcher)
            last_api_call = current_time

        is_gps_ready = True if TEST_MODE else sm.updated.get('gpsLocationExternal', False)

        if is_gps_ready:
            if TEST_MODE:
                lat = TEST_LAT
                lon = TEST_LON
                bearing = TEST_BEARING
            else:
                gps = sm['gpsLocationExternal']
                lat = gps.latitude
                lon = gps.longitude
                bearing = gps.bearingDeg

            my_direction = bearing_to_direction(bearing)

            current_section = matcher.find_current_section(lat, lon, threshold_meters=300, bearing_deg=bearing)
            ahead_section = matcher.find_ahead_section(lat, lon, bearing,
                                                        total_m=3000, step_m=500,
                                                        threshold_meters=500)

            msg = messaging.new_message('tdx')

            # --- 車速設定 ---
            traffic = msg.tdx.init('trafficStatus')
            ahead_speed = client.cached_speeds.get(str(ahead_section).strip(), -1) if ahead_section else -1
            current_speed = client.cached_speeds.get(str(current_section).strip(), -1) if current_section else -1
            
            # 優先採用前方車速，沒有才拿當下車速
            display_speed = ahead_speed if ahead_speed > 0 else current_speed

            # 車速大於 0 才視為需要發布的有效資料 (已取消 < 70 的限制)
            if display_speed > 0:
                traffic.sectionId = str(ahead_section) if ahead_speed > 0 else str(current_section)
                traffic.speed = int(display_speed)
                
                # 配合全速域顯示，新增 80 以上為綠燈的判斷
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

            # --- 事件設定 ---
            current_event = None
            ahead_event = None

            for evt in client.cached_events:
                evt_sid = evt['SectionID']
                evt_dir = evt['Direction']
                evt_desc = evt['Description']

                if evt_dir:
                    OPPOSITE = {'南向': '北向', '北向': '南向', '東向': '西向', '西向': '東向'}
                    if OPPOSITE.get(my_direction) == evt_dir:
                        continue

                if current_section and evt_sid == str(current_section).strip():
                    current_event = (current_event + " / " + evt_desc) if current_event else evt_desc

                if ahead_section and evt_sid == str(ahead_section).strip():
                    ahead_event = (ahead_event + " / " + evt_desc) if ahead_event else evt_desc

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

            # --- 決定是否傳送資料 ---
            # 移除 has_warning_data 的 if 判斷，強制每次都發送
            pm.send('tdx', msg) 

            print(f"航向: {bearing:.1f}°({my_direction}) | 當下: {current_section or '無'} | 前方: {ahead_section or '無'}")
            
            if traffic.speed > 0:
                print(f" => 車速顯示: {traffic.speed} km/h")
            if event.isActive:
                print(f" => 事件警告: {event.description}")
            
            if traffic.speed <= 0 and not event.isActive and TEST_MODE:
                print(f"[TEST DEBUG] lat={lat}, lon={lon} | 無事件或車速為0，已發送空狀態洗掉 UI。")
            print("-" * 40)

if __name__ == "__main__":
    main()
