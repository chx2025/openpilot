#!/usr/bin/env python3
# ruff: noqa: E501  # The embedded HTML/CSS/JavaScript is intentionally compact.
import json
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from openpilot.sunnypilot.selfdrive.car.tesla.validation_controller import VALIDATION_LOG_PATH
from openpilot.selfdrive.debug.tesla_turn_signal_test import (
  cancel_validation_session,
  get_validation_status,
  start_validation_session,
)
from openpilot.selfdrive.debug.device_settings import settings_snapshot, validate_and_write
from openpilot.selfdrive.debug.device_hotspot import hotspot_status, set_hotspot_enabled
from openpilot.selfdrive.debug.device_console_auth import client_is_local, console_status, require_offroad
from openpilot.selfdrive.debug.device_log_download import (
  LogSelection,
  available_log_range,
  download_filename,
  select_log_range,
  stream_log_zip,
)
from openpilot.selfdrive.debug.device_terminal import change_password, run_command, terminal_status
from openpilot.selfdrive.debug.driving_status import driving_status_enabled, driving_status_snapshot
from openpilot.selfdrive.debug.tesla_speed_button_test import SpeedButtonAction, run_validation


HOST = "0.0.0.0"
PORT = 8088
_SESSION_LOCK = threading.Lock()
_ACTIVE_WEB_TEST_ID: str | None = None
_ACTIVE_WEB_SESSION_STARTED = 0.0
WEB_SESSION_TIMEOUT_S = 20.0
_LOG_DOWNLOAD_LOCK = threading.Lock()


def _clear_active_session(test_id: str) -> None:
  global _ACTIVE_WEB_SESSION_STARTED, _ACTIVE_WEB_TEST_ID
  with _SESSION_LOCK:
    if _ACTIVE_WEB_TEST_ID == test_id:
      _ACTIVE_WEB_TEST_ID = None
      _ACTIVE_WEB_SESSION_STARTED = 0.0


def render_page() -> bytes:
  driving_tab_state = "" if driving_status_enabled() else 'disabled title="请先在设备设置中开启浏览器行驶信息"'
  return """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
  <title>车载设置</title>
  <style>
    :root { color-scheme:dark; font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif; }
    body { margin:0; background:#111827; color:#f9fafb; } main { max-width:840px; margin:auto; padding:18px 14px 42px; }
    h1 { font-size:24px; margin:4px 0; } p { color:#cbd5e1; line-height:1.45; } .tabs { display:flex; gap:8px; margin:18px 0; }
    .tab, button { border:0; border-radius:12px; font-weight:700; color:white; background:#334155; padding:12px 16px; font-size:16px; }
    .tab.active { background:#2563eb; } .notice { padding:11px 13px; border-radius:10px; margin:12px 0; background:#14532d; color:#dcfce7; }
    .notice.onroad { background:#7c2d12; color:#ffedd5; } .group { margin:22px 0 9px; color:#93c5fd; font-size:15px; }
    .category-nav { display:flex; gap:8px; overflow-x:auto; padding:4px 0 10px; position:sticky; top:0; background:#111827; z-index:1; } .category { white-space:nowrap; padding:9px 12px; font-size:14px; }
    .category.active { background:#2563eb; }
    .card { display:grid; grid-template-columns:1fr auto; gap:10px; align-items:center; background:#1e293b; border-radius:13px; padding:14px; margin:8px 0; }
    .card h2 { font-size:16px; margin:0 0 5px; } .card p { font-size:13px; margin:0; } .lock { color:#fbbf24; font-size:12px; }
    input[type=number], input[type=datetime-local], select { width:120px; padding:9px; border:1px solid #475569; border-radius:9px; background:#0f172a; color:white; font-size:16px; }
    input[type=checkbox] { width:28px; height:28px; accent-color:#2563eb; } input:disabled, select:disabled, button:disabled { opacity:.45; }
    #turn-panel { text-align:center; } .buttons { display:grid; grid-template-columns:1fr 1fr; gap:16px; margin-top:28px; }
    .turn { min-height:120px; font-size:26px; } #left { background:#2563eb; } #right { background:#ea580c; } #cancel { display:none; width:100%; min-height:64px; margin-top:16px; background:#dc2626; }
    #status { min-height:52px; margin-top:18px; color:#fde68a; white-space:pre-wrap; }
    textarea { width:100%; min-height:130px; box-sizing:border-box; padding:11px; border:1px solid #475569; border-radius:10px; background:#020617; color:#e2e8f0; font:14px ui-monospace,monospace; }
    #terminal-output { min-height:100px; max-height:420px; overflow:auto; text-align:left; white-space:pre-wrap; background:#020617; border-radius:10px; padding:12px; color:#cbd5e1; }
    .terminal-row { display:flex; gap:8px; margin:10px 0; } .terminal-row input { min-width:0; flex:1; padding:9px; border:1px solid #475569; border-radius:9px; background:#0f172a; color:white; }
    .drive-alert { white-space:pre-wrap; } #driving-canvas { display:block; width:100%; height:min(72vh,640px); margin:12px 0; border:1px solid #334155; border-radius:18px; background:#07111f; }
    .ped-coordinate-lab { display:flex; flex-wrap:wrap; align-items:center; gap:8px; margin:10px 0 4px; padding:10px 12px; border:1px solid #92400e; border-radius:11px; background:#451a0355; color:#fde68a; font-size:12px; } .ped-coordinate-lab select { width:auto; max-width:100%; font-size:13px; padding:7px; }
    .can-diagnostics { margin-top:10px; } .can-diagnostics summary { cursor:pointer; color:#94a3b8; font-size:13px; padding:8px 2px; user-select:none; }
    .can-grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(220px,1fr)); gap:9px; margin-top:10px; } .can-detail { background:#1e293b; border-radius:11px; padding:11px; color:#cbd5e1; font-size:12px; line-height:1.55; } .can-detail strong { display:block; color:#93c5fd; font-size:14px; margin-bottom:3px; } .can-detail .ok { color:#86efac; } .can-detail.warn { color:#fbbf24; }
    .log-range { display:grid; grid-template-columns:1fr 1fr; gap:12px; margin:16px 0; } .log-range label { display:grid; gap:6px; color:#cbd5e1; font-size:13px; } .log-range input { width:100%; box-sizing:border-box; }
    .log-actions { display:flex; flex-wrap:wrap; gap:10px; align-items:center; } #log-download { background:#2563eb; } #log-preview { min-height:52px; margin-top:14px; white-space:pre-wrap; }
    @media (max-width:600px) { .log-range { grid-template-columns:1fr; } }
  </style>
</head><body><main>
  <h1>车载设置</h1><p>连接设备局域网后可直接访问普通设置；任意 Bash 终端单独使用密码。</p>
  <div class="tabs"><button class="tab active" id="settings-tab" onclick="showPanel('settings')">设置</button><button class="tab" id="driving-tab" __DRIVING_TAB_STATE__ onclick="showPanel('driving')">行驶信息</button><button class="tab" id="logs-tab" onclick="showPanel('logs')">日志下载</button><button class="tab" id="turn-tab" onclick="showPanel('turn')">Tesla 验证</button><button class="tab" id="terminal-tab" onclick="showPanel('terminal')">终端</button></div>
  <section id="settings-panel"><div id="mode" class="notice">正在读取设置…</div><div id="category-nav" class="category-nav"></div><div id="settings"></div></section>
  <section id="driving-panel" hidden><h1>行驶道路视图</h1><p>只读实时视图；融合 SP 模型与 HW4 Model Y 原车 CAN，不启动视频或屏幕采集。</p><div id="driving-state" class="notice">正在连接车辆数据…</div><div class="ped-coordinate-lab"><strong>行人坐标</strong><select id="pedestrian-coordinate-mode" onchange="setPedestrianCoordinateMode(this.value)"><option value="off">关闭（默认）</option><option value="dx_forward_dy_left">dX 前后 / dY 左右</option><option value="dx_forward_dy_right">dX 前后 / -dY 左右</option><option value="dy_forward_dx_left">dY 前后 / dX 左右</option><option value="dy_forward_dx_right">dY 前后 / -dX 左右</option></select><span>黄色/蓝色/粉色对应行人 #1/#2/#3；坐标单位为米。</span></div><canvas id="driving-canvas" aria-label="预测道路轨迹与原车感知"></canvas><details id="can-diagnostics" class="can-diagnostics"><summary>CAN 诊断详情（可选）</summary><div id="can-details" class="can-grid"></div></details><div id="driving-alert" class="notice drive-alert" hidden></div></section>
  <section id="turn-panel" hidden>
    <h1>Tesla CAN 验证</h1><p>转向请求由 card 实时线程跟随原车 0x3E9 模板发送；速度按钮使用新鲜的原车 0x3C2 模板。所有发送仍受 Panda safety 限制。</p>
    <div class="buttons"><button class="turn" id="left" onclick="runTurn('left')">← 左转</button><button class="turn" id="right" onclick="runTurn('right')">右转 →</button></div>
    <div class="buttons"><button onclick="runSpeed('decrease')">− 速度按钮</button><button onclick="runSpeed('increase')">+ 速度按钮</button></div>
    <button id="cancel" onclick="cancelSession()">立即取消</button><div id="status"></div>
  </section>
  <section id="logs-panel" hidden>
    <h1>日志下载</h1><p>选择本地时间范围后，设备会将重叠路线段中的 rlog/qlog 流式打包为 ZIP 直接下载。视频文件不会包含；行驶中禁止下载。</p>
    <div id="log-state" class="notice">正在读取可用日志时间…</div>
    <div class="log-range"><label>开始时间<input id="log-start" type="datetime-local" onchange="previewLogs()"></label><label>结束时间<input id="log-end" type="datetime-local" onchange="previewLogs()"></label></div>
    <div class="log-actions"><button onclick="previewLogs()">刷新范围</button><button id="log-download" onclick="downloadLogs()" disabled>打包并下载</button></div>
    <div id="log-preview" class="notice">尚未选择日志范围</div>
  </section>
  <section id="terminal-panel" hidden>
    <h1>设备终端</h1><p>仅在设置模式（非行驶状态）且设备端显式启用后可用。终端单独使用密码；命令最长 20 秒，输出上限 64 KiB。</p>
    <div id="terminal-state" class="notice">正在检查终端状态…</div><div class="terminal-row"><input id="terminal-password" type="password" autocomplete="off" placeholder="终端密码"><button onclick="runTerminal()">运行</button></div><div class="terminal-row"><input id="terminal-new-password" type="password" autocomplete="new-password" placeholder="新密码（4-64个字符）"><button onclick="changeTerminalPassword()">修改密码</button></div>
    <textarea id="terminal-command" spellcheck="false" placeholder="git status --short"></textarea><pre id="terminal-output"></pre>
  </section>
<script>
let settingsState = null, hotspotState = null, selectedCategory = null, currentPanel = 'settings', drivingLoading = false, logsStatus = null, logsInitialized = false, logsPreviewValid = false;
function apiFetch(url, options = {}) { return fetch(url, options); }
let pedestrianCoordinateMode = localStorage.getItem('pedestrianCoordinateMode') || 'off';
function setPedestrianCoordinateMode(value) { pedestrianCoordinateMode = value; localStorage.setItem('pedestrianCoordinateMode', value); loadDrivingStatus(); }
document.getElementById('pedestrian-coordinate-mode').value = pedestrianCoordinateMode;
function showPanel(name) {
  currentPanel = name;
  document.getElementById('settings-panel').hidden = name !== 'settings'; document.getElementById('driving-panel').hidden = name !== 'driving'; document.getElementById('logs-panel').hidden = name !== 'logs'; document.getElementById('turn-panel').hidden = name !== 'turn'; document.getElementById('terminal-panel').hidden = name !== 'terminal';
  document.getElementById('settings-tab').classList.toggle('active', name === 'settings'); document.getElementById('driving-tab').classList.toggle('active', name === 'driving'); document.getElementById('logs-tab').classList.toggle('active', name === 'logs'); document.getElementById('turn-tab').classList.toggle('active', name === 'turn'); document.getElementById('terminal-tab').classList.toggle('active', name === 'terminal');
  if (name === 'driving') loadDrivingStatus();
  if (name === 'logs') loadLogStatus();
}
function element(tag, attrs = {}, text = '') { const e = document.createElement(tag); Object.assign(e, attrs); if (text) e.textContent = text; return e; }
function renderSettings(data) {
  settingsState = data; const mode = document.getElementById('mode'); mode.textContent = data.onroad ? '行驶中：只允许修改标注“行驶中可调”的设置。' : '设置模式：可修改全部白名单设置。'; mode.className = 'notice' + (data.onroad ? ' onroad' : '');
  const categories = data.menu; if (!selectedCategory || !categories.includes(selectedCategory)) selectedCategory = categories[0];
  const nav = document.getElementById('category-nav'); nav.replaceChildren(); categories.forEach(category => { const button = element('button', {className:'category' + (category === selectedCategory ? ' active' : '')}, category); button.onclick = () => { selectedCategory = category; renderSettings(data); }; nav.append(button); });
  const root = document.getElementById('settings'); root.replaceChildren(); let group = '';
  data.settings.filter(setting => setting.category === selectedCategory).sort((a,b) => a.group.localeCompare(b.group) || a.title.localeCompare(b.title)).forEach(setting => {
    if (setting.group !== group) { group = setting.group; root.append(element('div', {className:'group'}, group)); }
    const card = element('div', {className:'card'}), description = element('div'), title = element('h2', {}, setting.title || setting.key);
    description.append(title); if (setting.description) description.append(element('p', {}, setting.description));
    const locked = (data.onroad && setting.offroad_only) || setting.enabled === false;
    if (data.onroad && setting.offroad_only) description.append(element('div', {className:'lock'}, '仅设置模式可调'));
    else if (setting.enabled === false) description.append(element('div', {className:'lock'}, '当前规划器不支持此参数'));
    let control;
    if (setting.widget === 'toggle') { control = element('input', {type:'checkbox', checked:setting.value, disabled:locked}); control.onchange = () => save(setting, control.checked, control); }
    else if (setting.options) { control = element('select', {disabled:locked}); setting.options.forEach(option => control.append(element('option', {value:String(option.value), selected:option.value === setting.value}, option.label))); control.onchange = () => save(setting, control.value === '' ? '' : Number(control.value), control); }
    else { control = element('input', {type:'number', value:setting.value, min:setting.min ?? '', max:setting.max ?? '', step:setting.step ?? 1, disabled:locked}); control.onchange = () => save(setting, Number(control.value), control); }
    control.title = setting.key; card.append(description, control); root.append(card);
  });
  if (selectedCategory === '设备' && hotspotState?.available) {
    const card = element('div', {className:'card'}), description = element('div');
    description.append(element('h2', {}, '设备 Wi-Fi 热点'));
    description.append(element('p', {}, hotspotState.active ? '热点已开启。连接后访问 ' + hotspotState.url : '开启后设备会切换为热点；连接手机或电脑后访问 ' + hotspotState.url));
    const control = element('input', {type:'checkbox', checked:hotspotState.active});
    control.onchange = () => saveHotspot(control.checked, control); card.append(description, control); root.append(card);
  }
}
async function loadSettings() { try { const [settingsResponse, hotspotResponse] = await Promise.all([apiFetch('/api/settings', {cache:'no-store'}), apiFetch('/api/hotspot', {cache:'no-store'})]); if (!settingsResponse.ok) throw new Error('HTTP ' + settingsResponse.status); hotspotState = hotspotResponse.ok ? await hotspotResponse.json() : null; renderSettings(await settingsResponse.json()); } catch (e) { document.getElementById('mode').textContent = '设置读取失败：' + e; } }
async function save(setting, value, control) { control.disabled = true; try { const r = await apiFetch('/api/settings/' + encodeURIComponent(setting.key), {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({value})}); const result = await r.json(); if (!r.ok) throw new Error(result.message || 'HTTP ' + r.status); setting.value = result.value; await loadSettings(); } catch (e) { alert('保存失败：' + e); } finally { if (settingsState) renderSettings(settingsState); } }
async function saveHotspot(enabled, control) { control.disabled = true; try { const r = await apiFetch('/api/hotspot', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({enabled})}); const result = await r.json(); if (!r.ok) throw new Error(result.message || 'HTTP ' + r.status); hotspotState = result; renderSettings(settingsState); } catch (e) { alert('热点切换失败：' + e); renderSettings(settingsState); } }
loadSettings();
function localTimeInput(ms) { const d = new Date(ms), local = new Date(d.getTime() - d.getTimezoneOffset() * 60000); return local.toISOString().slice(0,16); }
function selectedLogRange() { const start = new Date(document.getElementById('log-start').value).getTime(), end = new Date(document.getElementById('log-end').value).getTime(); return {start,end}; }
function formatBytes(bytes) { if (!Number.isFinite(bytes)) return '—'; const units=['B','KiB','MiB','GiB','TiB']; let value=bytes, unit=0; while(value>=1024&&unit<units.length-1){value/=1024;unit++;} return (unit?value.toFixed(value>=10?1:2):String(value))+' '+units[unit]; }
async function loadLogStatus() { const state=document.getElementById('log-state'), button=document.getElementById('log-download'); try { const response=await apiFetch('/api/logs/status',{cache:'no-store'}), data=await response.json(); if(!response.ok)throw new Error(data.message||'HTTP '+response.status); logsStatus=data; state.className='notice'+(data.onroad?' onroad':''); if(!data.available){state.textContent='没有找到可下载的 rlog/qlog';button.disabled=true;return;} state.textContent=(data.onroad?'行驶中：仅可查看范围，禁止下载。':'设置模式：可以打包下载。')+' 可用范围：'+new Date(data.start_ms).toLocaleString()+' → '+new Date(data.end_ms).toLocaleString()+' · '+data.segment_count+' 个路线段'; if(!logsInitialized){const end=data.end_ms,start=Math.max(data.start_ms,end-30*60*1000);document.getElementById('log-start').value=localTimeInput(start);document.getElementById('log-end').value=localTimeInput(end);logsInitialized=true;} await previewLogs(); } catch(error){state.className='notice onroad';state.textContent='日志范围读取失败：'+error;button.disabled=true;} }
async function previewLogs() { const preview=document.getElementById('log-preview'),button=document.getElementById('log-download'),range=selectedLogRange(); logsPreviewValid=false;button.disabled=true;if(!Number.isFinite(range.start)||!Number.isFinite(range.end)||range.end<=range.start){preview.className='notice onroad';preview.textContent='请选择有效的开始和结束时间';return;} try {const response=await apiFetch('/api/logs/preview?start_ms='+range.start+'&end_ms='+range.end,{cache:'no-store'}),data=await response.json();if(!response.ok)throw new Error(data.message||'HTTP '+response.status);preview.className='notice';preview.textContent='命中 '+data.segment_count+' 个路线段 · '+data.file_count+' 个日志文件 · '+formatBytes(data.total_bytes)+'\\n仅包含 rlog/qlog，不包含视频。';logsPreviewValid=data.file_count>0;button.disabled=!logsPreviewValid||!logsStatus||Boolean(logsStatus.onroad);}catch(error){preview.className='notice onroad';preview.textContent='日志范围无效：'+error;} }
function downloadLogs() { if(!logsPreviewValid||logsStatus?.onroad)return;const range=selectedLogRange();window.location.assign('/api/logs/download?start_ms='+range.start+'&end_ms='+range.end); }
function drawLine(ctx, points, xScale, yScale, color, width) { if (!points.length) return; ctx.beginPath(); points.forEach(([x,y], i) => { const px = ctx.canvas.clientWidth / 2 - y * yScale, py = ctx.canvas.clientHeight - 38 - x * xScale; i ? ctx.lineTo(px, py) : ctx.moveTo(px, py); }); ctx.strokeStyle = color; ctx.lineWidth = width; ctx.stroke(); }
function drawModelLine(ctx, points, xScale, yScale, color, width) { if (!points.length) return; ctx.beginPath(); points.forEach(([x,y], i) => { const px = ctx.canvas.clientWidth / 2 + y * yScale, py = ctx.canvas.clientHeight - 38 - x * xScale; i ? ctx.lineTo(px, py) : ctx.moveTo(px, py); }); ctx.strokeStyle = color; ctx.lineWidth = width; ctx.stroke(); }
const canText = {
  lead:'前方', left:'左侧', right:'右侧', cutin:'切入', car:'车辆', truck:'卡车', motorcycle:'摩托车', bicycle:'自行车', pedestrian:'行人', unknown:'未知',
  red:'红灯', green:'绿灯', yellow:'黄灯', white:'白灯', off:'信号灯熄灭', none:'无', stop_sign:'停止标志', traffic_light:'红绿灯', yield:'让行', crosswalk:'人行横道', pedestrian_crossing:'行人过街', ramp_meter:'匝道灯', speed_bump:'减速带', speed_hump:'减速丘',
  disabled:'禁用', unavailable:'不可用', available:'可用', active:'激活', standby:'待机', aware:'感知', warning:'警告', stopping:'停车中', stopped:'已停车', continuing:'继续通行',
  map:'地图', vision:'视觉', map_and_vision:'地图+视觉', navigation:'导航', fused:'已融合', rejected:'拒绝', blacklisted:'黑名单', nominal:'正常', degraded:'降级', severely_degraded:'严重降级', fault:'故障', normal:'普通路面', enhanced:'增强路面',
  regular:'常规限速', advisory:'建议限速', dependent:'条件限速', bumps:'减速设施', class_1_major:'一级主干道', class_2:'二级道路', class_3:'三级道路', class_4:'四级道路', class_5:'五级道路', class_6_minor:'六级支路', circle:'圆形', straight:'直行',
  camera_detection:'相机区域检测', positioned_object:'CH 定位对象',
  in_lane:'车道内', lane_change_left:'向左变道', lane_change_right:'向右变道', virtual_lane:'虚拟车道', follow:'跟车', lane_change_requested:'请求变道', lane_change_in_progress:'变道中', waiting_side_obstacle:'等待侧方车辆', waiting_forward_obstacle:'等待前方车辆', lane_change_abort:'变道中止',
  open:'断开', opening:'正在断开', closing:'正在闭合', closed:'已闭合', welded:'触点粘连', blocked:'被阻止', disconnected:'未连接', no_power:'无电源', about_to_charge:'准备充电', charging:'充电中', charge_complete:'充电完成', charge_stopped:'充电停止', calibrating:'校准中', drive:'行驶', support:'供电支持', diagnostic:'诊断', down:'高压关闭', coming_up:'高压上电中', going_down:'高压下电中', up_for_drive:'行驶高压就绪', up_for_charge:'充电高压就绪', up_for_dc_charge:'直流充电高压就绪', up:'高压已开启', front_left:'左前', front_right:'右前', rear_left:'左后', rear_right:'右后', solid:'常亮', flashing:'闪烁', wait_for_stationary:'等待静止', ready:'就绪', irrational:'异常', rational:'正常', initializing:'初始化', auto:'自动', low:'低矮', high:'较高', no_object:'无障碍', pcb:'控制板', inverter:'逆变器', stator:'定子', dc_capacitor:'直流电容', heatsink:'散热器', heatsink_1:'散热器1', heatsink_2:'散热器2', heatsink_3:'散热器3', pcb_2:'控制板2', junction:'结温', t_pak_1:'功率模块1', t_pak_2:'功率模块2', inlet:'入口', stator_housing:'定子壳体', front_left_door:'左前门', front_right_door:'右前门', rear_left_door:'左后门', rear_right_door:'右后门', instrument_panel_left:'仪表台左', instrument_panel_right:'仪表台右'
};
function ct(value) { return canText[value] || String(value ?? '—').replaceAll('_', ' '); }
// The compact modelV2 top-down view and Tesla OEM diagnostics have opposite
// observed lateral signs. Keep separate transforms; sharing one mirrored SP.
function canvasPoint(canvas, x, y, xScale, yScale) { return [canvas.clientWidth / 2 - y * yScale, canvas.clientHeight - 38 - x * xScale]; }
function modelCanvasPoint(canvas, x, y, xScale, yScale) { return [canvas.clientWidth / 2 + y * yScale, canvas.clientHeight - 38 - x * xScale]; }
function roundedRect(ctx,x,y,w,h,r=8){const q=Math.min(r,w/2,h/2);ctx.beginPath();ctx.moveTo(x+q,y);ctx.lineTo(x+w-q,y);ctx.quadraticCurveTo(x+w,y,x+w,y+q);ctx.lineTo(x+w,y+h-q);ctx.quadraticCurveTo(x+w,y+h,x+w-q,y+h);ctx.lineTo(x+q,y+h);ctx.quadraticCurveTo(x,y+h,x,y+h-q);ctx.lineTo(x,y+q);ctx.quadraticCurveTo(x,y,x+q,y);ctx.closePath();}
function fillRoundedRect(ctx,x,y,w,h,color,r=8){roundedRect(ctx,x,y,w,h,r);ctx.fillStyle=color;ctx.fill();}
function drawRoadBackground(ctx,width,height,xScale){
  const sky=ctx.createLinearGradient(0,0,0,height);sky.addColorStop(0,'#13233b');sky.addColorStop(.42,'#0b1728');sky.addColorStop(1,'#020617');ctx.fillStyle=sky;ctx.fillRect(0,0,width,height);
  const horizon=58,roadBottom=Math.min(width*.47,330);const road=ctx.createLinearGradient(0,horizon,0,height);road.addColorStop(0,'#172033');road.addColorStop(1,'#090f1c');ctx.fillStyle=road;ctx.beginPath();ctx.moveTo(width/2-22,horizon);ctx.lineTo(width/2+22,horizon);ctx.lineTo(width/2+roadBottom,height);ctx.lineTo(width/2-roadBottom,height);ctx.closePath();ctx.fill();
  ctx.strokeStyle='#33415588';ctx.lineWidth=1;ctx.font='10px sans-serif';ctx.fillStyle='#64748b';for(const metres of [20,40,60,80]){const y=height-38-metres*xScale;if(y<=horizon)continue;const progress=(height-y)/(height-horizon),half=roadBottom-(roadBottom-22)*progress;ctx.beginPath();ctx.moveTo(width/2-half,y);ctx.lineTo(width/2+half,y);ctx.stroke();ctx.fillText(metres+'m',width/2+half+5,y+3);}
}
function drawObjectLabel(ctx,text,x,y,color){ctx.font='bold 11px sans-serif';const w=Math.ceil(ctx.measureText(text).width)+14;fillRoundedRect(ctx,x,y-15,w,21,'#020617dd',7);ctx.fillStyle=color;ctx.fillRect(x,y-15,3,21);ctx.fillStyle='#f8fafc';ctx.fillText(text,x+8,y);}
function drawCanVehicle(ctx, vehicle, xScale, yScale) {
  const [px, py] = canvasPoint(ctx.canvas, vehicle.x_m, vehicle.y_m, xScale, yScale); if (py < 52 || py > ctx.canvas.clientHeight - 20) return;
  const color = vehicle.relevant_for_control ? '#ef4444' : vehicle.category === 'left' ? '#60a5fa' : vehicle.category === 'right' ? '#c084fc' : '#f97316'; ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 2;
  if (vehicle.type === 'pedestrian') { ctx.beginPath(); ctx.arc(px, py - 6, 4, 0, Math.PI * 2); ctx.fill(); ctx.beginPath(); ctx.moveTo(px, py - 2); ctx.lineTo(px, py + 8); ctx.moveTo(px, py + 2); ctx.lineTo(px - 5, py + 7); ctx.moveTo(px, py + 2); ctx.lineTo(px + 5, py + 7); ctx.stroke(); }
  else if (vehicle.type === 'bicycle' || vehicle.type === 'motorcycle') { ctx.beginPath(); ctx.arc(px - 6, py + 4, 4, 0, Math.PI * 2); ctx.arc(px + 6, py + 4, 4, 0, Math.PI * 2); ctx.moveTo(px - 6, py + 4); ctx.lineTo(px, py - 4); ctx.lineTo(px + 6, py + 4); ctx.stroke(); }
  else { const w = vehicle.type === 'truck' ? 22 : 17, h = vehicle.type === 'truck' ? 29 : 22; fillRoundedRect(ctx,px-w/2,py-h/2,w,h,color,vehicle.type==='truck'?4:7);ctx.fillStyle='#0f172a';fillRoundedRect(ctx,px-w/2+3,py-h/2+3,w-6,6,'#0f172a',2); }
  const rel = Number(vehicle.relative_speed); const velocity = Number.isFinite(rel) ? '  Δv ' + rel.toFixed(0) : ''; drawObjectLabel(ctx,ct(vehicle.category)+' '+vehicle.x_m.toFixed(0)+'m'+velocity,px+12,py+4,color);
}
function drawHudChip(ctx, label, x, y, color = '#2563eb') {
  ctx.font='bold 12px sans-serif'; const w=Math.ceil(ctx.measureText(label).width)+18; fillRoundedRect(ctx,x,y-15,w,22,'#020617cc',8);ctx.fillStyle=color;ctx.fillRect(x,y-15,4,22);ctx.fillStyle='#f8fafc';ctx.fillText(label,x+10,y);return w;
}
function drawPedestrianCameraIndicators(ctx, pedestrian, width, height) {
  if (!pedestrian?.available) return;
  const front=pedestrian.front_main||pedestrian.front_fisheye||pedestrian.front_narrow, left=pedestrian.left_pillar||pedestrian.left_repeater, right=pedestrian.right_pillar||pedestrian.right_repeater;
  ctx.fillStyle='#fbbf24'; ctx.font='bold 13px sans-serif';
  if(front){ctx.textAlign='center';ctx.fillText('⚠ 前方相机发现行人',width/2,174);}
  if(left){ctx.textAlign='left';ctx.fillText('⚠ 左侧行人',12,height/2);}
  if(right){ctx.textAlign='right';ctx.fillText('右侧行人 ⚠',width-12,height/2);}
  if(pedestrian.backup){ctx.textAlign='center';ctx.fillText('⚠ 后方行人',width/2,height-58);}
  ctx.textAlign='left';
}
const pedestrianCoordinateModeLabels = {
  dx_forward_dy_left:'dX→前，dY→左', dx_forward_dy_right:'dX→前，-dY→左',
  dy_forward_dx_left:'dY→前，dX→左', dy_forward_dx_right:'dY→前，-dX→左'
};
function experimentalPedestrianPosition(slot, mode) {
  const dx=Number(slot.dx_scaled),dy=Number(slot.dy_scaled);if(!Number.isFinite(dx)||!Number.isFinite(dy))return null;
  if(mode==='dx_forward_dy_left')return {x:dx,y:dy};
  if(mode==='dx_forward_dy_right')return {x:dx,y:-dy};
  if(mode==='dy_forward_dx_left')return {x:dy,y:dx};
  if(mode==='dy_forward_dx_right')return {x:dy,y:-dx};
  return null;
}
function drawExperimentalPedestrianSlots(ctx, pedestrian, xScale, yScale, width, height) {
  if(pedestrianCoordinateMode==='off'||!pedestrian?.detected_any)return;
  const colors=['#fbbf24','#38bdf8','#f472b6'];
  const slots=(pedestrian.coordinate_slots||[]).map(slot=>({slot,pos:experimentalPedestrianPosition(slot,pedestrianCoordinateMode)})).filter(item=>item.pos&&!(item.pos.x===0&&item.pos.y===0));
  ctx.save();
  for(const {slot,pos} of slots){
    if(pos.x<0||pos.x>100||Math.abs(pos.y)>14)continue;
    const [px,py]=canvasPoint(ctx.canvas,pos.x,pos.y,xScale,yScale);if(py<52||py>height-20)continue;
    const color=colors[(slot.index-1)%colors.length];ctx.strokeStyle=color;ctx.lineWidth=2;ctx.setLineDash([4,3]);ctx.beginPath();ctx.arc(px,py,9,0,Math.PI*2);ctx.stroke();ctx.setLineDash([]);drawObjectLabel(ctx,'行人 #'+slot.index+' '+pos.x.toFixed(1)+'m/'+pos.y.toFixed(1)+'m',px+12,py+4,color);
  }
  const size=Math.min(132,width*.36),left=width-size-12,top=height-size-48,cx=left+size/2,cy=top+size/2,scale=(size-24)/25.6;
  fillRoundedRect(ctx,left,top,size,size,'#020617e8',10);ctx.strokeStyle='#64748b';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(cx,top+8);ctx.lineTo(cx,top+size-8);ctx.moveTo(left+8,cy);ctx.lineTo(left+size-8,cy);ctx.stroke();ctx.fillStyle='#cbd5e1';ctx.font='10px sans-serif';ctx.fillText('+前',cx+4,top+12);ctx.fillText('+左',left+6,cy-4);ctx.fillText(pedestrianCoordinateModeLabels[pedestrianCoordinateMode],left+7,top+size-7);
  for(const {slot,pos} of slots){const px=cx-pos.y*scale,py=cy-pos.x*scale,color=colors[(slot.index-1)%colors.length];ctx.fillStyle=color;ctx.beginPath();ctx.arc(px,py,5,0,Math.PI*2);ctx.fill();ctx.font='bold 10px sans-serif';ctx.fillText('#'+slot.index,px+7,py+3);}
  if(!slots.length){ctx.fillStyle='#94a3b8';ctx.font='11px sans-serif';ctx.fillText('坐标槽均为 0',left+22,cy+4);}
  ctx.restore();
}
function drawParkingObstacle(ctx, obstacle, xScale, yScale, width, height) {
  if (!obstacle?.valid_obstacle) return;
  let px=width/2, py=height-48; if(obstacle.x_m!=null&&obstacle.y_m!=null){[px,py]=canvasPoint(ctx.canvas,obstacle.x_m,obstacle.y_m,xScale,yScale);} else if(obstacle.collision_side==='left')px=width/2-46;else if(obstacle.collision_side==='right')px=width/2+46;else if(obstacle.collision_side==='front')py=height-82;
  ctx.strokeStyle='#fb7185';ctx.lineWidth=3;ctx.beginPath();ctx.arc(px,py,10,0,Math.PI*2);ctx.stroke();drawObjectLabel(ctx,'障碍 '+obstacle.distance_m.toFixed(1)+'m',px+14,py+4,'#fb7185');
}
function drawCanvasSummary(ctx, can, width) {
  const vehicles=can.vehicles||[], pedestrian=can.pedestrian_detection||{}, traffic=can.traffic||{}, positionedPedestrians=vehicles.filter(v=>v.type==='pedestrian').length, chips=[];
  chips.push([can.available?'原车 CAN':'等待原车 CAN',can.available?'#34d399':'#94a3b8']);
  if(vehicles.length)chips.push(['目标 '+vehicles.length,'#f97316']);
  if(positionedPedestrians)chips.push(['CH 定位行人 '+positionedPedestrians,'#fbbf24']);
  else if(pedestrian.detected_any)chips.push(['相机检测到行人','#fbbf24']);
  if(traffic.control_available||traffic.light_observation_available){const lightColor=({red:'#ef4444',green:'#22c55e',yellow:'#facc15'})[traffic.light_state]||'#94a3b8';chips.push([ct(traffic.light_state)+(traffic.control_distance_m!=null?' '+traffic.control_distance_m.toFixed(0)+'m':''),lightColor]);}
  if(can.lanes?.available)chips.push(['车道 CAN','#22d3ee']);
  let x=12,y=116; for(const [label,color] of chips){const w=drawHudChip(ctx,label,x,y,color);x+=w+6;if(x>width-105){x=12;y+=28;}}
}
function renderOptionalCanDetails(can) {
  const diagnostics=document.getElementById('can-diagnostics'),root=document.getElementById('can-details');if(diagnostics.open)renderCanDetails(can);else if(root.childElementCount)root.replaceChildren();
}
function drawTraffic(ctx, traffic, xScale, width, height, mapSign) {
  const sign = traffic?.road_sign_available ? {distance: traffic.stop_line_distance_m, color: traffic.road_sign_color, arrow: traffic.road_sign_arrow, label: traffic.road_sign_type} : null;
  const control = traffic?.control_available || traffic?.light_observation_available ? {distance: traffic.control_distance_m, color: traffic.light_state, arrow: null, label: traffic.light_state, type: traffic.control_type} : null;
  const mapLight = mapSign && mapSign.traffic_light_stop_line_distance_m != null ? {distance: mapSign.traffic_light_stop_line_distance_m, color: null, arrow: null, label: 'traffic_light', type: null, map: true} : null;
  const directional = sign && (sign.arrow === 'left' || sign.arrow === 'right' || sign.arrow === 'straight');
  const src = directional && sign.distance != null ? sign : (control && control.distance != null ? control : (sign && sign.distance != null ? sign : (mapLight || null)));
  if (!src) return; const distance = src.distance; if (distance < 0 || distance > 100) return;
  const color = ({red:'#ef4444',green:'#22c55e',yellow:'#facc15',white:'#f8fafc'})[src.color] || ({red:'#ef4444',green:'#22c55e',yellow:'#facc15',red_yellow:'#f97316'})[src.color] || '#94a3b8'; const py = height - 32 - distance * xScale;
  ctx.strokeStyle = color; ctx.lineWidth = 5; ctx.beginPath(); ctx.moveTo(width / 2 - 78, py); ctx.lineTo(width / 2 + 78, py); ctx.stroke(); ctx.fillStyle = color; ctx.font = 'bold 13px sans-serif';
  let label = ''; if (['red','green','yellow','white','off','other'].includes(src.label)) label = ct(src.label); else if (src.label === 'traffic_light' || src.label === 'stop_sign') label = ct(src.label); else if (src.type === 'crosswalk' || src.type === 'pedestrian_crossing') label = ct(src.type); else label = '红绿灯';
  const arrow = ({left:' 左转', right:' 右转', straight:' 直行', circle:' 圆灯'})[src.arrow] || '';
  ctx.fillText(label + arrow + (src.map ? ' 地图' : '') + '  ' + distance.toFixed(0) + 'm', width / 2 + 86, py + 4);
  if (traffic.control_type === 'crosswalk' || traffic.control_type === 'pedestrian_crossing') { ctx.strokeStyle = '#f8fafc'; ctx.lineWidth = 2; for (let i=-3;i<=3;i++) { ctx.beginPath(); ctx.moveTo(width/2+i*17-6,py+8); ctx.lineTo(width/2+i*17+6,py+8); ctx.stroke(); } }
}
function renderCanDetails(can) {
  const root = document.getElementById('can-details'); root.replaceChildren(); if (!can?.available) { const box = element('div',{className:'can-detail warn'},'尚未收到当前线束可达的 HW4 CAN 报文'); root.append(box); }
  const add = (title, lines) => { const box = element('div',{className:'can-detail'}); box.append(element('strong',{},title)); lines.filter(Boolean).forEach(line => box.append(element('div',{},line))); root.append(box); };
  const val = (value, unit='', digits=2) => value == null ? '—' : Number(value).toFixed(digits).replace(/\\.00$/,'') + unit;
  const yn = value => value ? '是' : '否';
  const temps = values => Object.entries(values||{}).filter(([,value])=>value!=null).map(([name,value])=>ct(name)+' '+val(value,'°C')).join(' · ');
  const n = can.navigation || {}; if(n.available){const rejects = [['导航',n.reject_navigation],['HPP',n.reject_hpp],['左车道',n.reject_left_lane],['右车道',n.reject_right_lane],['左自由空间',n.reject_left_free_space],['右自由空间',n.reject_right_free_space],['自动转向',n.reject_autosteer],['手扶方向盘',n.reject_hands_on]].filter(([,value]) => value).map(([name]) => name).join('/'); add('导航 / 地图', [n.route_active ? '路线：已激活' : '路线：未激活', '地图导航：' + (n.nav_available ? '可用' : '不可用') + (n.nav_distance_m != null ? ' · ' + n.nav_distance_m + 'm' : ''), '道路匹配：' + (n.gps_road_match ? '是' : '否'), n.next_branch_distance_m != null ? '下一分支：' + n.next_branch_distance_m + 'm' + (n.next_branch_left_off_ramp ? ' 左出口' : n.next_branch_right_off_ramp ? ' 右出口' : '') : '', n.speed_limit_unlimited ? '限速：不限速' : n.speed_limit != null ? '限速：' + n.speed_limit + ' ' + n.speed_limit_unit.toUpperCase() + ' · ' + ct(n.speed_limit_type) : '', rejects ? '拒绝项：' + rejects : '', '数据源总线：' + (n.sources || []).join(' / ')]);}
  const l = can.lanes || {}; if(l.available)add('车道 · 0x239（CH）', ['宽度：' + l.width_m + 'm · 范围：' + l.view_range_m + 'm', '左线：' + (l.left_exists ? ct(l.left_usage) : '不存在'), '右线：' + (l.right_exists ? ct(l.right_usage) : '不存在'), '数据源总线：' + (l.bus || '—')]);
  const t = can.traffic || {}, trafficObserved=t.control_available||t.light_observation_available; add('交通控制 · 0x25D', ['状态：'+(trafficObserved?ct(t.control_type):'当前无有效控制'), '灯态：'+(trafficObserved?ct(t.light_state):'--')+' · 来源：'+(trafficObserved?ct(t.control_source):'--'), '距离：'+(trafficObserved&&t.control_distance_m!=null?t.control_distance_m.toFixed(0)+' m':'--')+' · state='+(t.control_frame_fresh?t.state_machine_code:'--'), '功能状态：'+(t.control_frame_fresh?ct(t.feature_state)+' ('+t.feature_state_code+')':'--'), '数据源：'+((t.sources||[]).join(' / ')||'等待 AP-PARTY')]);
  const vehicles = can.vehicles || []; if(vehicles.length)add('周边目标 · DAS_object 0x30A（CH）', [...vehicles.map(v => ct(v.category) + ' ' + ct(v.type) + ' #' + v.track_id + '：纵向 ' + v.x_m + 'm，横向 ' + v.y_m + 'm，相对速度值 ' + v.relative_speed), '最多 7 个目标；当前仅配置独立 CH 时可用']);
  const a = can.driver_assist || {}; if(a.available)add('驾驶辅助', ['规划：' + ct(a.planner_state) + ' · 行为 ' + ct(a.behavior), '健康：' + ct(a.health) + ' · 异常等级 ' + a.health_anomaly_level, '数据源总线：' + (a.bus || '—')]);
  const rs = can.road_sign || {}; if(rs.available)add('道路标志 · 0x218（CH）', [rs.traffic_light_stop_line_distance_m != null ? '红绿灯停止线：' + rs.traffic_light_stop_line_distance_m + 'm' : '', rs.stop_sign_stop_line_distance_m != null ? '停止标志停止线：' + rs.stop_sign_stop_line_distance_m + 'm' : '', '数据源总线：' + (rs.bus || '—')]);
  const p = can.pedestrian_detection || {}, cameraNames={front_main:'前主',front_fisheye:'前鱼眼',front_narrow:'前窄',left_pillar:'左柱',left_repeater:'左镜',right_pillar:'右柱',right_repeater:'右镜',backup:'后视'}; if(p.available){const cameraList=(p.active_cameras||[]).map(name=>cameraNames[name]||name), slots=p.coordinate_slots||[]; add('行人检测 · 0x400（VEH）', ['相机区域：' + (cameraList.length?cameraList.join(' / '):'未检测到'), p.simultaneous_front_rear ? '原始前后位同时置位 · mask 0x'+Number(p.camera_mask||0).toString(16).padStart(2,'0') : '', slots.length ? '原始坐标槽：' + slots.map(s=>'#'+s.index+' dX '+s.dx_scaled+' / dY '+s.dy_scaled).join(' · ') : '', '坐标解析：' + (pedestrianCoordinateModeLabels[pedestrianCoordinateMode]||'关闭') + ' · 数据源：' + (p.bus || '—')]);}
  const f = can.front_safety || {}; if(f.available)add('前向安全 · 0x299（PARTY）', [f.valid_target ? '近距目标：' + f.target_distance_m + 'm' : '报文在线，当前无有效目标', f.relative_velocity_mps != null ? '相对速度 ' + f.relative_velocity_mps + ' m/s' : '', f.time_to_impact_s != null ? 'TTI ' + f.time_to_impact_s + 's' : '', '数据源总线：' + (f.bus || '—')]);
  const lc = can.longitudinal_shadow || {}, vp = lc.velocity_profile || {}, tp = lc.torque_profiler || {}; if(lc.available)add('Tesla 纵向状态 · 0x209（PARTY）', ['当前栈：' + (lc.current_stack || 'unknown') + '（' + (lc.current_stack_code ?? '—') + '）', tp.available ? '目标速度 ' + (tp.target_speed_kph ?? '—') + ' km/h' : '', vp.available ? '未来目标速度 ' + (vp.future_target_speed_kph ?? '—') + ' km/h' : '', '数据源总线：' + (lc.bus || '—')]);
  const po = can.parking_obstacle || {}; if(po.available)add('泊车障碍 · 0x23E（VEH）', [po.valid_obstacle ? '距离 '+val(po.distance_m,'m')+' · 方向 '+ct(po.collision_side)+' · 高度 '+ct(po.height)+' · 置信度 '+po.confidence : '报文在线，当前无可信障碍', po.valid_obstacle ? '车辆坐标：X '+val(po.x_m,'m')+' · Y '+val(po.y_m,'m')+' · off-course '+(po.off_course??'—') : '', po.valid_obstacle ? '仅直接回波：'+yn(po.direct_echo_only)+' · 未跟踪 '+val(po.untracked_time_s,'s') : '', '数据源总线：' + (po.bus || '—')]);
  if(n.available)add('导航详细 · 0x238（VEH）', ['道路等级：'+ct(n.road_class)+' · controlled access：'+yn(n.controlled_access), '国家代码：'+n.country_code+' · street count：'+n.street_count, '限速原始依赖：'+n.speed_limit_dependency+' · Botts dots：'+yn(n.accept_botts_dots), '拒绝导航 '+yn(n.reject_navigation)+' · HPP '+yn(n.reject_hpp)+' · Autosteer '+yn(n.reject_autosteer)+' · Hands-on '+yn(n.reject_hands_on), '拒绝左/右车道：'+yn(n.reject_left_lane)+' / '+yn(n.reject_right_lane)+' · 左/右自由空间：'+yn(n.reject_left_free_space)+' / '+yn(n.reject_right_free_space), 'Autosteer 受限：'+yn(n.autosteer_restricted)+' · PMM：'+yn(n.pmm_enabled)+' · SCA：'+yn(n.sca_enabled), '平行泊车：'+yn(n.parallel_autopark_enabled)+' · 垂直泊车：'+yn(n.perpendicular_autopark_enabled)]);
  const rd=can.road_disturbance||{};if(rd.available)add('路面扰动 · 0x1FC（VEH）',['索引：'+rd.index+' · 高度 '+val(rd.height_m,'m'), '范围：X '+val(rd.x0_m,'m')+' → '+val(rd.x1_m,'m')+'（跨度 '+val(rd.longitudinal_span_m,'m')+'）', '范围：Y '+val(rd.y0_m,'m')+' → '+val(rd.y1_m,'m')+'（跨度 '+val(rd.lateral_span_m,'m')+'）', '悬架高度请求：'+rd.suspension_level_request+' · 数据源：'+(rd.bus||'—')]);
  const b=can.battery_diagnostics||{};if(b.available)add('高压电池 · 0x132 / 0x212（VEH）',['母线电压 '+val(b.dc_link_voltage_v,'V')+' · 电池电流 '+val(b.pack_current_a,'A')+' · 未滤波 '+val(b.current_unfiltered_a,'A'), 'BMS：'+ct(b.state)+'（'+(b.state_code??'—')+'） · 请求 '+ct(b.requested_state), '接触器：'+ct(b.contactor_state)+' · 高压：'+ct(b.hv_state)+' · 充电：'+ct(b.charge_status), '电池输入功率 '+val(b.battery_input_power_kw,'kW')+' · 可用充电功率 '+val(b.charge_power_available_kw,'kW'), '预热允许 '+yn(b.precondition_allowed)+' · 调节请求 '+yn(b.conditioning_request)+' · HVAC 请求 '+yn(b.hvac_power_request), '行驶功率不足 '+yn(b.not_enough_power_for_drive)+' · 支持功率不足 '+yn(b.not_enough_power_for_support)+' · limp '+yn(b.limp_request), '充电请求 '+yn(b.charge_request)+' · 重试 '+(b.charge_retry_count??'—')+' · PCS PWM '+yn(b.pcs_pwm_enabled), '更新允许 '+yn(b.update_allowed)+' · 充电口 HVS MIA '+yn(b.charge_port_missing_on_hv_system)+' · 空运/陆运许可 '+yn(b.ok_to_ship_by_air)+' / '+yn(b.ok_to_ship_by_land), '数据源：'+(b.sources||[]).join(' / ')]);
  const tm=can.tpms||{};if(tm.available){const wheelNames={front_left:'左前',front_right:'右前',rear_left:'左后',rear_right:'右后'};const wheelLines=Object.entries(tm.wheels||{}).map(([name,w])=>wheelNames[name]+': 显示 '+val(w.display_pressure_bar,'bar',3)+' · 直接 '+val(w.direct_pressure_bar,'bar',3)+' / '+val(w.direct_temperature_c,'°C')+' · 上次 '+val(w.last_known_pressure_bar,'bar',3)+(w.soft_warning?' · 软告警':'')+(w.hard_warning?' · 硬告警':''));const sensorLines=(tm.sensors||[]).map(s=>'传感器 #'+s.sensor_index+' '+ct(s.location)+': '+val(s.pressure_bar,'bar',3)+' / '+val(s.temperature_c,'°C')+' · 补偿 '+val(s.temperature_compensated_pressure_bar,'bar',3)+' · 变化率 '+val(s.pressure_rate)+' · 电池 '+val(s.battery_voltage_v,'V')+' · 广播压力 '+yn(s.pressure_in_advertisement)+' · 可配置 '+yn(s.configurable_pressure));add('胎压 · 0x219 / 0x25A / 0x31F（VEH）',[...wheelLines,...sensorLines,'冷胎建议：前 '+val(tm.recommended_cold_pressure_front_bar,'bar',3)+' · 后 '+val(tm.recommended_cold_pressure_rear_bar,'bar',3),'指示灯：'+ct(tm.telltale)+' · 功能 '+ct(tm.feature_state)+' · 接近状态 '+ct(tm.proximity_state)+' · 次数 '+(tm.feature_count??'—')+' · 时间 '+val(tm.feature_time_s,'s'),'Autonomy：'+ct(tm.autonomy_status)+' · MIA '+val(tm.autonomy_mia_time_s,'s'),'数据源：'+(tm.sources||[]).join(' / ')]);}
  const dp=can.drive_power||{};if(dp.available){const power=(label,p)=>p?.available?label+': 电功率 '+val(p.electrical_power_kw,'kW')+' · 最大驱动 '+val(p.drive_power_max_kw,'kW')+' · 实际热功率 '+val(p.heat_power_actual_kw,'kW')+' / 最优 '+val(p.heat_power_optimal_kw,'kW')+' / 最大 '+val(p.heat_power_max_kw,'kW')+' · 余热请求 '+val(p.excess_heat_command_kw,'kW'):'';add('前后驱动功率 · 0x266 / 0x2E5（VEH）',[power('前驱动',dp.front),power('后驱动',dp.rear),'单位来自信号语义，需与实车功率页复核 · 数据源：'+(dp.sources||[]).join(' / ')]);}
  const dt=can.drive_temperatures||{};if(dt.available){const side=(label,s)=>{const lines=[label+' mux页：'+(s.received_pages||[]).join(','),label+'运行：'+temps(s.operating_c),label+'温度百分比：逆变器 '+val(s.operating_percent?.inverter,'%')+' · 定子 '+val(s.operating_percent?.stator,'%'),label+'散热/功率模块：'+temps(s.heatsink_and_pack_c),label+'冷却液入口：'+val(s.fluid_in_c,'°C')+' · FET burn-in '+val(s.fet_burn_in?.normal)+' / '+val(s.fet_burn_in?.additional),label+'估算：'+temps(s.estimated_c),label+'寿命估算：当前 '+val(s.life_estimates?.current_weibull_miles,'mi',0)+' · 终点 '+val(s.life_estimates?.end_of_service_weibull_miles,'mi',0)+' · 损伤比 '+val(s.life_estimates?.burn_in_damage_ratio)];return lines;};add('驱动温度 · 0x315 / 0x376（VEH）',[...side('前',dt.front),...side('后',dt.rear),'数据源：'+(dt.bus||'—')]);}
  const vt=can.vehicle_totals||{};if(vt.available)add('里程 / 能量 / 刹车温度 · 0x3B6 / 0x3D2 / 0x3FE（VEH）',['里程 '+val(vt.odometer_km,'km',3)+' · OBD drive cycle '+yn(vt.obd_drive_cycle_active), '累计放电 '+val(vt.discharge_total_kwh,'kWh',3)+' · 累计充电 '+val(vt.charge_total_kwh,'kWh',3), '刹车温度：左前 '+val(vt.brake_temperature_c?.front_left,'°C')+' · 右前 '+val(vt.brake_temperature_c?.front_right,'°C')+' · 左后 '+val(vt.brake_temperature_c?.rear_left,'°C')+' · 右后 '+val(vt.brake_temperature_c?.rear_right,'°C'), 'MCP '+val(vt.mcp_index)+' · filtered '+val(vt.mcp_index_filtered), '数据源：'+(vt.sources||[]).join(' / ')]);
  const al=can.ambient_lighting||{};if(al.available)add('氛围灯 · 0x679（VEH）',['状态：'+ct(al.enable_state)+' · 强制供电 '+yn(al.power_override)+' · 亮度 '+(al.brightness??'—'), '颜色：'+al.hex_color+' · RGB '+al.rgb.red+'/'+al.rgb.green+'/'+al.rgb.blue, '效果：'+al.effect_code+'（'+al.effect_duration_ms+'ms） · 音频可视化 '+yn(al.audio_visualizer), '目标：'+((al.targets||[]).map(ct).join(' / ')||'无')+' · 数据源：'+(al.bus||'—')]);
}
function drawDrivingGeometry(geometry, data) {
  const canvas = document.getElementById('driving-canvas'), ratio = window.devicePixelRatio || 1, width = Math.max(1, canvas.clientWidth), height = Math.max(1, canvas.clientHeight); if (canvas.width !== width * ratio || canvas.height !== height * ratio) { canvas.width = width * ratio; canvas.height = height * ratio; }
  const ctx = canvas.getContext('2d'); ctx.setTransform(ratio, 0, 0, ratio, 0, 0); ctx.clearRect(0, 0, width, height); const xScale = (height - 78) / 100, yScale = Math.min(width / 12, 31), danger = geometry.hard_brake_predicted, can = geometry.oem_can || {}; drawRoadBackground(ctx,width,height,xScale);
  ctx.setLineDash([7,7]); (geometry.edges || []).forEach(line => drawModelLine(ctx,line,xScale,yScale,'#64748b',2)); ctx.setLineDash([]); (geometry.lanes || []).forEach(line => drawModelLine(ctx,line,xScale,yScale,'#e2e8f0',2));
  const oemLanes = can.lanes || {}; if (oemLanes.available) { drawLine(ctx,oemLanes.left||[],xScale,yScale,'#22d3ee',3); drawLine(ctx,oemLanes.right||[],xScale,yScale,'#22d3ee',3); ctx.setLineDash([5,5]); drawLine(ctx,oemLanes.center||[],xScale,yScale,'#0891b2',2); ctx.setLineDash([]); }
  drawModelLine(ctx, geometry.path || [], xScale, yScale, danger ? '#ef4444' : '#22c55e', 5); const legacyTraffic=geometry.oem_traffic||{}; const traffic=can.traffic?.available?can.traffic:legacyTraffic.available?{available:true,control_available:true,light_state:['none','red','green','yellow'][legacyTraffic.light_color]||'unknown',control_distance_m:legacyTraffic.stop_line_distance,control_type:'traffic_light'}:{}; drawTraffic(ctx, traffic, xScale, width, height, can.road_sign || {});
  (can.vehicles || []).forEach(vehicle => drawCanVehicle(ctx,vehicle,xScale,yScale)); (geometry.leads || []).forEach(lead => { if ((can.vehicles||[]).some(v => v.category==='lead')) return; const [px,py]=modelCanvasPoint(canvas,lead.x,lead.y,xScale,yScale); if(py>50&&py<height){const color=danger?'#ef4444':'#f97316';fillRoundedRect(ctx,px-8,py-8,16,16,color,6);drawObjectLabel(ctx,'SP '+Math.round(lead.x)+'m',px+11,py+4,color);} });
  const ped = can.pedestrian_detection || {};
  drawPedestrianCameraIndicators(ctx,ped,width,height); drawExperimentalPedestrianSlots(ctx,ped,xScale,yScale,width,height); drawParkingObstacle(ctx,can.parking_obstacle||{},xScale,yScale,width,height);
  ctx.fillStyle='#2563eb';roundedRect(ctx,width/2-17,height-35,34,25,9);ctx.fill();ctx.fillStyle='#93c5fd';ctx.fillRect(width/2-8,height-31,16,5);fillRoundedRect(ctx,10,10,178,62,'#020617b8',12);ctx.fillStyle='#f8fafc';ctx.font='bold 21px sans-serif';ctx.fillText(data.speed_kph.toFixed(0)+' km/h',22,36);ctx.font='13px sans-serif';const mode=data.openpilot_enabled?'SP 接管':data.mads_enabled?'MADS 横向':'未接管';ctx.fillStyle=data.openpilot_enabled?'#86efac':'#cbd5e1';ctx.fillText(mode+'  ·  设定 '+data.set_speed_kph.toFixed(0),22,58);
  const fs = can.front_safety || {}; if (fs.available && fs.target_distance_m != null) { drawHudChip(ctx,'前方 '+fs.target_distance_m.toFixed(1)+'m'+(fs.time_to_impact_s!=null?' · TTI '+fs.time_to_impact_s.toFixed(1)+'s':''),12,91,fs.imminent_collision?'#ef4444':'#34d399'); }
  const nav=can.navigation||{}; if(nav.available){const panelW=Math.min(172,width*.42);fillRoundedRect(ctx,width-panelW-10,10,panelW,50,'#020617b8',12);ctx.textAlign='right';ctx.fillStyle='#dbeafe';ctx.font='bold 14px sans-serif';const branch=nav.next_branch_distance_m!=null?nav.next_branch_distance_m+'m '+(nav.next_branch_left_off_ramp?'↖ 左出口':nav.next_branch_right_off_ramp?'右出口 ↗':'下一分支'):nav.route_active?'导航路线已激活':'导航可用';ctx.fillText(branch,width-20,31);ctx.font='12px sans-serif';const limit=nav.speed_limit_unlimited?'不限速':nav.speed_limit!=null?'限速 '+nav.speed_limit+' '+nav.speed_limit_unit.toUpperCase():ct(nav.road_class);ctx.fillText(limit,width-20,50);ctx.textAlign='left';}
  if(geometry.lane_change!=='off'){ctx.fillStyle='#fbbf24';ctx.font='bold 15px sans-serif';ctx.fillText('变道 '+(geometry.lane_change_direction==='left'?'←':geometry.lane_change_direction==='right'?'→':'进行中'),width-92,70);} drawCanvasSummary(ctx,can,width); renderOptionalCanDetails(can);
}
async function loadDrivingStatus() { if (drivingLoading) return; drivingLoading = true; const state = document.getElementById('driving-state'), alert = document.getElementById('driving-alert'); try { const r = await apiFetch('/api/driving-status', {cache:'no-store'}); const data = await r.json(); if (!r.ok) throw new Error(data.message || 'HTTP ' + r.status); const connected = Object.values(data.connected).every(Boolean); state.textContent = !data.onroad ? '设置模式：等待车辆启动。' : connected ? '行驶中：车辆数据正常。' : '行驶中：部分车辆数据暂未收到。'; state.className = 'notice' + ((!data.onroad || !connected) ? ' onroad' : ''); drawDrivingGeometry(data.geometry, data); alert.hidden = !data.alert; alert.textContent = data.alert || ''; } catch (e) { state.textContent = '行驶数据读取失败：' + e; state.className = 'notice onroad'; } finally { drivingLoading = false; } }
setInterval(() => { if (currentPanel === 'driving') loadDrivingStatus(); }, 500);
async function loadTerminalStatus() { const el = document.getElementById('terminal-state'); try { const r = await apiFetch('/api/terminal/status', {cache:'no-store'}); const s = await r.json(); el.textContent = !s.terminal_enabled ? '终端未启用：请先在设置中开启。' : s.onroad ? '行驶中：请先进入设置模式。' : '终端已启用：请输入密码后运行。'; el.className = 'notice' + ((!s.terminal_enabled || s.onroad) ? ' onroad' : ''); } catch (e) { el.textContent = '终端状态读取失败：' + e; } }
const terminalPasswordInput = document.getElementById('terminal-password'); terminalPasswordInput.value = localStorage.getItem('openpilotTerminalPassword') || '';
async function runTerminal() { const password = terminalPasswordInput.value, command = document.getElementById('terminal-command').value, output = document.getElementById('terminal-output'); output.textContent = '正在运行…'; try { const r = await apiFetch('/api/terminal/exec', {method:'POST', headers:{'Content-Type':'application/json', 'X-Terminal-Password':password}, body:JSON.stringify({command})}); const result = await r.json(); if (!r.ok) throw new Error(result.message || 'HTTP ' + r.status); localStorage.setItem('openpilotTerminalPassword', password); output.textContent = `[exit ${result.exit_code}${result.timed_out ? ', timeout' : ''}${result.blocked_onroad ? ', onroad blocked' : ''}]\\n` + result.output; } catch (e) { output.textContent = '运行失败：' + e; } }
async function changeTerminalPassword() { const password = terminalPasswordInput.value, newPassword = document.getElementById('terminal-new-password').value; try { const r = await apiFetch('/api/terminal/password', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({current_password:password, new_password:newPassword})}); const result = await r.json(); if (!r.ok) throw new Error(result.message || 'HTTP ' + r.status); terminalPasswordInput.value = newPassword; localStorage.setItem('openpilotTerminalPassword', newPassword); document.getElementById('terminal-new-password').value = ''; alert('密码已修改'); } catch (e) { alert('修改失败：' + e); } }
loadTerminalStatus();
let activeTestId = null;
const phaseText = {queued:'请求已提交',waiting_vehicle_feedback:'等待车辆转向灯响应',waiting_sp_start:'等待 SP 开始变道',lane_changing:'SP 正在执行变道',cancelling:'正在关闭转向灯',confirming_cancel:'正在确认转向灯关闭'};
async function runTurn(direction) { document.querySelectorAll('#left,#right').forEach(button => button.disabled = true); const status = document.getElementById('status'); status.textContent = '正在提交…'; try { const response = await apiFetch('/api/turn/' + direction, {method:'POST'}); const result = await response.json(); if (!response.ok) throw new Error(result.message || '提交失败'); activeTestId = result.test_id; document.getElementById('cancel').style.display = 'block'; await pollStatus(); } catch (error) { finishTurnUi('请求失败：' + error); } }
async function pollStatus() { if (!activeTestId) return; try { const response = await apiFetch('/api/status/' + activeTestId, {cache:'no-store'}); const result = await response.json(); const detail = '已发送 ' + (result.action_frames_sent || 0) + ' 帧'; if (result.done) { const ok = result.result === 'PASS'; finishTurnUi((ok ? '完成：转向灯已自动关闭' : '结束：' + result.result) + '\\n' + detail); return; } document.getElementById('status').textContent = (phaseText[result.phase] || result.phase) + '\\n' + detail; setTimeout(pollStatus, 200); } catch (error) { finishTurnUi('状态读取失败：' + error); } }
async function cancelSession() { if (!activeTestId) return; document.getElementById('status').textContent = '正在请求关闭转向灯…'; try { await apiFetch('/api/cancel/' + activeTestId, {method:'POST'}); } catch (error) { finishTurnUi('取消请求失败：' + error); } }
function finishTurnUi(message) { document.getElementById('status').textContent = message; document.querySelectorAll('#left,#right').forEach(button => button.disabled = false); document.getElementById('cancel').style.display = 'none'; activeTestId = null; }
async function runSpeed(action) { const status = document.getElementById('status'); status.textContent = '正在发送速度按钮模板…'; try { const response = await apiFetch('/api/speed/' + action, {method:'POST'}); const result = await response.json(); if (!response.ok) throw new Error(result.message || '测试失败'); status.textContent = result.message; } catch (error) { status.textContent = '速度按钮测试失败：' + error; } }
</script></main></body></html>""".replace("__DRIVING_TAB_STATE__", driving_tab_state).encode()


class DeviceConsoleHandler(BaseHTTPRequestHandler):
  server_version = "DeviceConsole/1.0"

  def _send(self, status: HTTPStatus, content_type: str, body: bytes) -> None:
    self.send_response(status)
    self.send_header("Content-Type", content_type)
    self.send_header("Content-Length", str(len(body)))
    self.send_header("Cache-Control", "no-store")
    self.end_headers()
    self.wfile.write(body)

  def _authorize_api(self) -> bool:
    try:
      if not client_is_local(self.client_address[0]):
        raise PermissionError("仅允许本机、热点或局域网客户端访问")
      return True
    except PermissionError as error:
      self._json(HTTPStatus.FORBIDDEN, {"ok": False, "message": str(error)})
      return False

  @staticmethod
  def _range_from_query(query: dict[str, list[str]]) -> tuple[int, int]:
    try:
      start_ms = int(query.get("start_ms", [""])[0])
      end_ms = int(query.get("end_ms", [""])[0])
    except (TypeError, ValueError):
      raise ValueError("必须提供有效的开始和结束时间") from None
    return start_ms, end_ms

  def _stream_log_download(self, selection: LogSelection) -> None:
    if not _LOG_DOWNLOAD_LOCK.acquire(blocking=False):
      self._json(HTTPStatus.CONFLICT, {"ok": False, "message": "已有日志下载正在进行"})
      return
    try:
      self.send_response(HTTPStatus.OK)
      self.send_header("Content-Type", "application/zip")
      self.send_header("Content-Disposition", f'attachment; filename="{download_filename(selection)}"')
      self.send_header("Cache-Control", "no-store")
      self.send_header("X-Content-Type-Options", "nosniff")
      self.send_header("Connection", "close")
      self.end_headers()
      self.close_connection = True
      stream_log_zip(selection, self.wfile)
      self.wfile.flush()
    except (BrokenPipeError, ConnectionResetError):
      pass
    finally:
      _LOG_DOWNLOAD_LOCK.release()

  def do_GET(self) -> None:
    request = urlparse(self.path)
    path = request.path
    query = parse_qs(request.query, keep_blank_values=True)
    if not client_is_local(self.client_address[0]):
      self._send(HTTPStatus.FORBIDDEN, "text/plain; charset=utf-8", "仅允许本地网络访问".encode())
      return
    if path.startswith("/api/") and not self._authorize_api():
      return
    if path == "/api/hotspot":
      self._json(HTTPStatus.OK, hotspot_status())
      return
    if path == "/api/logs/status":
      self._json(HTTPStatus.OK, {**available_log_range(), **console_status(), "structured_logs_only": True})
      return
    if path == "/api/logs/preview":
      try:
        selection = select_log_range(*self._range_from_query(query))
        self._json(HTTPStatus.OK, {"ok": True, **selection.summary()})
      except ValueError as error:
        self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "message": str(error)})
      return
    if path == "/api/logs/download":
      try:
        require_offroad()
        selection = select_log_range(*self._range_from_query(query))
        if not selection.files:
          raise ValueError("所选时间段没有 rlog/qlog 日志")
        self._stream_log_download(selection)
      except PermissionError as error:
        self._json(HTTPStatus.FORBIDDEN, {"ok": False, "message": str(error)})
      except ValueError as error:
        self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "message": str(error)})
      return
    if path == "/api/driving-status":
      try:
        self._json(HTTPStatus.OK, driving_status_snapshot())
      except PermissionError as error:
        self._json(HTTPStatus.FORBIDDEN, {"ok": False, "message": str(error)})
      except Exception as error:
        self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"ok": False, "message": f"行驶数据暂不可用：{error}"})
      return
    if path == "/api/terminal/status":
      self._json(HTTPStatus.OK, terminal_status())
      return
    if path == "/api/settings":
      self._json(HTTPStatus.OK, settings_snapshot())
      return
    if path.startswith("/api/status/"):
      test_id = path.removeprefix("/api/status/")
      with _SESSION_LOCK:
        session_expired = (_ACTIVE_WEB_TEST_ID == test_id and
                           time.monotonic() - _ACTIVE_WEB_SESSION_STARTED >= WEB_SESSION_TIMEOUT_S)
      if session_expired:
        cancel_validation_session(test_id)
        _clear_active_session(test_id)
        self._json(HTTPStatus.OK, {"test_id": test_id, "done": True, "result": "WEB_SESSION_TIMEOUT"})
        return
      status = get_validation_status(test_id)
      if status.get("done"):
        _clear_active_session(test_id)
      self._json(HTTPStatus.OK, status)
      return
    if path not in ("/", "/index.html"):
      self._send(HTTPStatus.NOT_FOUND, "text/plain; charset=utf-8", b"Not found")
      return
    self._send(HTTPStatus.OK, "text/html; charset=utf-8", render_page())

  def do_POST(self) -> None:
    if not self._authorize_api():
      return
    if self.path == "/api/hotspot":
      try:
        require_offroad()
        content_length = int(self.headers.get("Content-Length", "0"))
        if content_length <= 0 or content_length > 4096:
          raise ValueError("请求内容无效")
        payload = json.loads(self.rfile.read(content_length))
        if not isinstance(payload, dict) or not isinstance(payload.get("enabled"), bool):
          raise ValueError("请求必须包含 enabled 开关")
        self._json(HTTPStatus.OK, set_hotspot_enabled(payload["enabled"]))
      except PermissionError as error:
        self._json(HTTPStatus.FORBIDDEN, {"ok": False, "message": str(error)})
      except RuntimeError as error:
        self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"ok": False, "message": str(error)})
      except (TypeError, ValueError, json.JSONDecodeError) as error:
        self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "message": str(error)})
      return
    if self.path == "/api/terminal/exec":
      try:
        content_length = int(self.headers.get("Content-Length", "0"))
        if content_length <= 0 or content_length > 8192:
          raise ValueError("请求内容无效")
        payload = json.loads(self.rfile.read(content_length))
        if not isinstance(payload, dict) or "command" not in payload:
          raise ValueError("请求必须包含 command")
        self._json(HTTPStatus.OK, run_command(payload["command"], self.headers.get("X-Terminal-Password")))
      except PermissionError as error:
        self._json(HTTPStatus.FORBIDDEN, {"ok": False, "message": str(error)})
      except (TypeError, ValueError, json.JSONDecodeError) as error:
        self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "message": str(error)})
      return
    if self.path == "/api/terminal/password":
      try:
        content_length = int(self.headers.get("Content-Length", "0"))
        if content_length <= 0 or content_length > 8192:
          raise ValueError("请求内容无效")
        payload = json.loads(self.rfile.read(content_length))
        if not isinstance(payload, dict):
          raise ValueError("请求必须为 JSON 对象")
        change_password(payload.get("current_password"), payload.get("new_password"))
        self._json(HTTPStatus.OK, {"ok": True})
      except PermissionError as error:
        self._json(HTTPStatus.FORBIDDEN, {"ok": False, "message": str(error)})
      except (TypeError, ValueError, json.JSONDecodeError) as error:
        self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "message": str(error)})
      return
    if self.path.startswith("/api/settings/"):
      key = self.path.removeprefix("/api/settings/")
      if not key or "/" in key:
        self._json(HTTPStatus.NOT_FOUND, {"ok": False, "message": "未知设置"})
        return
      try:
        content_length = int(self.headers.get("Content-Length", "0"))
        if content_length <= 0 or content_length > 4096:
          raise ValueError("请求内容无效")
        payload = json.loads(self.rfile.read(content_length))
        if not isinstance(payload, dict) or "value" not in payload:
          raise ValueError("请求必须包含 value")
        self._json(HTTPStatus.OK, {"ok": True, **validate_and_write(key, payload["value"])})
      except KeyError:
        self._json(HTTPStatus.NOT_FOUND, {"ok": False, "message": "未知或不允许修改的设置"})
      except PermissionError as error:
        self._json(HTTPStatus.FORBIDDEN, {"ok": False, "message": str(error)})
      except (TypeError, ValueError, json.JSONDecodeError) as error:
        self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "message": str(error)})
      return
    if self.path.startswith("/api/cancel/"):
      test_id = self.path.removeprefix("/api/cancel/")
      cancel_validation_session(test_id)
      self._json(HTTPStatus.ACCEPTED, {"ok": True, "test_id": test_id})
      return
    if self.path.startswith("/api/speed/"):
      action_value = self.path.removeprefix("/api/speed/")
      if action_value not in (SpeedButtonAction.increase.value, SpeedButtonAction.decrease.value):
        self._json(HTTPStatus.NOT_FOUND, {"ok": False, "message": "未知速度按钮测试"})
        return
      try:
        result = run_validation(SpeedButtonAction(action_value))
      except RuntimeError as error:
        self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"ok": False, "message": f"测试被阻止：{error}"})
        return
      messages = {
        0: "验证通过：设定速度已按预期改变",
        1: "验证被阻止：检查车辆状态、参数和新鲜原车模板",
        2: "验证失败：Panda 拒绝或未观察到发送回显",
        3: "模板已发送；请观察车辆设定速度显示",
      }
      self._json(HTTPStatus.OK if result in (0, 3) else HTTPStatus.CONFLICT,
                 {"ok": result in (0, 3), "result": result, "message": messages[result]})
      return
    direction = self.path.removeprefix("/api/turn/")
    if direction in ("left", "right") and self.path == f"/api/turn/{direction}":
      try:
        global _ACTIVE_WEB_SESSION_STARTED, _ACTIVE_WEB_TEST_ID
        with _SESSION_LOCK:
          if _ACTIVE_WEB_TEST_ID is not None:
            existing = get_validation_status(_ACTIVE_WEB_TEST_ID)
            session_expired = time.monotonic() - _ACTIVE_WEB_SESSION_STARTED >= WEB_SESSION_TIMEOUT_S
            if not existing.get("done") and not session_expired:
              self._json(HTTPStatus.CONFLICT, {"ok": False, "message": "已有变道请求正在运行"})
              return
            if not existing.get("done"):
              cancel_validation_session(_ACTIVE_WEB_TEST_ID)
          test_id = start_validation_session(direction)
          _ACTIVE_WEB_TEST_ID = test_id
          _ACTIVE_WEB_SESSION_STARTED = time.monotonic()
        self._json(HTTPStatus.ACCEPTED, {"ok": True, "test_id": test_id, "log": VALIDATION_LOG_PATH})
      except RuntimeError as error:
        self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"ok": False, "message": f"测试被阻止：{error}"})
      return
    self._json(HTTPStatus.NOT_FOUND, {"ok": False, "message": "未知请求"})

  def _json(self, status: HTTPStatus, payload: dict) -> None:
    self._send(status, "application/json; charset=utf-8", json.dumps(payload, ensure_ascii=False).encode())

  def log_message(self, message_format: str, *args) -> None:
    pass


def main() -> None:
  server = ThreadingHTTPServer((HOST, PORT), DeviceConsoleHandler)
  server.serve_forever()


if __name__ == "__main__":
  main()
