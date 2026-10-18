"""Dashy accelerator settings; reuse the device panels' parameter protocol."""
import asyncio
import logging
from aiohttp import web
from dragonpilot import jetlink_adapter as jl
from dragonpilot.egpu import constants as c
from dragonpilot.egpu.bundles import cached_bundles, pointer_for, model_compiled, onnx_present


def offroad(p):
  return p.get_bool('IsOffroad') and not p.get_bool('IsOnroad')


def snapshot(p):
  rows = []
  for b in cached_bundles(p):
    pointer = pointer_for(p, b['ref'])
    rows.append({'ref': b['ref'], 'name': b['display_name'],
                 'compiled': bool(pointer and model_compiled(pointer['oid'])), 'downloaded': onnx_present(pointer)})
  status = jl.status()
  active = p.get(c.P_ACTIVE_BUNDLE)
  selected = p.get(jl.KEYS.big_model)
  return {'offroad': offroad(p), 'egpu': {
    'enabled': p.get_bool(c.P_ENABLED), 'present': p.get_bool(c.P_PRESENT),
    'loading': p.get_bool(c.P_LOADING), 'running': p.get_bool(c.P_RUNNING),
    'ready': p.get_bool(c.P_READY), 'failed': p.get_bool(c.P_FAILED),
    'active': active if isinstance(active, dict) else {}, 'pending': p.get(c.P_DOWNLOAD_REF),
    'progress': p.get(c.P_STATUS) or {}, 'models': rows,
  }, 'jetlink': {'mode': int(p.get(jl.KEYS.link) or 0),
    'selected': selected if isinstance(selected, dict) else {}, 'models': jl.model_menu(p),
    'adb': p.get_bool('AdbEnabled'),
    'status': status._asdict() if status else {}, 'reason': jl.reason(),
    'reserved': p.get_bool(c.P_PRESENT) or (p.get_bool(c.P_ENABLED) and p.get_bool(c.P_SEEN))}}


def mutate(p, data):
  # Missing/unknown driving state is not authorization to change settings.
  if not offroad(p):
    raise web.HTTPConflict(text='請在 offroad 時變更設定')
  if not isinstance(data, dict):
    raise web.HTTPBadRequest(text='無效的設定格式')
  target, action = data.get('target'), data.get('action')
  value = data.get('value')
  if target not in ('egpu', 'jetlink'):
    raise web.HTTPBadRequest(text='未知的裝置')
  progress = p.get(c.P_STATUS) or {}
  busy = bool(p.get(c.P_DOWNLOAD_REF)) or (isinstance(progress, dict) and progress.get('state') in c.BUSY_STATES)
  if action == 'refresh':
    p.put(c.P_MANIFEST_SYNC, 0)  # egpu_manager refreshes the shared catalog offroad
  elif target == 'egpu':
    if action == 'enabled':
      if type(value) is not bool:
        raise web.HTTPBadRequest(text='開關必須為布林值')
      if value and int(p.get(jl.KEYS.link) or 0) != 0:
        raise web.HTTPConflict(text='請先關閉 Jetlink，再啟用 eGPU')
      p.put_bool(c.P_ENABLED, value)
    elif action == 'cancel':
      p.put_bool(c.P_AUTO_DEFAULT, False)
      p.remove(c.P_DOWNLOAD_REF)
    elif action in ('select', 'delete', 'clear'):
      if busy or p.get(c.P_DELETE_REF) or p.get_bool(c.P_CLEAR_CACHE):
        raise web.HTTPConflict(text='模型處理中，請先取消或等待完成')
      refs = {b['ref'] for b in cached_bundles(p)}
      active = p.get(c.P_ACTIVE_BUNDLE)
      if isinstance(active, dict) and active.get('ref'):
        refs.add(active['ref'])
      if action != 'clear' and not (action == 'select' and value == '') and (not isinstance(value, str) or value not in refs):
        raise web.HTTPBadRequest(text='模型不在清單內，請更新清單')
      p.put_bool(c.P_AUTO_DEFAULT, False)
      if action == 'clear':
        p.put_bool(c.P_CLEAR_CACHE, True)
      elif action == 'delete':
        p.put(c.P_DELETE_REF, value)
      elif value == '':
        p.remove(c.P_DOWNLOAD_REF)
        p.remove(c.P_ACTIVE_BUNDLE)
      else:
        p.put(c.P_DOWNLOAD_REF, value)
    else:
      raise web.HTTPBadRequest(text='未知操作')
  elif action == 'mode':
    if type(value) is not int or value not in range(len(jl.MODES)):
      raise web.HTTPBadRequest(text='無效的連線方式')
    if value and (p.get_bool(c.P_PRESENT) or p.get_bool(c.P_ENABLED)):
      raise web.HTTPConflict(text='請先停用並拔除 eGPU，再開啟 Jetlink')
    if value:
      p.put_bool('AdbEnabled', False)
    p.put(jl.KEYS.link, value)
  elif action == 'select':
    if not isinstance(value, str) or (value and value not in {b['ref'] for b in jl.model_menu(p)}):
      raise web.HTTPBadRequest(text='模型不在清單內')
    jl.pick_model(value or None, p)
  else:
    raise web.HTTPBadRequest(text='未知操作')
  return {'ok': True}


async def get_status(request):
  try:
    return web.json_response(await asyncio.to_thread(snapshot, request.app['cache'].params))
  except Exception:
    logging.getLogger('dashy').exception('Could not read accelerator settings')
    return web.json_response({'error': '無法讀取裝置狀態，請查看 dashy 日誌'}, status=503)


async def post_action(request):
  # JSON-only and same-origin: do not accept form posts from unrelated websites.
  origin = request.headers.get('Origin')
  if origin and origin != f'{request.scheme}://{request.host}':
    raise web.HTTPForbidden(text='不允許跨網站操作')
  if request.content_type != 'application/json':
    raise web.HTTPUnsupportedMediaType()
  try:
    data = await request.json()
  except ValueError:
    raise web.HTTPBadRequest(text='無效 JSON')
  result = mutate(request.app['cache'].params, data)
  request.app['cache'].invalidate()
  return web.json_response(result)
