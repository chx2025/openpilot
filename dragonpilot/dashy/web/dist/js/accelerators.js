'use strict';
(() => {
  const $ = id => document.getElementById(id);
  let state = null, sending = false;
  const dirty = new Set();
  const stages = {idle:'待命',resolving:'解析模型',downloading:'下載中',compiling_warp:'編譯影像處理',compiling:'編譯模型',done:'完成',failed:'失敗',cancelled:'已取消',waiting_egpu:'等待接上 eGPU',connect:'連線中',fetch:'下載中',upload:'傳送模型',build:'建置引擎'};
  const busy = e => !!e.pending || ['resolving','downloading','compiling_warp','compiling'].includes(e.progress.state);
  function options(id, rows, first, selected) {
    const el = $(id), wanted = dirty.has(id) ? el.value : selected;
    const entries = [[ '', first ], ...rows.map(r => [r.ref, r.name + (r.compiled ? ' · 已編譯' : r.built ? ' · 已建置' : r.downloaded ? ' · 已下載' : '')])];
    if (wanted && !entries.some(([v]) => v === wanted)) entries.push([wanted, `目前選取：${wanted.slice(0,12)}`]);
    const signature = JSON.stringify(entries);
    if (el.dataset.options !== signature) {
      el.replaceChildren(...entries.map(([v,t]) => new Option(t,v)));
      el.dataset.options = signature;
    }
    el.value = wanted || '';
  }
  function progress(p) {
    if (!p || !Object.keys(p).length) return '';
    const frac = typeof p.progress === 'number' ? ` ${Math.round(p.progress)}%` : typeof p.frac === 'number' ? ` ${Math.round(Math.max(0,Math.min(1,p.frac))*100)}%` : '';
    return [stages[p.state || p.stage] || p.state || p.stage, frac, p.msg || p.message || p.error || p.detail || ''].filter(Boolean).join(' ');
  }
  function render() {
    if (!state) return;
    const e=state.egpu,j=state.jetlink,s=j.status;
    $('notice').textContent=state.offroad?'已在 offroad，可變更設定。':'行車模式或狀態尚未確認：僅能查看，設定已鎖定。';
    if (!dirty.has('egpu-enabled')) $('egpu-enabled').checked=e.enabled;
    if (!dirty.has('jetlink-mode')) $('jetlink-mode').value=String(j.mode);
    options('egpu-model',e.models,'不使用大模型',e.pending || e.active.ref);
    options('egpu-delete',e.models.filter(r=>r.compiled||r.downloaded),'請選擇要刪除的模型','');
    options('jetlink-model',j.models,'跟隨外部裝置',j.selected.ref);
    const es=!e.enabled?'已停用':e.running?'大模型運作中':e.loading?'載入中，完成前尚無模型輸出':e.failed?'本趟改用小模型，下次 offroad／onroad 再嘗試':e.present?'eGPU 已連接':'未偵測到 eGPU';
    $('egpu-status').textContent=[es,`目前模型：${e.active.display_name || '未選擇'}`,progress(e.progress)].filter(Boolean).join('\n');
    $('jetlink-status').textContent=[`外部裝置：${s.present?'已連接':'未連接'}`,`引擎：${s.ready?'已就緒':'尚未就緒'}`,`行車將使用：${s.model || '未知'}`,`外部裝置目前模型：${j.models.find(r=>r.loaded)?.name || '未回報或不在清單內'}`,s.transport,`USB 埠：${s.port || '未知'}`,`ADB：${j.adb?'開啟':'關閉'}`,j.reason||s.reason,progress(s.progress),j.reserved?'eGPU 保留連接埠，Jetlink 暫不可使用。':''].filter(Boolean).join('\n');
    document.querySelectorAll('button,select,input').forEach(el=>{el.disabled=sending||!state.offroad;});
    if (!sending && state.offroad) {
      document.querySelectorAll('[data-target="egpu"]').forEach(el=>{
        if(['select','delete','clear'].includes(el.dataset.action)) el.disabled=busy(e);
        if(el.dataset.action==='cancel') el.disabled=!e.pending;
        if(el.dataset.action==='delete') el.disabled=busy(e)||!$('egpu-delete').value;
      });
    }
  }
  async function refresh() {
    try {
      const r=await fetch('/api/accelerators',{cache:'no-store'});
      if(!r.ok) throw new Error(`無法讀取狀態（${r.status}）`);
      state=await r.json();render();
    } catch(err) {
      state=null;$('notice').textContent=`${err.message}；設定已鎖定。`;
      document.querySelectorAll('button,select,input').forEach(el=>{el.disabled=true;});
    }
  }
  async function send(target,action,value) {
    if(sending||!state?.offroad)return;
    sending=true;render();$('message').textContent='處理中…';$('message').className='';
    try {
      const r=await fetch('/api/accelerators',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({target,action,value})});
      if(!r.ok) throw new Error(await r.text());
      dirty.delete(`${target}-${action==='select'?'model':action}`);
      $('message').textContent=action==='refresh'?'已要求背景更新共用模型清單。':'設定已送出。';
    }catch(err){$('message').textContent=err.message;$('message').className='error';dirty.delete('egpu-enabled');}
    finally{sending=false;await refresh();}
  }
  document.querySelectorAll('select').forEach(el=>el.addEventListener('change',()=>{dirty.add(el.id);render();}));
  $('egpu-enabled').addEventListener('change',()=>{dirty.add('egpu-enabled');send('egpu','enabled',$('egpu-enabled').checked);});
  document.querySelectorAll('button[data-action]').forEach(el=>el.addEventListener('click',()=>{
    const {target,action}=el.dataset;
    if(['delete','clear'].includes(action)&&!confirm(action==='delete'?'刪除這個模型？若正在使用此模型，之後將改用小模型。':'清除快取？只保留目前使用的模型。'))return;
    const value=action==='mode'?Number($('jetlink-mode').value):action==='select'?$(target+'-model').value:action==='delete'?$('egpu-delete').value:null;
    send(target,action,value);
  }));
  async function poll(){await refresh();setTimeout(poll,2000);}
  poll();
})();
