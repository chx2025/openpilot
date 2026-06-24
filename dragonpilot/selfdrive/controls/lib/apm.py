"""
Copyright (c) 2026, Rick Lan

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

from cereal import log
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL

# 速度門檻常數 (km/h 轉換為 m/s)
APM_DEPARTURE_SPEED = 5 * 1000 / 3600   # 5 km/h：起步激烈模式上限

# [暫時不用] 以下常數保留不影響執行
V_LEAD_RELAX_ENTER = 10 * 1000 / 3600
A_LEAD_RELAX_ENTER = -0.1
V_REL_RELAX_ENTER = 20 * 1000 / 3600
V_REL_RELAX_EXIT = 5 * 1000 / 3600
V_EGO_LOCK_DECEL = 20 * 1000 / 3600

# 完全靜止門檻
V_EGO_STANDSTILL = 0.01                  # 低於 0.01 m/s 視為完全靜止，才準備起步


class APM:
  def __init__(self):
    self.params = Params()
    self.frame = 0
    
    # 用來記憶車輛狀態
    self.is_departing = False       
    self.is_relaxed_mode = False    
    self.is_scene2_standard = False 
    self.is_approaching = False     
    
    # 濾波平滑化狀態
    self.v_rel_smoothed = None      

    # 初始化開關狀態
    self._enabled = self.params.get_bool("dp_lon_apm")

  def update(self, sm=None):
    """依照系統物理週期更新 Params，參照 accel_controller"""
    self.frame += 1
    if self.frame % int(10.0 / DT_MDL) == 0:
      self._enabled = self.params.get_bool("dp_lon_apm")

  def is_enabled(self) -> bool:
    return self._enabled

  def get_personality(self, v_ego, has_lead, v_lead, a_lead, d_lead, personality, t_follow_relaxed=1.75):
    if not self._enabled:
      return personality
      
    # --- 1. 起步狀態更新 ---
    if v_ego < V_EGO_STANDSTILL:
      self.is_departing = True
    elif v_ego >= APM_DEPARTURE_SPEED:
      self.is_departing = False

    # --- 2. 判斷前車狀況 ---
    if has_lead:
      v_rel_raw = v_ego - v_lead

      # --- 低通濾波處理 ---
      if self.v_rel_smoothed is None:
        self.v_rel_smoothed = v_rel_raw
      else:
        self.v_rel_smoothed = self.v_rel_smoothed * 0.90 + v_rel_raw * 0.10
      
      # ==========================================
      # [測試修改] 強制將場景 2 與 場景 3 狀態歸零，停止動態切換
      self.is_relaxed_mode = False
      self.is_scene2_standard = False
      self.is_approaching = False
      
      """
      v_rel = self.v_rel_smoothed
      d_req = max(20.0, v_ego * t_follow_relaxed)

      if v_ego < V_EGO_LOCK_DECEL and a_lead <= 0.1:
        pass # 低速鎖定，保持狀態
      else:
        # 場景 2：前車絕對速度判斷 + 負加速判斷
        if v_lead < V_LEAD_RELAX_ENTER and a_lead < A_LEAD_RELAX_ENTER:
          if d_lead >= d_req:
            self.is_relaxed_mode = True
            self.is_scene2_standard = False
          elif d_lead < (d_req - 5.0):
            self.is_relaxed_mode = False
            self.is_scene2_standard = True
        elif v_rel <= V_REL_RELAX_EXIT:
          self.is_relaxed_mode = False
          self.is_scene2_standard = False
          
        # 場景 3：與前車相對速差判斷 + 車距大於動態門檻
        if v_rel >= V_REL_RELAX_ENTER and d_lead >= d_req:
          self.is_approaching = True
        elif v_rel <= V_REL_RELAX_EXIT:
          self.is_approaching = False
      """
      # ==========================================
        
    else:
      self.is_relaxed_mode = False
      self.is_scene2_standard = False
      self.is_approaching = False
      self.v_rel_smoothed = None  

    # --- 3. 決定最終輸出的模式 ---
    if self.is_departing and v_ego < APM_DEPARTURE_SPEED:
      return log.LongitudinalPersonality.aggressive

    # 因為上面已強制設為 False，以下三個條件在此版本將不會觸發
    if self.is_scene2_standard:
      return log.LongitudinalPersonality.standard

    if self.is_relaxed_mode:
      return log.LongitudinalPersonality.relaxed

    if self.is_approaching:
      return log.LongitudinalPersonality.relaxed

    return personality
