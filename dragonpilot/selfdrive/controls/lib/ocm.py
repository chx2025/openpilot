"""
Copyright (c) 2025, Rick Lan
Modified OCM (Overdrive Coasting Mode) integrated with DTSC strict abort logic.
"""

import time
import numpy as np
from openpilot.common.swaglog import cloudlog

# =========================================================
# 滑行速度與安全參數設定區 (沿用 OCM 邏輯)
# =========================================================
OVERTAKE_THRESHOLD = 2.0 / 3.6   # 2 km/h - 只要比定速快 2 公里，就允許進入滑行
HYSTERESIS_OFFSET = 1.0 / 3.6    # 1 km/h - 保持滑行直到接近定速時才解除
TTC_THRESHOLD = 2.0              # 秒 - 前方 2.0 秒內有車即停用

# 車速開關門檻 (50 km/h 打開，40 km/h 關閉)
MIN_SPEED_ENABLE = 50.0 / 3.6    
MIN_SPEED_DISABLE = 40.0 / 3.6   

# 下坡/超速防護上限 (加入遲滯區間防震盪)
OVERSPEED_ACTIVE_MARGIN = 25.0 / 3.6   # 正在滑行時最高容忍超速 25 km/h
OVERSPEED_INACTIVE_MARGIN = 20.0 / 3.6 # 關閉後需降至超速 20 km/h 以下才重啟

# 緊急安全防線
EMERGENCY_TTC = 2.0
EMERGENCY_RELATIVE_SPEED = 10.0
EMERGENCY_DECEL_THRESHOLD = -1.5 # 煞車力道超過此值視為緊急狀況，立刻交還控制權

# 動態安全距離參數
SPEED_BP = [0., 10., 20., 30.]
MIN_DIST_V = [5., 10., 15., 20.]

# 坡度參數
PITCH_SMOOTH_ALPHA_UP = 0.30           
PITCH_SMOOTH_ALPHA_DOWN = 0.15
PITCH_DOWNHILL_THRESHOLD = -0.030      # 判定為下坡的閾值 (-3% 坡度)


class OCM:
  def __init__(self):
    self.enabled = False
    
    self.active = False
    self.just_disabled = False
    self.speed_allowed = False  
    
    self._is_speed_over_cruise = False
    self._has_lead = False
    self._active_prev = False
    self._last_lead_time = 0.0
    
    # 坡度記憶
    self.current_pitch = 0.0              
    self._is_first_pitch = True           

  def _update_pitch(self, orientation_ned):
    """更新並平滑化道路坡度資訊"""
    if len(orientation_ned) == 3:
      new_pitch = orientation_ned[1]
      if self._is_first_pitch:
          self.current_pitch = new_pitch
          self._is_first_pitch = False
      else:
          alpha = PITCH_SMOOTH_ALPHA_UP if new_pitch > self.current_pitch else PITCH_SMOOTH_ALPHA_DOWN
          self.current_pitch = alpha * new_pitch + (1.0 - alpha) * self.current_pitch

  def _check_emergency_conditions(self, lead, v_ego, current_time):
    """緊急狀況判斷 (結合動態安全距離)"""
    if not lead or not lead.status:
      return False
      
    closing_speed = max(v_ego - lead.vLead, 0.1)
    self.lead_ttc = lead.dRel / closing_speed
    relative_speed = v_ego - lead.vLead
    
    min_dist_for_speed = np.interp(v_ego, SPEED_BP, MIN_DIST_V)

    if (self.lead_ttc < EMERGENCY_TTC) or \
       (relative_speed > EMERGENCY_RELATIVE_SPEED) or \
       (lead.dRel < min_dist_for_speed and relative_speed > 0):
      self._last_lead_time = current_time
      if self.active:
        cloudlog.warning(f"OCM emergency disable: dRel={lead.dRel:.1f}m, TTC={self.lead_ttc:.1f}s")
      return True
      
    return False

  def _update_lead_status(self, lead, v_ego, current_time):
    if lead and lead.status:
      closing_speed = max(v_ego - lead.vLead, 0.1)
      self.lead_ttc = lead.dRel / closing_speed
      if self.lead_ttc < TTC_THRESHOLD:
        self._has_lead = True
        self._last_lead_time = current_time
      else:
        self._has_lead = False
    else:
      self._has_lead = False

  def _should_activate(self, user_ctrl_lon, v_ego, v_cruise, in_cooldown):
    # 坡度安全防護：如果是明顯下坡 (-3%以上)，不允許純滑行
    if self.current_pitch < PITCH_DOWNHILL_THRESHOLD:
        return False

    # ==========================================
    # 修改區塊：超速上限防護 (加入遲滯區間防震盪)
    # ==========================================
    overspeed_margin = OVERSPEED_ACTIVE_MARGIN if self.active else OVERSPEED_INACTIVE_MARGIN
    if v_ego > (v_cruise + overspeed_margin):
        return False
    # ==========================================

    if self.active:
      self._is_speed_over_cruise = v_ego > (v_cruise + HYSTERESIS_OFFSET)
    else:
      self._is_speed_over_cruise = v_ego >= (v_cruise + OVERTAKE_THRESHOLD)

    return (not user_ctrl_lon and not self._has_lead and 
            not in_cooldown and self._is_speed_over_cruise)

  def update_states(self, cc, rs, user_ctrl_lon, v_ego, v_cruise, dtsc_active=False):
    # 車速總開關狀態更新 (40關閉 50打開)
    if v_ego >= MIN_SPEED_ENABLE:
      self.speed_allowed = True
    elif v_ego <= MIN_SPEED_DISABLE:
      self.speed_allowed = False

    # 絕對硬防線：當前車速低於巡航車速時，強制關閉
    if v_ego < v_cruise:
      self.active = False
      self.just_disabled = self._active_prev and not self.active
      self._active_prev = self.active
      return

    # 🚨 DTSC 攔截防護 (修正狀態機同步)
    if dtsc_active:
      self.active = False
      self.just_disabled = self._active_prev and not self.active
      if self.just_disabled:
        cloudlog.info("OCM deactivated: DTSC is currently active in curve")
      self._active_prev = self.active
      return

    # 如果開關未啟用、車速未達標、或姿態資料不完整
    if not self.enabled or not self.speed_allowed or len(cc.orientationNED) != 3:
      self.active = False
      self.just_disabled = self._active_prev and not self.active
      self._active_prev = self.active
      return

    self._update_pitch(cc.orientationNED)
      
    current_time = time.monotonic()
    lead = rs.leadOne

    if self._check_emergency_conditions(lead, v_ego, current_time):
      self.active = False
      self.just_disabled = self._active_prev and not self.active
      self._active_prev = self.active
      return

    self._update_lead_status(lead, v_ego, current_time)
    in_cooldown = (current_time - self._last_lead_time) < 0.5
    
    self.active = self._should_activate(user_ctrl_lon, v_ego, v_cruise, in_cooldown)
    self.just_disabled = self._active_prev and not self.active
    
    # 寫入系統日誌
    if self.active and not self._active_prev:
      cloudlog.info(f"OCM activated: v_ego={v_ego*3.6:.1f} km/h, v_cruise={v_cruise*3.6:.1f} km/h")
    elif self.just_disabled:
      cloudlog.info("OCM deactivated (Normal condition met)")
      
    self._active_prev = self.active

  def update_a_desired_trajectory(self, a_desired_trajectory):
    if not self.active:
      return a_desired_trajectory

    # 安全檢查：若 MPC 模型判定需要急煞，不予攔截
    min_accel = np.min(a_desired_trajectory)
    if min_accel < EMERGENCY_DECEL_THRESHOLD:
      cloudlog.warning(f"OCM aborting: MPC requested {min_accel:.2f} m/s² braking")
      self.active = False
      return a_desired_trajectory

    # 取消滑行下限，實現平順滑行 (保留 -1.0 的過彎穩定底線)
    modified = np.copy(a_desired_trajectory)
    for i in range(len(modified)):
      if -1.0 < modified[i] < 0:
        modified[i] = 0.0
        
    return modified
