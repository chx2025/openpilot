import time
import numpy as np
from cereal import log
from openpilot.common.swaglog import cloudlog

# 匯入 Openpilot 原廠 MPC (模型預測控制) 相關的安全距離與參數計算公式
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  COMFORT_BRAKE, STOP_DISTANCE, get_safe_obstacle_distance,
  get_stopped_equivalence_factor, get_T_FOLLOW
)

# =========================================================
# 參數設定區 (物理限制與門檻)
# =========================================================

SPEED_OFFSET_MIN_KPH = 1.0             
SPEED_OFFSET_MAX_FLAT_KPH = 15.0       
SPEED_OFFSET_MAX_DOWNHILL_KPH = 5.0    

PITCH_SMOOTH_ALPHA_UP = 0.30           
PITCH_SMOOTH_ALPHA_DOWN = 0.05         
PITCH_UPHILL_THRESHOLD = 0.050         
PITCH_DOWNHILL_THRESHOLD = -0.030      

SOFT_HOLD_PITCH_START = 0.050          
SOFT_HOLD_PITCH_MAX = 0.080            

TTC_BP = [10., 30.]                    
TTC_V  = [3.0, 3.0]                    
EMERGENCY_TTC = 2.0                    
EMERGENCY_RELATIVE_SPEED = 10.0        
EMERGENCY_DECEL_THRESHOLD = -1.5       

LEAD_COOLDOWN_TIME = 0.5               
SPEED_BP = [0., 10., 20., 30.]         
MIN_DIST_V = [5., 10., 15., 20.]       

SOFT_HOLD_RANGE_MIN = 0.70             
SOFT_HOLD_RANGE_MAX = 0.99             
SOFT_HOLD_TTC_THRESHOLD = 2.5          
VREL_DEBOUNCE_TIME = 0.6               

SOFT_HOLD_SPEED_BP = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
SOFT_HOLD_ACCEL_V  = [1.0,  0.80,  0.70,  0.50,  0.40,  0.10,  0.0]


# =========================================================
# 邏輯模組 1：純滑行控制器
# =========================================================
class CoastingLogic:
  def __init__(self):
    self.active = False                
    self.current_max_offset = 0.0      
    self._has_lead = False             
    self._last_lead_time = 0.0         
    self._active_prev = False          

  def check_emergency(self, lead, v_ego, current_time):
    if not lead or not lead.status:
      return False
    closing_speed = max(v_ego - lead.vLead, 0.1)
    lead_ttc = lead.dRel / closing_speed 
    relative_speed = v_ego - lead.vLead         
    min_dist_for_speed = np.interp(v_ego, SPEED_BP, MIN_DIST_V)

    if (lead_ttc < EMERGENCY_TTC) or \
       (relative_speed > EMERGENCY_RELATIVE_SPEED) or \
       (lead.dRel < min_dist_for_speed and relative_speed > 0):
      self._last_lead_time = current_time
      return True
    return False

  def update_lead_status(self, lead, v_ego, current_time):
    if lead and lead.status:
      closing_speed = max(v_ego - lead.vLead, 0.1)
      lead_ttc = lead.dRel / closing_speed
      current_ttc_threshold = np.interp(v_ego, TTC_BP, TTC_V) 
      if lead_ttc < current_ttc_threshold:
        self._has_lead = True
        self._last_lead_time = current_time
      else:
        self._has_lead = False 
    else:
      self._has_lead = False

  def update_states(self, enabled, user_ctrl_lon, v_ego, v_cruise, current_pitch, dtsc_is_active, current_time):
    if not enabled:
      self.active = False
      return

    if current_pitch < PITCH_DOWNHILL_THRESHOLD:
        self.current_max_offset = SPEED_OFFSET_MAX_DOWNHILL_KPH
    else:
        self.current_max_offset = SPEED_OFFSET_MAX_FLAT_KPH

    upper_bound = v_cruise + (self.current_max_offset / 3.6) 
    is_in_coast_window = (v_ego >= v_cruise and v_ego < upper_bound)
    in_cooldown = (current_time - self._last_lead_time) < LEAD_COOLDOWN_TIME

    should_activate = (not dtsc_is_active and
                       current_pitch <= PITCH_UPHILL_THRESHOLD and
                       not user_ctrl_lon and     
                       not self._has_lead and    
                       not in_cooldown and       
                       is_in_coast_window)
    
    self.active = should_activate
    self._active_prev = self.active

  def process_trajectory(self, a_desired_trajectory, lead):
    traj = np.copy(a_desired_trajectory)
    if self.active:
      min_accel = np.min(traj)
      if min_accel < EMERGENCY_DECEL_THRESHOLD:
        self.active = False
      else:
        if not (lead is not None and lead.status):
          for i in range(len(traj)):
            if -0.3 < traj[i] < 0:
              traj[i] = 0.0
    return traj


# =========================================================
# 邏輯模組 2：柔和跟車控制器
# =========================================================
class SoftHoldLogic:
  def __init__(self):
    self._soft_hold_factor = 1.0          
    self._vrel_high_start_time = 0.0      
    self._vrel_high_active = False        
    
    self._last_lead_time = 0.0            
    self._last_target_factor = 1.0        
    self._last_soft_hold_accel = 0.0      

  def process_trajectory(self, a_desired_trajectory, v_ego, lead, current_pitch, t_follow):
    should_cancel_soft_hold = False
    current_time = time.monotonic()
    mpc_max_accel_intent = np.max(a_desired_trajectory)
    has_valid_lead = lead is not None and lead.status

    target_factor = 1.0   
    v_ego_kph = v_ego * 3.6
    current_soft_hold_accel = np.interp(v_ego_kph, SOFT_HOLD_SPEED_BP, SOFT_HOLD_ACCEL_V)
    is_lead_braking_strict = False
    skip_state_2 = False

    # 狀態機 1：雷達防閃爍邏輯
    if not has_valid_lead:
        self._vrel_high_active = False
        if (current_time - self._last_lead_time) < 0.5:
            if self._last_soft_hold_accel >= 0.0:
                target_factor = self._last_target_factor
                current_soft_hold_accel = self._last_soft_hold_accel
            else:
                target_factor = 0.0
                current_soft_hold_accel = 0.0
            should_cancel_soft_hold = False
            skip_state_2 = True 
        else:
            should_cancel_soft_hold = True
            skip_state_2 = True
    else:
        self._last_lead_time = current_time 
        if lead.vRel > 1.0:
            if not self._vrel_high_active:
                self._vrel_high_active = True
                self._vrel_high_start_time = current_time
            elif (current_time - self._vrel_high_start_time) > VREL_DEBOUNCE_TIME:
                should_cancel_soft_hold = True
        else:
            self._vrel_high_active = False
            
        if current_pitch > SOFT_HOLD_PITCH_MAX or mpc_max_accel_intent > 0.4:
            should_cancel_soft_hold = True

    # 狀態機 2：前車動態精細判斷
    if not should_cancel_soft_hold and not skip_state_2:
        # 判定靜止車或極低速車 (< 1.0 m/s 即 3.6 km/h)
        is_lead_stopped = (lead.vLead < 1.0) and (lead.vRel <= 0.3)  
        
        if v_ego_kph <= 10.0:
            is_lead_braking_strict = (lead.aLeadK < -0.1 or is_lead_stopped) and (lead.vRel < 0.5)
        elif v_ego_kph <= 30.0:
            is_lead_braking_strict = (lead.aLeadK < -0.5 or is_lead_stopped) and (lead.vRel < 0.5)
        elif v_ego_kph <= 40.0:
            is_lead_braking_strict = lead.aLeadK < -1.0 or is_lead_stopped
        else: 
            is_lead_braking_strict = lead.aLeadK < -1.25 or is_lead_stopped

        closing_speed = max(v_ego - lead.vLead, 0.1)
        current_ttc = lead.dRel / closing_speed
        desired_dist = get_safe_obstacle_distance(v_ego, t_follow)
        lead_obstacle_dist = lead.dRel + get_stopped_equivalence_factor(lead.vLead)

        ratio = 10.0 if desired_dist < 0.1 else (lead_obstacle_dist / desired_dist)
        if ratio > 1.2:
            should_cancel_soft_hold = True

    # === 最終結算目標動力與微煞車力道 ===
    if should_cancel_soft_hold:
        target_factor = 1.0 
        alpha = 0.40        
    elif not skip_state_2: 
        distance_factor = 1.0 
        if current_pitch <= SOFT_HOLD_PITCH_MAX:
            if SOFT_HOLD_RANGE_MIN < ratio < SOFT_HOLD_RANGE_MAX and current_ttc <= SOFT_HOLD_TTC_THRESHOLD:
                distance_factor = 0.0

        v_rel_factor = np.interp(lead.vRel, [-2.0, 0.5], [0.0, 1.0])
        target_factor = max(distance_factor, v_rel_factor)

        # 在 99%~70% 跟車範圍內，且前車符合嚴格煞車/停止條件
        if SOFT_HOLD_RANGE_MIN < ratio < SOFT_HOLD_RANGE_MAX and is_lead_braking_strict:
            if current_pitch > SOFT_HOLD_PITCH_START:
                smooth_factor = float(np.interp(current_pitch, [SOFT_HOLD_PITCH_START, SOFT_HOLD_PITCH_MAX], [0.0, 1.0]))
                target_factor = smooth_factor  
                current_soft_hold_accel = current_soft_hold_accel * smooth_factor 
            else:
                # 【新增：靜止車查表法】取代原本單一的 0.0
                if is_lead_stopped:
                    # 使用 np.interp：當車速 150 時輸出 -0.15，車速 0 時輸出 0.0
                    # 車速例如 100km/h 時，將得到 -0.10 的微煞車
                    current_soft_hold_accel = float(np.interp(v_ego_kph, [0.0, 150.0], [0.0, -0.15]))
                
                # 若非靜止車，執行一般的高速動態微煞車邏輯
                elif v_ego_kph >= 50.0:
                    if lead.vRel < -0.1 and lead.aLeadK <= -1.5:
                        dynamic_brake = lead.aLeadK * 0.30
                        current_soft_hold_accel = np.clip(dynamic_brake, -1.0, 0.0)
                    else:
                        current_soft_hold_accel = 0.0 
                else:
                    # 一般慢速移動車輛 (非停止)，給予純滑行
                    current_soft_hold_accel = 0.0
                
                target_factor = 0.0 

        alpha = 0.10 if target_factor > self._soft_hold_factor else 0.20 
    else:
        alpha = 0.10 if target_factor > self._soft_hold_factor else 0.20 

    self._last_target_factor = target_factor
    self._last_soft_hold_accel = current_soft_hold_accel

    self._soft_hold_factor = (1.0 - alpha) * self._soft_hold_factor + alpha * target_factor

    traj = np.copy(a_desired_trajectory)
    if self._soft_hold_factor < 0.99:
        dynamic_limit = np.maximum(traj, 0.0) * self._soft_hold_factor + current_soft_hold_accel * (1.0 - self._soft_hold_factor)
        traj = np.minimum(traj, dynamic_limit)

    return traj


# =========================================================
# 統一對外接口
# =========================================================
class ACM:
  def __init__(self):
    self.enabled = False                  
    self.current_pitch = 0.0              
    self._is_first_pitch = True           
    self.personality = log.LongitudinalPersonality.standard 
    self._dtsc_is_active = False          
    self._is_normal_mode = True           

    self.coasting = CoastingLogic()       
    self.soft_hold = SoftHoldLogic()      

  @property
  def active(self):
    return self.coasting.active

  def update_states(self, cc, rs, user_ctrl_lon, v_ego, v_cruise, mode='acc', personality=log.LongitudinalPersonality.standard, dtsc_is_active=False):
    self.personality = personality
    self._dtsc_is_active = dtsc_is_active 
    self._is_normal_mode = (mode == 'acc')

    if not self.enabled or len(cc.orientationNED) != 3:
      self.coasting.active = False
      return

    new_pitch = cc.orientationNED[1]
    if self._is_first_pitch:
        self.current_pitch = new_pitch
        self._is_first_pitch = False
    else:
        alpha = PITCH_SMOOTH_ALPHA_UP if new_pitch > self.current_pitch else PITCH_SMOOTH_ALPHA_DOWN
        self.current_pitch = alpha * new_pitch + (1.0 - alpha) * self.current_pitch

    current_time = time.monotonic()
    lead = rs.leadOne 

    if self.coasting.check_emergency(lead, v_ego, current_time):
      self.coasting.active = False
      return

    self.coasting.update_lead_status(lead, v_ego, current_time)
    self.coasting.update_states(self.enabled, user_ctrl_lon, v_ego, v_cruise, self.current_pitch, dtsc_is_active, current_time)

  def update_a_desired_trajectory(self, a_desired_trajectory, v_ego=0.0, lead=None, t_follow=None):
    if self._dtsc_is_active or not self._is_normal_mode:
        return a_desired_trajectory

    if t_follow is None:
        t_follow = get_T_FOLLOW(self.personality)

    traj = self.coasting.process_trajectory(a_desired_trajectory, lead)
    traj = self.soft_hold.process_trajectory(traj, v_ego, lead, self.current_pitch, t_follow)
    
    return traj
