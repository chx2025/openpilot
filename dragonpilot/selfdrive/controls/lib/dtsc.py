"""
Dynamic Turn Speed Controller (DTSC) - v27 物理重生版 (Golden Right Foot Hybrid)
特色：
1. 融合 v27 MPC 陣列規劃與 10 秒低頻 UI 開關檢查，架構最現代化。
2. 融合 Candy 版「老司機黃金右腳」：引進加速度低通濾波 (LPF) 與速度階梯爬升。
3. 採用「出彎實體壓制」+「狀態死咬 (Hysteresis Recovery)」雙重出彎防護。
4. 保留 v27 寬容度較高的備援參數，完美免疫高速公路變換車道的幽靈急煞。
"""

import time
import numpy as np
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params

# =============================
# [一、核心參數設定]
# =============================
MODEL_T_IDXS = ModelConstants.T_IDXS
DT_MPC = 0.05  

FILE_LOG_ENABLED = False

LAT_LIMIT_BP = [5.0, 7.5, 10.0, 12.5, 15.0, 17.5, 20.0, 25.0, 30.0]
LAT_LIMIT_V  = [1.9, 1.9, 2.0,  2.0,  2.2,  2.4,  2.5,  2.7,  2.8]

DECEL_BP = np.array([1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.3, 2.6, 3.0])
DECEL_V  = np.array([0.0, -0.1, -0.3, -0.5, -0.8, -1.1, -1.4, -1.8, -2.3])

MAX_COMFORT_DECEL = -2.0       
EMERGENCY_DECEL   = -3.0       
MIN_CURVE_DISTANCE = 10.0      
MAX_EXIT_ACCEL = 0.4           

# ==========================================================
# [二、智慧備援機制參數 (採用 v27 安全設定，防變道急煞)]
# ==========================================================
BACKUP_LAT_G_TH = 1.2          
BACKUP_BASE_DECEL = -0.5       

INNER_DEV_TH = 2.0             
INNER_GAIN_PER_10CM = 0.04     
INNER_MAX_DECEL = -0.5         

OUTER_DEV_TH = 0.3             
OUTER_GAIN_PER_10CM = 0.3      
OUTER_MAX_DECEL = -2.0         

LPF_ALPHA = 0.15                
LPF_RESET_TIME = 2.0           
LPF_RESET_LAT_ACC_THRESHOLD = 0.3 
PERSISTENCE_MIN_FRAC = 0.6     
CURVATURE_MIN_FOR_PERSIST = 0.01 
SHORT_DIST_IGNORE = 3.5        
SCCV_ABORT_PRED_LAT_ACC_TH = 0.7 
FUTURE_CURVE_THRESHOLD = 0.015 
HYSTERESIS_TIME = 1.5          

# =============================
# 工具函式
# =============================
def clamp(x, low, high):
    return max(low, min(high, x))

def interp_clamped(x, bp, fp):
    if x <= bp[0]: return fp[0]
    if x >= bp[-1]: return fp[-1]
    return float(np.interp(x, bp, fp))

def write_file_log(msg):
    if not FILE_LOG_ENABLED: return
    try:
        with open("/data/media/0/dtsc_log.txt", "a") as f:
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            f.write(f"[{timestamp}] {msg}\n")
    except Exception:
        pass

# =============================
# DTSC 主類別
# =============================
class DTSC:
    def __init__(self, aggressiveness=1.0, **kwargs):
        self.aggressiveness = clamp(aggressiveness, 0.5, 1.8)
        self.active = False
        self.hysteresis_timer = 0.0
        self.filtered_lat_limits = None
        self.suggested_speed = V_CRUISE_MAX
        self.lpf_reset_timer = 0.0
        self.last_log_time = 0.0
        
        # --- 加入 HTD 安全係數動態過渡變數 ---
        self.safety_speed_factor = 0.95         # 當前實際使用的係數
        self.target_safety_speed_factor = 0.85  # 目標係數 (0.85 或 0.95)
        self.ramp_up_rate = 0.10                # 0.85 -> 0.95 需 1.0 秒: 每秒上升 0.10
        self.ramp_down_rate = 0.20              # 0.95 -> 0.85 需 0.5 秒: 每秒下降 0.20
        # -----------------------------------
        
        # [Candy 融合] 黃金右腳狀態變數
        self.smoothed_a_target = 0.0  
        self.output_v_target = V_CRUISE_MAX
        self.output_a_target = 0.0 

        self.params = Params()
        self.is_enabled = self.params.get_bool("dp_lon_dtsc")
        self.toggle_check_timer = 0.0
        
        cloudlog.info(f"DTSC (v27 Golden Right Foot Hybrid): 初始化完成. Aggressiveness={self.aggressiveness:.2f}")

    def set_aggressiveness(self, value):
        self.aggressiveness = clamp(value, 0.5, 1.8)

    def _is_model_valid(self, model_msg):
        try:
            return (len(model_msg.position.x) == ModelConstants.IDX_N and
                    len(model_msg.velocity.x) == ModelConstants.IDX_N and
                    len(model_msg.orientationRate.z) == ModelConstants.IDX_N)
        except Exception:
            return False

    def _compute_model_arrays(self, model_msg):
        v_arr = np.array(model_msg.velocity.x)
        pos_x_arr = np.array(model_msg.position.x)
        pos_y_arr = np.array(model_msg.position.y) 
        yaw_arr = np.array(model_msg.orientationRate.z)

        v_pred = np.interp(T_IDXS_MPC, MODEL_T_IDXS, v_arr)
        pos_x = np.interp(T_IDXS_MPC, MODEL_T_IDXS, pos_x_arr)
        pos_y = np.interp(T_IDXS_MPC, MODEL_T_IDXS, pos_y_arr) 
        yaw = np.interp(T_IDXS_MPC, MODEL_T_IDXS, yaw_arr)

        rel_pos = pos_x - pos_x[0]
        rel_pos = np.maximum(rel_pos, 0.0)
        return v_pred, rel_pos, yaw, pos_y

    def _compute_safe_speeds(self, v_pred, yaw_rates):
        raw_lat_limits = np.interp(v_pred, LAT_LIMIT_BP, LAT_LIMIT_V) * self.aggressiveness

        if self.filtered_lat_limits is None:
            self.filtered_lat_limits = raw_lat_limits
        else:
            self.filtered_lat_limits = (LPF_ALPHA * raw_lat_limits) + \
                                       ((1.0 - LPF_ALPHA) * self.filtered_lat_limits)

        current_lat_limits = np.maximum(self.filtered_lat_limits, 1.0)
        v_clip = np.clip(v_pred, 1.0, 100.0)
        curvatures = np.abs(yaw_rates / v_clip)
        
        # 這裡改成套用動態計算的 self.safety_speed_factor
        safe_speeds = np.sqrt(current_lat_limits / (curvatures + 1e-6)) * self.safety_speed_factor
        return safe_speeds, curvatures

    def _compute_sp_decel(self, predicted_lat_acc_max):
        if predicted_lat_acc_max <= DECEL_BP[0]: return 0.0
        final_decel = interp_clamped(predicted_lat_acc_max, DECEL_BP, DECEL_V)
        return clamp(final_decel, EMERGENCY_DECEL, 0.0)

    def _compute_dtsc_decel(self, v_ego, v_pred, rel_pos, safe_speeds):
        speed_excess = v_pred - safe_speeds
        if np.all(speed_excess <= 0.0): return 0.0, None, None

        critical_idx = int(np.argmax(speed_excess))
        critical_rel_dist = max(rel_pos[critical_idx], 1.0)

        decel_by_distance = (safe_speeds[critical_idx] ** 2 - v_ego ** 2) / (2.0 * critical_rel_dist)
        decel_by_distance = min(decel_by_distance, 0.0)

        if critical_rel_dist <= MIN_CURVE_DISTANCE:
            mode = 'EMERGENCY'
        else:
            mode = 'COMFORT'
            decel_by_distance = max(decel_by_distance, MAX_COMFORT_DECEL)
        return decel_by_distance, critical_idx, mode

    def get_suggested_speed(self):
        return self.suggested_speed

    def get_mpc_constraints(self, model_msg, v_ego, base_a_min, base_a_max, **kwargs):
        # 10 秒檢查一次 UI 開關
        self.toggle_check_timer += DT_MPC
        if self.toggle_check_timer >= 10.0:
            self.is_enabled = self.params.get_bool("dp_lon_dtsc")
            self.toggle_check_timer = 0.0

        horizon_len = len(T_IDXS_MPC)
        a_min = np.ones(horizon_len) * (base_a_min if np.isscalar(base_a_min) else base_a_min[0])
        a_max = np.array(base_a_max) if not np.isscalar(base_a_max) else np.ones(horizon_len) * base_a_max

        if not self.is_enabled:
            self.active = False
            self.suggested_speed = V_CRUISE_MAX
            self.output_v_target = V_CRUISE_MAX
            self.output_a_target = a_max[0]
            self.smoothed_a_target = 0.0
            return a_min, a_max

        if not self._is_model_valid(model_msg):
            self.filtered_lat_limits = None 
            self.lpf_reset_timer = 0
            self.active = False
            self.smoothed_a_target = 0.0
            return a_min, a_max

        v_pred, rel_pos, yaw_rates, pred_y = self._compute_model_arrays(model_msg)
        predicted_lat_accels = np.abs(v_pred * yaw_rates)
        predicted_lat_acc_max = float(np.max(predicted_lat_accels))

        if predicted_lat_acc_max < LPF_RESET_LAT_ACC_THRESHOLD:
             self.lpf_reset_timer += DT_MPC
             if self.lpf_reset_timer > LPF_RESET_TIME:
                 self.filtered_lat_limits = None
                 self.lpf_reset_timer = 0
        else:
             self.lpf_reset_timer = 0

        safe_speeds, curvatures = self._compute_safe_speeds(v_pred, yaw_rates)
        raw_suggested_speed = float(np.min(safe_speeds)) if len(safe_speeds) > 0 else V_CRUISE_MAX

        sp_decel = self._compute_sp_decel(predicted_lat_acc_max) 
        dt_decel, critical_idx, dt_mode = self._compute_dtsc_decel(v_ego, v_pred, rel_pos, safe_speeds)

        speed_excess = v_pred - safe_speeds
        mask_curve = curvatures > CURVATURE_MIN_FOR_PERSIST
        mask_speed = speed_excess > 0.01
        mask = np.logical_and(mask_speed, mask_curve)
        persistence_ok = (float(np.sum(mask)) / len(mask)) >= PERSISTENCE_MIN_FRAC if len(mask) > 0 else False
        critical_dist = rel_pos[critical_idx] if critical_idx is not None else 999.0

        if predicted_lat_acc_max < SCCV_ABORT_PRED_LAT_ACC_TH:
            dt_decel = sp_decel = 0.0
            dt_mode = None
            raw_suggested_speed = V_CRUISE_MAX 
        elif not persistence_ok and critical_dist < SHORT_DIST_IGNORE:
            dt_decel = sp_decel = 0.0
            dt_mode = None
            raw_suggested_speed = V_CRUISE_MAX

        final_required_decel = dt_decel if dt_mode == "EMERGENCY" else min(sp_decel, dt_decel)
        final_required_decel = clamp(final_required_decel, EMERGENCY_DECEL, 0.0)

        # ==========================================================
        # [智慧備援機制 (v27 高速免疫版參數)]
        # ==========================================================
        check_idx = 5 
        current_y = pred_y[check_idx]      
        current_yaw = yaw_rates[check_idx] 
        current_lane_deviation = abs(current_y)
        backup_triggered = False

        is_cutting_corner = (current_y * current_yaw) > 0

        if is_cutting_corner:
            actual_dev_th = INNER_DEV_TH        
            gain_per_10cm = INNER_GAIN_PER_10CM 
            max_backup_decel = INNER_MAX_DECEL  
            dev_type_str = "切西瓜(Inner)"
        else:
            actual_dev_th = OUTER_DEV_TH        
            gain_per_10cm = OUTER_GAIN_PER_10CM 
            max_backup_decel = OUTER_MAX_DECEL  
            dev_type_str = "外拋(Outer)⚠️"

        if predicted_lat_acc_max > BACKUP_LAT_G_TH and current_lane_deviation > actual_dev_th:
            excess_dev_m = current_lane_deviation - actual_dev_th
            excess_units = excess_dev_m * 10.0
            extra_brake = excess_units * gain_per_10cm
            backup_required = BACKUP_BASE_DECEL - extra_brake
            backup_required = max(backup_required, max_backup_decel)

            if backup_required < final_required_decel:
                final_required_decel = backup_required
                backup_triggered = True

                if FILE_LOG_ENABLED and (time.monotonic() - self.last_log_time > 0.2):
                    log_msg = f"備援觸發[{dev_type_str}]: G={predicted_lat_acc_max:.2f}, Dev={current_lane_deviation:.2f}m, Req={backup_required:.2f}"
                    write_file_log(log_msg)
                    self.last_log_time = time.monotonic()
        # ==========================================================

        # ==========================================================
        # [Candy 融合：狀態死咬 (Hysteresis Recovery)]
        # ==========================================================
        is_recovering = False
        if self.active:
            # 判斷是否在恢復期：煞車還沒放平，或是目標速度還沒爬完
            brake_recovering = self.smoothed_a_target < -0.05
            speed_recovering = self.output_v_target < (raw_suggested_speed - 0.5)
            is_recovering = brake_recovering or speed_recovering

        if final_required_decel < -0.1 or backup_triggered:
            self.hysteresis_timer = HYSTERESIS_TIME
            self.active = True
        else:
            if self.hysteresis_timer > 0 or (self.active and is_recovering):
                if self.hysteresis_timer > 0:
                    self.hysteresis_timer -= DT_MPC
                self.active = True
                final_required_decel = 0.0  # 進入平滑釋放模式
            else:
                self.active = False
                self.hysteresis_timer = 0

        # ==========================================================
        # [Candy 融合：黃金右腳濾波 (速度 + 加速度雙重平滑)]
        # ==========================================================
        if self.active:
            # A. 加速度平滑 (LPF)
            raw_a_target = float(clamp(final_required_decel, EMERGENCY_DECEL, MAX_EXIT_ACCEL))
            alpha_a = 0.15 
            self.smoothed_a_target = (1.0 - alpha_a) * self.smoothed_a_target + (alpha_a * raw_a_target)

            # B. 速度階梯爬升 (Staircase)
            if raw_suggested_speed < self.output_v_target:
                # 遇到更急的彎，瞬間下拉以保證安全
                self.output_v_target = raw_suggested_speed
            else:
                # 出彎時，每秒最多爬升 2.0 m/s，營造順暢推背感
                max_dv_per_step = 2.0 * DT_MPC
                self.output_v_target = min(self.output_v_target + max_dv_per_step, raw_suggested_speed)
        else:
            self.smoothed_a_target = 0.0
            self.output_v_target = V_CRUISE_MAX

        self.suggested_speed = self.output_v_target
        self.output_a_target = self.smoothed_a_target  # 供 Planner 判斷煞車狀態

        # ==========================================================
        # [輸出至 MPC 陣列 (將平滑後的極限餵給大腦)]
        # ==========================================================
        if self.active:
            pass_decel = self.smoothed_a_target if self.smoothed_a_target < 0 else 0.0
            critical_distance = rel_pos[critical_idx] if critical_idx is not None else np.max(rel_pos)
            critical_distance = max(critical_distance, 1e-3)

            # 實體車身檢測 (0.1G)：方向盤尚未回正，車身還在明顯彎中
            is_physically_in_curve = predicted_lat_accels[0] > 1.0

            for i in range(horizon_len):
                if rel_pos[i] <= critical_distance + 1e-6:
                    if pass_decel < 0:
                        a_max[i] = min(a_max[i], pass_decel)
                else:
                    if is_physically_in_curve:
                        a_max[i] = min(a_max[i], MAX_EXIT_ACCEL)

        for i in range(horizon_len):
            if a_max[i] < a_min[i]:
                a_min[i] = a_max[i] - 0.05

        return a_min, a_max

    def update(self, sm, v_ego, a_ego, v_cruise):
        # 1. 安全讀取 HTD 介入狀態 (避免初始化時 sm['controlsStateExt'] 尚未準備好)
        try:
            htd_is_active = sm['controlsStateExt'].htdAction
        except Exception:
            htd_is_active = False

        # 2. 根據狀態設定「目標」安全係數
        if htd_is_active:
            self.target_safety_speed_factor = 0.90
        else:
            self.target_safety_speed_factor = 0.70

        # 3. 執行平滑過渡 (Ramping)
        # DT_MPC 預設為 0.05 秒，每次呼叫 update 時推進計算
        if self.safety_speed_factor < self.target_safety_speed_factor:
            self.safety_speed_factor += self.ramp_up_rate * DT_MPC
            self.safety_speed_factor = min(self.safety_speed_factor, self.target_safety_speed_factor)
            
        elif self.safety_speed_factor > self.target_safety_speed_factor:
            self.safety_speed_factor -= self.ramp_down_rate * DT_MPC
            self.safety_speed_factor = max(self.safety_speed_factor, self.target_safety_speed_factor)
