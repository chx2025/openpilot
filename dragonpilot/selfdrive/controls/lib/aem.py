"""
AEM (Automatic Experimental Mode) - Anti-Ghost Braking Version
功能：
1. 紅燈減速、綠燈快速恢復 ACC
2. 針對無號誌斑馬線的抗干擾機制 (Debounce)
3. 平滑化急迫度數值，配合 High Slew Rate Planner
4. [新增] 高速抑制遲滯邏輯 (70開啟，低於65才關閉)
5. [新增] 穩定前車檢測邏輯
6. [修正] 座標軸 X 讀取、濾波器平滑歸零、前車初次偵測防抖
"""

import numpy as np
from openpilot.selfdrive.modeld.constants import ModelConstants

# ==============================================================================
#                               CONFIG (參數設定區)
# ==============================================================================
class Config:
    # --- 速度定義 ---
    HIGHWAY_SPEED_ON  = 60.0
    HIGHWAY_SPEED_OFF = 55.0
    HIGHWAY_EMERGENCY_DIST = 20.0     # [新增] 高速緊急保底距離(公尺)：低於此距離無視抑制

    # --- 靈敏度曲線 (KPH) ---
    SENSITIVITY_BP   = [0.,  50., 80., 110.]
    SENSITIVITY_VALS = [1.0, 1.0, 0.85, 0.4]

    # --- 減速模型 (M/S 對應 距離) ---
    SLOW_DOWN_BP   = [0.,  5.,   10.,  15.,  20.,  25.,   30.]
    SLOW_DOWN_DIST = [5.,  25.,  50.,  75.,  100., 130.,  160.]

    # --- 模式定義 ---
    MODE_ACC = 'acc'
    MODE_BLENDED = 'blended'
    
    # --- 穩定前車檢測參數 ---
    LEAD_STABLE_TIME_SEC = 1.0        # 前車需穩定存在1秒
    LEAD_STABLE_FRAMES = 20           # 對應20幀（20Hz）
    LEAD_STABLE_DIST_VAR = 3.0        # 前車距離變化允許的最大波動 (米/幀)
    LEAD_MIN_DIST = 2.0               # 最小前車距離 (太近視為不穩定)
    
    # --- 實驗模式進入/退出機制 (統一管理魔法數字) ---
    ENTER_TRIGGER_THRESHOLD = 0.45    # 進入實驗模式的急迫度門檻
    ENTER_CONFIRMATION_FRAMES = 5     # 需要連續高於門檻的幀數
    EXIT_TRIGGER_THRESHOLD = 0.45     # 退出實驗模式的急迫度門檻
    EXIT_DEBOUNCE_FRAMES = 5          # 需要連續低於閾值5幀才完全退出

# ==============================================================================
#                         UTILITY CLASSES (工具類別)
# ==============================================================================

class SmoothKalmanFilter:
  """簡化的濾波器，僅保留核心平滑運算"""
  def __init__(self, initial_value=0.0):
    self.x = initial_value
    self.P = 1.0
    self.R = 0.2
    self.Q = 0.01
    self.initialized = False

  def add_data(self, measurement):
    if not self.initialized:
      self.x = measurement
      self.initialized = True
      return

    # 標準卡爾曼更新
    self.P = self.P + self.Q
    K = self.P / (self.P + self.R)
    
    # 混合平滑因子 (固定為優化後的 0.85 效果)
    smoothing_factor = 0.85
    effective_K = K * (1.0 - smoothing_factor) + smoothing_factor * 0.1
    
    self.x = self.x + effective_K * (measurement - self.x)
    self.P = (1 - effective_K) * self.P

  def get_value(self):
    return self.x if self.initialized else 0.0

class ModeTransitionManager:
  """模式切換管理器"""
  def __init__(self):
    self.current_mode = Config.MODE_ACC
    self.mode_confidence = {Config.MODE_ACC: 1.0, Config.MODE_BLENDED: 0.0}
    self.low_urgency_counter = 0

  def request_mode(self, mode, confidence=1.0):
    # 如果請求 ACC 且信心很高 (代表綠燈)，加速信心回升
    step = 0.2 if (mode == Config.MODE_ACC and confidence >= 0.9) else 0.05

    target_conf = min(1.0, self.mode_confidence[mode] + step * confidence)
    self.mode_confidence[mode] = target_conf

    for m in self.mode_confidence:
      if m != mode:
        self.mode_confidence[m] = max(0.0, self.mode_confidence[m] - step)

    threshold = 0.75 if mode != self.current_mode else 0.4
    if self.mode_confidence[mode] > threshold:
        self.current_mode = mode

  def update(self, urgency_val):
    if self.current_mode == Config.MODE_BLENDED:
        if urgency_val < Config.EXIT_TRIGGER_THRESHOLD:
            self.low_urgency_counter += 1
        else:
            self.low_urgency_counter = 0
        
        if self.low_urgency_counter >= Config.EXIT_DEBOUNCE_FRAMES:
            self.mode_confidence[Config.MODE_BLENDED] *= 0.7
    else:
        self.low_urgency_counter = 0
    
    self.mode_confidence[Config.MODE_BLENDED] *= 0.95
    self.mode_confidence[Config.MODE_ACC] = 1.0 - self.mode_confidence[Config.MODE_BLENDED]

  def get_mode(self):
    return self.current_mode

# ==============================================================================
#                               CORE LOGIC (核心邏輯)
# ==============================================================================

class AEM:
    def __init__(self):
        self._mode_manager = ModeTransitionManager()
        self._slow_down_filter = SmoothKalmanFilter()
        self._urgency = 0.0
        
        self._high_urgency_counter = 0
        self._highway_suppression_active = False
        
        # 穩定前車檢測
        self._prevent_experiment_mode = False
        self._experiment_blocked_reason = ''
        
        # 前車追蹤變數
        self._lead_stable_counter = 0
        self._lead_unstable_counter = 0
        self._lead_confidence = 0.0
        self._last_lead_dRel = float('inf')
        self._last_lead_vRel = 0.0
        self._current_lead_id = None
        self._lead_same_target_frames = 0
        self._lead_detected_frames = 0
        
        # 實驗模式狀態追蹤
        self._experiment_mode_active = False
        self._experiment_enter_counter = 0
        self._last_lead_dist_when_entered = float('inf')
        
        # 歷史數據追蹤
        self._lead_distance_history = []
        self._max_history_frames = 30

    def get_mode(self, current_mode_str):
        return self._mode_manager.get_mode()

    def update_states(self, model_msg, radar_msg, v_ego):
        """主邏輯更新"""
        if len(model_msg.position.x) != ModelConstants.IDX_N:  # [修正] 改為檢查 x 陣列長度
            return

        v_kph = v_ego * 3.6
        # [修復 Critical Bug]: 改抓 position.x，代表車頭正前方的縱向距離
        model_end_dist = model_msg.position.x[ModelConstants.IDX_N - 1]

        # ============== 1. 穩定前車檢測 ==============
        self._update_lead_stability(radar_msg, v_ego)
        
        # ============== 2. 檢查是否可以進入實驗模式 ==============
        can_enter_experiment = self._can_enter_experiment_mode(model_msg, radar_msg, v_ego, v_kph)
        
        if not can_enter_experiment:
            self._high_urgency_counter = 0
            # [修正]: 使用 add_data 平滑下降，避免暴力歸零 (self._slow_down_filter.x = 0.0) 導致的頓挫
            self._slow_down_filter.add_data(0.0)
            self._urgency = self._slow_down_filter.get_value()
            
            self._mode_manager.request_mode(Config.MODE_ACC, confidence=1.0)
            self._mode_manager.update(self._urgency)
            
            if self._experiment_mode_active:
                self._experiment_mode_active = False
                self._experiment_enter_counter = 0
                self._last_lead_dist_when_entered = float('inf')
            return

        # ============== 3. 計算紅綠燈減速邏輯 ==============
        self._calculate_slow_down(model_end_dist, v_ego, v_kph)

        # ============== 4. 決策與模式切換 ==============
        # [修正] 統一使用 Config 內的常數
        if self._urgency > Config.ENTER_TRIGGER_THRESHOLD:
            self._high_urgency_counter += 1
        else:
            self._high_urgency_counter = 0

        if self._high_urgency_counter >= Config.ENTER_CONFIRMATION_FRAMES:
            self._mode_manager.request_mode(Config.MODE_BLENDED, confidence=min(1.0, self._urgency))
            
            if not self._experiment_mode_active:
                self._experiment_mode_active = True
                self._experiment_enter_counter += 1
                
                if radar_msg and radar_msg.leadOne and radar_msg.leadOne.status:
                    self._last_lead_dist_when_entered = radar_msg.leadOne.dRel
        else:
            self._mode_manager.request_mode(Config.MODE_ACC, confidence=0.9)
            
            if self._experiment_mode_active:
                self._experiment_mode_active = False
                self._experiment_enter_counter = 0
                self._last_lead_dist_when_entered = float('inf')

        # ============== 5. 更新管理器狀態 ==============
        self._mode_manager.update(self._urgency)

    def _update_lead_stability(self, radar_msg, v_ego):
        has_lead = False
        current_lead_dRel = float('inf')
        current_lead_vRel = 0.0
        
        if radar_msg and radar_msg.leadOne and radar_msg.leadOne.status:
            has_lead = True
            current_lead_dRel = radar_msg.leadOne.dRel
            current_lead_vRel = radar_msg.leadOne.vRel
            
            if current_lead_dRel < Config.LEAD_MIN_DIST:
                has_lead = False
        
        if has_lead:
            self._lead_detected_frames += 1
            
            # [修正]: 處理前車初次出現的第一幀，避免速度差導致直接判定為不穩定
            if self._last_lead_dRel == float('inf'):
                is_same_target = True
            else:
                distance_variance = abs(current_lead_dRel - self._last_lead_dRel)
                speed_variance = abs(current_lead_vRel - self._last_lead_vRel)
                is_same_target = (distance_variance < Config.LEAD_STABLE_DIST_VAR and 
                                 speed_variance < 2.0)
            
            if is_same_target:
                self._lead_same_target_frames += 1
                self._lead_stable_counter = min(
                    Config.LEAD_STABLE_FRAMES, 
                    self._lead_stable_counter + 1
                )
                self._lead_unstable_counter = max(0, self._lead_unstable_counter - 1)
            else:
                self._lead_same_target_frames = 1
                self._lead_stable_counter = 0
                self._lead_unstable_counter += 1
            
            if self._lead_same_target_frames >= Config.LEAD_STABLE_FRAMES:
                self._lead_confidence = min(1.0, self._lead_stable_counter / Config.LEAD_STABLE_FRAMES)
            else:
                self._lead_confidence = 0.0
            
            self._lead_distance_history.append(current_lead_dRel)
            if len(self._lead_distance_history) > Config.LEAD_STABLE_FRAMES:
                self._lead_distance_history.pop(0)
            
            self._last_lead_dRel = current_lead_dRel
            self._last_lead_vRel = current_lead_vRel
            
        else:
            self._lead_detected_frames = 0
            self._lead_same_target_frames = 0
            self._lead_stable_counter = max(0, self._lead_stable_counter - 1)
            self._lead_unstable_counter += 1
            self._lead_confidence = 0.0
            self._last_lead_dRel = float('inf')
            self._last_lead_vRel = 0.0
            self._lead_distance_history = []

    def _can_enter_experiment_mode(self, model_msg, radar_msg, v_ego, v_kph):
        has_lead = False
        lead_dRel = float('inf')
        
        if radar_msg and radar_msg.leadOne and radar_msg.leadOne.status:
            has_lead = True
            lead_dRel = radar_msg.leadOne.dRel
            
            if lead_dRel < Config.LEAD_MIN_DIST:
                has_lead = False
        
        if not has_lead:
            return True
        
        experiment_stop_dist = self._calculate_experiment_stop_distance(model_msg, v_ego, v_kph)
        
        if self._lead_confidence < 0.8:
            return True
        
        if lead_dRel > experiment_stop_dist * 1.5:
            return True
        
        if lead_dRel < experiment_stop_dist:
            if (self._lead_same_target_frames >= Config.LEAD_STABLE_FRAMES and 
                self._lead_confidence >= 0.8):
                self._experiment_blocked_reason = (
                    f'穩定前車({lead_dRel:.1f}m) < AEM停車距離({experiment_stop_dist:.1f}m)'
                )
                return False
        
        return True

    def _calculate_experiment_stop_distance(self, model_msg, v_ego, v_kph):
        base_expected = np.interp(v_ego, Config.SLOW_DOWN_BP, Config.SLOW_DOWN_DIST)
        sensitivity = np.interp(v_kph, Config.SENSITIVITY_BP, Config.SENSITIVITY_VALS)
        experiment_stop_dist = base_expected * sensitivity * 1.1
        return experiment_stop_dist

    def _calculate_slow_down(self, model_end_dist, v_ego, v_kph):
        base_expected = np.interp(v_ego, Config.SLOW_DOWN_BP, Config.SLOW_DOWN_DIST)
        sensitivity = np.interp(v_kph, Config.SENSITIVITY_BP, Config.SENSITIVITY_VALS)
        expected_distance = base_expected * sensitivity * 1.1

        # [修正] 綠燈/路徑通暢時，改為平滑歸零，避免暴力歸零造成頓挫
        if model_end_dist > expected_distance:
            self._slow_down_filter.add_data(0.0) 
            self._urgency = self._slow_down_filter.get_value()
            self._high_urgency_counter = 0 
            return

        shortage_ratio = (expected_distance - model_end_dist) / max(1.0, expected_distance)
        raw_urgency = np.clip((shortage_ratio ** 1.5) * 2.5, 0.0, 1.2)

        # 高速抑制 (70開 65關 遲滯邏輯)
        if v_kph > Config.HIGHWAY_SPEED_ON:
            self._highway_suppression_active = True
        elif v_kph < Config.HIGHWAY_SPEED_OFF:
            self._highway_suppression_active = False

        if self._highway_suppression_active:
            # [新增] 緊急保底機制：如果距離極端短 (如前方突然大塞車靜止)，無視高速抑制
            if model_end_dist < Config.HIGHWAY_EMERGENCY_DIST:
                pass  # 放行原始急迫度
            else:
                raw_urgency = min(raw_urgency, 0.4)

        self._slow_down_filter.add_data(raw_urgency)
        self._urgency = self._slow_down_filter.get_value()
