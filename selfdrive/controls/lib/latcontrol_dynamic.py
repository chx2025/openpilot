from cereal import car
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque

class LatControlDynamic(LatControl):
  def __init__(self, CP, CI, dt):
    super().__init__(CP, CI, dt)
    # 同時初始化兩個控制器 (DP版)
    self.angle_ctrl = LatControlAngle(CP, CI, dt)
    self.torque_ctrl = LatControlTorque(CP, CI, dt)

    # 預設使用 CP 讀出來的設定值
    self.use_angle = (CP.steerControlType == car.CarParams.SteerControlType.angle)

  # ==========================================
  # 🌟 新增：Torque 專屬代理轉發 (Proxy)
  # ==========================================
  @property
  def extension(self):
    # 當外層要求 extension 時，直接把 torque_ctrl 的 extension 交出去
    return self.torque_ctrl.extension

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, frictionCoefficient):
    # 將即時參數更新指令轉發給內部的 torque_ctrl
    if hasattr(self.torque_ctrl, 'update_live_torque_params'):
      self.torque_ctrl.update_live_torque_params(latAccelFactor, latAccelOffset, frictionCoefficient)
  # ==========================================

  # ⚠️ 修正：拿掉 calibrated_pose，配合 DP 底層架構
  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay):
    # ✅ 保留您要的第 3 點修正：恢復「安全直行狀態」的保護機制，防止過彎中硬切換
    is_safe_to_switch = abs(CS.steeringAngleDeg) < 10.0 and abs(CS.steeringRateDeg) < 5.0

    # 1. 判斷主控權與遲滯區間，並且鎖死過彎時的切換 (維持 DP 原版的 8.33 / 5.56 m/s 設定)
    if CS.vEgo > 8.33 and not self.use_angle and is_safe_to_switch:
      self.use_angle = True
      self.angle_ctrl.reset()  # 確保角度控制器狀態乾淨
      
    elif CS.vEgo < 5.56 and self.use_angle and is_safe_to_switch:
      self.use_angle = False
      self.torque_ctrl.reset() # 確保扭矩控制器狀態乾淨
      if hasattr(self.torque_ctrl, 'pid'):
        self.torque_ctrl.pid.reset() # 徹底清除積分

    # 2. Angle 控制器永遠運算 (幾何計算，無風險) - 移除 calibrated_pose
    _, a_steer, a_log = self.angle_ctrl.update(active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay)

    # 3. Torque 控制器永遠運算 (熱備援) - 移除 calibrated_pose
    torque_is_frozen = True if self.use_angle else steer_limited_by_safety
    t_steer, _, t_log = self.torque_ctrl.update(active, CS, VM, params, torque_is_frozen, desired_curvature, curvature_limited, lat_delay)

    # 4. 雙輸出合併：回傳 (扭矩輸出, 角度輸出, 當前主控的Log)
    if self.use_angle:
      return t_steer, a_steer, a_log
    else:
      return t_steer, a_steer, t_log

  def reset(self):
    super().reset()
    self.angle_ctrl.reset()
    self.torque_ctrl.reset()
    if hasattr(self.torque_ctrl, 'pid'):
      self.torque_ctrl.pid.reset()
