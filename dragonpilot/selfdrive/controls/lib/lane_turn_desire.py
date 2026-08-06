"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
Adapted for dragonpilot (Metric Version - Updated Limits)
"""
from cereal import log
from openpilot.common.constants import CV
from openpilot.common.params import Params

# 最大值改為 35 km/h
LANE_CHANGE_SPEED_MIN = 35 * CV.KPH_TO_MS

class LaneTurnController:
  def __init__(self):
    self.turn_desire = log.Desire.none
    self.params = Params()
    self.param_read_counter = 0
    self.enabled = False
    # 預設值改為 20 km/h
    self.lane_turn_value = 20.0 * CV.KPH_TO_MS
    self.read_params()

  def read_params(self):
    self.enabled = self.params.get_bool("dp_lane_turn_desire")
    val = self.params.get("dp_lane_turn_value")
    value = float(val if val is not None else 20.0)
    self.lane_turn_value = min(float(LANE_CHANGE_SPEED_MIN), value * CV.KPH_TO_MS)

  def update_params(self) -> None:
    if self.param_read_counter % 50 == 0:
      self.read_params()
    self.param_read_counter += 1

  def update_lane_turn(self, blindspot_left: bool, blindspot_right: bool, left_blinker: bool, right_blinker: bool, v_ego: float) -> None:
    if left_blinker and not right_blinker and v_ego < self.lane_turn_value and not blindspot_left:
      self.turn_desire = log.Desire.turnLeft
    elif right_blinker and not left_blinker and v_ego < self.lane_turn_value and not blindspot_right:
      self.turn_desire = log.Desire.turnRight
    else:
      self.turn_desire = log.Desire.none

  def get_turn_desire(self):
    if not self.enabled:
      return log.Desire.none
    return self.turn_desire
