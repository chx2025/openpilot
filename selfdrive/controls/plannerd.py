#!/usr/bin/env python3
from cereal import car
from openpilot.common.params import Params
from openpilot.common.realtime import Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.ldw import LaneDepartureWarning
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner, DPFlags
import cereal.messaging as messaging


def main():
  config_realtime_process(5, Priority.CTRL_LOW)

  cloudlog.info("plannerd is waiting for CarParams")
  params = Params()
  CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  cloudlog.info("plannerd got CarParams: %s", CP.brand)

  ldw = LaneDepartureWarning()
  longitudinal_planner = LongitudinalPlanner(CP)
  
  # 保留發布頻道，解決 KeyError
  pm = messaging.PubMaster(['longitudinalPlan', 'longitudinalPlanDP', 'driverAssistance'])
  
  # 保留訂閱頻道，確保 DP 資料接收
  sm = messaging.SubMaster([
    'carControl', 
    'carState', 
    'controlsState', 
    'liveParameters', 
    'radarState', 
    'modelV2', 
    'selfdriveState',
    'controlsStateExt'
  ], poll='modelV2')

  dp_flags = 0

  # 僅保留新版中確實存在的參數，避免 UnknownKeyName 報錯
  if params.get_bool("dp_lon_ocm") and hasattr(DPFlags, 'OCM'):
    dp_flags |= DPFlags.OCM
  if params.get_bool("dp_lon_aem") and hasattr(DPFlags, 'AEM'):
    dp_flags |= DPFlags.AEM
  if params.get_bool("dp_lon_apm") and hasattr(DPFlags, 'APM'):
    dp_flags |= DPFlags.APM
  if params.get_bool("dp_lon_dtsc") and hasattr(DPFlags, 'DTSC'):
    dp_flags |= DPFlags.DTSC
  while True:
    sm.update()
    if sm.updated['modelV2']:
      longitudinal_planner.update(sm, dp_flags)
      longitudinal_planner.publish(sm, pm)

      ldw.update(sm.frame, sm['modelV2'], sm['carState'], sm['carControl'])
      msg = messaging.new_message('driverAssistance')
      msg.valid = sm.all_checks(['carState', 'carControl', 'modelV2', 'liveParameters'])
      msg.driverAssistance.leftLaneDeparture = ldw.left
      msg.driverAssistance.rightLaneDeparture = ldw.right
      pm.send('driverAssistance', msg)


if __name__ == "__main__":
  main()
