#!/usr/bin/env python3

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Priority, config_realtime_process
from openpilot.sunnypilot.selfdrive.traffic_control.controller import TrafficControlConfig, TrafficControlMode
from openpilot.sunnypilot.selfdrive.traffic_control.obstacle_state import (
  TrafficControlStrategy,
  TrafficObstacleGoPolicy,
  TrafficObstacleSource,
)


def _enum_param(params: Params, key: str, enum_type, default):
  try:
    return enum_type(int(params.get(key, return_default=True)))
  except (TypeError, ValueError):
    return default


def build_source(params: Params) -> TrafficObstacleSource:
  reference_dm = params.get("TeslaTrafficStopReference", return_default=True)
  try:
    reference = float(np.clip(float(reference_dm) / 10.0, 2.0, 12.0))
  except (TypeError, ValueError):
    reference = 6.0
  strategy = _enum_param(
    params, "TeslaTrafficControlStrategy", TrafficControlStrategy, TrafficControlStrategy.stopProfile,
  )
  config = TrafficControlConfig(
    mode=_enum_param(params, "TeslaTrafficControlMode", TrafficControlMode, TrafficControlMode.off),
    default_stop_reference=reference,
    adaptive_reference=params.get_bool("TeslaTrafficAdaptiveReference"),
    retain_event_with_lead=strategy == TrafficControlStrategy.obstacleChannel,
  )
  go_policy = _enum_param(
    params, "TeslaTrafficObstacleGoPolicy", TrafficObstacleGoPolicy, TrafficObstacleGoPolicy.passive,
  )
  return TrafficObstacleSource(config, go_policy=go_policy)


def main() -> None:
  config_realtime_process(5, Priority.CTRL_LOW)
  source = build_source(Params())
  services = ['carControl', 'carState', 'radarState', 'modelV2', 'carStateSP']
  sm = messaging.SubMaster(services, poll='modelV2', ignore_alive=['carStateSP'],
                           ignore_avg_freq=['carStateSP'], ignore_valid=['carStateSP'])
  pm = messaging.PubMaster(['trafficObstacleState'])

  while True:
    sm.update()
    if sm.updated['modelV2']:
      pm.send('trafficObstacleState', source.update(sm, sm.logMonoTime['modelV2']))


if __name__ == "__main__":
  main()
