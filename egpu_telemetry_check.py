import os
import time

os.environ['DEV'] = 'USB+AMD:LLVM'

import tg_env  # noqa: E402  (dragonpilot/egpu/tg_env.py, on PYTHONPATH through the repo)

tg_env.check()

from tinygrad.device import Device  # noqa: E402
from dragonpilot.egpu import telemetry  # noqa: E402
from openpilot.common.params import Params  # noqa: E402

adev = Device[Device.DEFAULT].iface.dev_impl
smu = adev.smu
print('smu_mod', smu.smu_mod.__name__.rsplit('.', 1)[-1], 'supported', telemetry.supported(adev))

for table_id in (5, 8):
  try:
    t = smu.read_table(smu.smu_mod.SmuMetricsExternal_t, table_id)
    m = t.SmuMetrics
    print(f'table {table_id}: edge={m.AvgTemperature[0]} hot={m.AvgTemperature[1]} '
          f'mem={m.AvgTemperature[4]} sock_w={m.AverageSocketPower} fan={m.AvgFanRpm}')
  except Exception as e:
    print(f'table {table_id}: {type(e).__name__}: {e}')

print('read()      ->', telemetry.read(adev))
print('read_detail ->', telemetry.read_detail(adev))

params = Params()
params.remove('dp_egpu_power')
params.remove('dp_egpu_temp')
s = telemetry.Sampler(Device.DEFAULT, params)
for i in range(2):
  s.sample()
  print(f'  sample {i}: power={params.get("dp_egpu_power")} temp={params.get("dp_egpu_temp")}')
  time.sleep(0.5)
print('DONE')
