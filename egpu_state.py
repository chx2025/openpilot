from openpilot.common.params import Params

p = Params()
for k in ('dp_egpu_status', 'dp_egpu_bundle', 'dp_egpu_failed', 'dp_egpu_present', 'dp_egpu_active'):
  print(k, '=', p.get(k))
