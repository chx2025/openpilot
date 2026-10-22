"""Cancel the running compile, restart the eGPU manager, queue a fresh compile.

The manager caches TG_FLAGS_AMD at import, so a change to the compile environment needs the process
restarted, not just new code on disk. procd brings it straight back.
"""
import os
import signal
import subprocess
import sys
import time

from openpilot.common.params import Params

REF = 'bf3e3631b3f91d92a1020a5e0dd4298b93ff4244'
MANAGER = 'dragonpilot.egpu.manager'


def procs_named(name):
  hits = []
  for pid in os.listdir('/proc'):
    if not pid.isdigit():
      continue
    try:
      raw = open(f'/proc/{pid}/cmdline', 'rb').read()
    except OSError:
      continue
    if name in [t.decode(errors='replace') for t in raw.split(b'\0') if t]:
      hits.append(int(pid))
  return hits


p = Params()
p.remove('dp_egpu_download_ref')
for _ in range(90):
  if not subprocess.run(['pgrep', '-f', 'egpu/compile.py'], capture_output=True, text=True).stdout.strip():
    break
  time.sleep(1)
print('compile procs:', subprocess.run(['pgrep', '-af', 'egpu/compile.py'], capture_output=True, text=True).stdout.strip() or 'none')

old = procs_named(MANAGER)
print('manager before:', old)
for pid in old:
  os.kill(pid, signal.SIGKILL)

new = []
for _ in range(60):
  time.sleep(1)
  new = [pid for pid in procs_named(MANAGER) if pid not in old]
  if new:
    break
print('manager after:', new)
if not new:
  sys.exit('manager did not come back')

time.sleep(3)
p.put('dp_egpu_download_ref', REF, block=True)
print('queued ->', p.get('dp_egpu_download_ref'))
print('status', p.get('dp_egpu_status'))
