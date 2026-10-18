"""Small-model-first eGPU routing with background retry and isolated inference."""
import atexit
import time

from dragonpilot.egpu.action import EgpuAction
from dragonpilot.egpu.bundles import active_ready, get_active_bundle
from dragonpilot.egpu.constants import P_ENABLED, P_PRESENT, P_FAILED, P_LOADING, P_RUNNING, P_READY
from dragonpilot.egpu.ipc import Worker

RETRY_SECONDS = 5.0
SMALL_WARMUP_FRAMES = 3
# Same dropped-camera-frame threshold as Jetlink 0.8.7 JoiningModelState.
DROP_LIMIT = 0.0075


class RetryGate:
  """Retry loading after a cooldown, independently of vehicle control."""
  def __init__(self):
    self.failed = False
    self.retry_at = 0.0

  def fail(self, now):
    self.failed = True
    self.retry_at = now + RETRY_SECONDS

  def allow(self, now):
    return not self.failed or now >= self.retry_at

  def reset(self):
    self.__init__()


class JoiningEgpu(EgpuAction):
  egpu_joining = True

  def __init__(self, small, width, height, frame_size, params, in_control, reset_small, log,
               small_lat, small_long, worker_factory=Worker, clock=time.monotonic):
    self.small = small
    self.params, self.in_control, self.reset_small, self.log = params, in_control, reset_small, log
    self.geometry = (width, height, frame_size)
    self.factory, self.clock = worker_factory, clock
    self.small_lat, self.small_long = small_lat, small_long
    self.worker = None
    self.bundle = None
    self.gate = RetryGate()
    self.chestnut = False
    self.handovers = 0
    self.small_frames = 0
    self.frame_drop_ratio = 0.0
    self.lat_delay = 0.0
    self._flags_cache = None
    self._flags(False, False, False, False)
    atexit.register(self.close)

  def __getattr__(self, name):
    if name.startswith('_'):
      raise AttributeError(name)
    return getattr(self.small, name)

  @property
  def big_model_state(self):
    if self.chestnut:
      return 'running'
    if self.gate.failed:
      return 'unavailable'
    if self.worker and self.worker.ready.is_set():
      return 'ready'
    return 'joining' if self.worker else 'none'

  def _flags(self, loading, running, ready, failed):
    flags = (loading, running, ready, failed)
    if flags != self._flags_cache:
      previous = self._flags_cache or (None, None, None, None)
      for key, value, old in zip((P_LOADING, P_RUNNING, P_READY, P_FAILED), flags, previous):
        if value != old:
          self.params.put_bool(key, value, block=True)
      self._flags_cache = flags

  def _fail(self, reason, now):
    self.log.error(f'egpu: returning to small model: {reason}; background retry scheduled')
    if self.worker:
      self.worker.close()
    if self.chestnut:
      self.reset_small()
      self.handovers += 1
    self.chestnut = False
    self.small_frames = 0
    self.gate.fail(now)
    self._flags(False, False, False, True)

  def close(self):
    if self.worker:
      self.worker.close()

  def run(self, bufs, transforms, inputs):
    controlling, now = self.in_control(), self.clock()
    enabled = self.params.get_bool(P_ENABLED)
    present = self.params.get_bool(P_PRESENT)
    if not self.gate.failed and self.worker and (
        self.worker.error is not None or self.worker.done.is_set() or not enabled or not present or
        self.params.get_bool(P_FAILED) or (self.chestnut and self.frame_drop_ratio > DROP_LIMIT)):
      self._fail(self.worker.error or 'disconnected, disabled, watchdog or frame lag', now)

    if not self.chestnut and (self.worker is None or self.gate.failed) and enabled and present:
      if self.gate.allow(now) and (self.worker is None or self.worker.done.is_set()):
        bundle = get_active_bundle(self.params)
        if active_ready(bundle):
          self.bundle = bundle
          self.generation = int(bundle.get('generation', 0))
          overrides = bundle.get('overrides', {})
          self.LAT_SMOOTH_SECONDS = float(overrides.get('lat', '.0'))
          self.LONG_SMOOTH_SECONDS = float(overrides.get('long', '.0'))
          self.gate.reset()
          self._flags(True, False, False, False)
          try:
            self.worker = self.factory(*self.geometry, bundle)
          except Exception as exc:
            self._fail(exc, now)

    ready = self.worker is not None and self.worker.ready.is_set() and not self.gate.failed
    if ready and not self.chestnut:
      self._flags(False, False, True, False)
    if ready and not controlling and self.small_frames >= SMALL_WARMUP_FRAMES and not self.chestnut:
      self.chestnut = True
      self.handovers += 1

    if self.chestnut:
      try:
        # modeld supplied its stock delays; the eGPU bundle owns its smoothing.
        big_inputs = {k: v.copy() for k, v in inputs.items()}
        big_inputs['action_t'][0] += self.LAT_SMOOTH_SECONDS - self.small_lat
        big_inputs['action_t'][1] += self.LONG_SMOOTH_SECONDS - self.small_long
        output = self.worker.run(bufs, transforms, big_inputs)
        self._flags(False, True, False, False)
        return output
      except Exception as exc:
        self._fail(exc, now)

    self.small.lat_delay = self.lat_delay
    self.small.frame_drop_ratio = self.frame_drop_ratio
    self.small.in_control = controlling
    self.small_frames += 1
    return self.small.run(bufs, transforms, inputs)

  def get_action(self, output, prev_action, lat_action_t, long_action_t, v_ego, offset):
    return super().get_action(output, prev_action,
                              lat_action_t + self.LAT_SMOOTH_SECONDS - self.small_lat,
                              long_action_t + self.LONG_SMOOTH_SECONDS - self.small_long, v_ego, offset)
