"""Private inherited IPC for the eGPU worker; no public socket or network listener."""
import mmap
import os
import queue
import subprocess
import sys
import threading
from multiprocessing.connection import Connection
from pathlib import Path

import numpy as np

LOAD_TIMEOUT = 150.0
FRAME_TIMEOUT = 0.080


def background():
  # sched_setscheduler(0) applies to the calling Linux thread, not modeld's frame thread.
  os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
  cpus = set(range(os.cpu_count() or 1)) - {7}
  if cpus:
    os.sched_setaffinity(0, cpus)


class Worker:
  def __init__(self, width, height, frame_size, bundle):
    self.width, self.height, self.frame_size, self.bundle = width, height, frame_size, bundle
    self.ready = threading.Event()
    self.stopped = threading.Event()
    self.done = threading.Event()
    self.error = None
    self.process = None
    self.buffer = None
    self.requests = queue.Queue(maxsize=1)
    self.thread = threading.Thread(target=self._loop, daemon=True, name='egpu-ipc')
    self.thread.start()

  def _loop(self):
    import socket
    parent = child = conn = None
    fd = None
    try:
      background()
      fd = os.memfd_create('egpu-frames', os.MFD_CLOEXEC)
      os.ftruncate(fd, 2 * self.frame_size)
      self.buffer = mmap.mmap(fd, 2 * self.frame_size)
      parent, child = socket.socketpair()
      conn = Connection(parent.detach())
      script = Path(__file__).with_name('worker.py')
      self.process = subprocess.Popen(
        [sys.executable, str(script), str(child.fileno()), str(fd), str(os.getpid())],
        pass_fds=(child.fileno(), fd), cwd=str(script.parents[2]))
      child.close()
      if self.stopped.is_set():
        return
      conn.send((self.width, self.height, self.frame_size, self.bundle))
      if not conn.poll(LOAD_TIMEOUT):
        raise TimeoutError('eGPU load/warmup timeout')
      reply = conn.recv()
      if reply != 'ready':
        raise RuntimeError(str(reply))
      self.ready.set()
      while not self.stopped.is_set():
        try:
          request = self.requests.get(timeout=0.1)
        except queue.Empty:
          if self.process.poll() is not None:
            raise RuntimeError('eGPU worker exited while waiting')
          continue
        transforms, inputs, result, event = request
        try:
          conn.send((transforms, inputs))
          # Only the background thread may block on recv of a partially written reply.
          reply = conn.recv()
          if not isinstance(reply, dict) or 'error' in reply:
            raise RuntimeError(str(reply))
          result.append(reply)
        except Exception as exc:
          result.append(exc)
          raise
        finally:
          event.set()
    except Exception as exc:
      self.error = exc
    finally:
      self.ready.clear()
      if self.process is not None:
        if self.process.poll() is None:
          self.process.kill()
        self.process.wait()
      if conn is not None:
        conn.close()
      if child is not None:
        child.close()
      # Keep mapping alive until this Worker is collected: the frame thread may
      # be copying into it when a worker exits. No background mmap.close race.
      if fd is not None:
        os.close(fd)
      self.done.set()

  def run(self, bufs, transforms, inputs):
    if not self.ready.is_set() or self.error is not None or self.stopped.is_set():
      raise RuntimeError(f'eGPU worker unavailable: {self.error}')
    for i, key in enumerate(('img', 'big_img')):
      data = bufs[key].data if hasattr(bufs[key], 'data') else bufs[key]
      source = np.frombuffer(data, dtype=np.uint8, count=self.frame_size)
      destination = np.ndarray((self.frame_size,), dtype=np.uint8, buffer=self.buffer, offset=i * self.frame_size)
      np.copyto(destination, source)
    result, event = [], threading.Event()
    self.requests.put_nowait((transforms, inputs, result, event))
    if not event.wait(FRAME_TIMEOUT):
      self.close()
      raise TimeoutError(f'eGPU frame exceeded {FRAME_TIMEOUT * 1000:.0f} ms')
    if isinstance(result[0], Exception):
      raise result[0]
    return result[0]

  def close(self):
    self.stopped.set()
    self.ready.clear()
    # Never wait for GPU shutdown on modeld's frame thread.
    if self.process is not None and self.process.poll() is None:
      try:
        self.process.kill()
      except ProcessLookupError:
        pass
