"""Read-only rolling observer for Tesla CAN addresses without a verified DBC."""
from __future__ import annotations

from collections import Counter, defaultdict, deque
from statistics import median
import threading
import time

from openpilot.cereal import messaging


TARGET_ADDRESSES = (0x37A, 0x3A9, 0x3B1)
WINDOW_NS = 60_000_000_000
MAX_SAMPLES_PER_KEY = 6000
RECENT_SAMPLE_LIMIT = 120


class UnknownCanObserver:
  def __init__(self) -> None:
    self.lock = threading.Lock()
    self.samples: dict[tuple[int, int], deque[tuple[int, bytes]]] = defaultdict(
      lambda: deque(maxlen=MAX_SAMPLES_PER_KEY),
    )
    self.lifetime_counts: Counter[tuple[int, int]] = Counter()
    self._thread: threading.Thread | None = None

  def update(self, packets: list[tuple[int, list[tuple[int, bytes, int]]]]) -> None:
    with self.lock:
      for timestamp_ns, frames in packets:
        for address, data, source in frames:
          if address not in TARGET_ADDRESSES:
            continue
          key = (address, source)
          self.samples[key].append((timestamp_ns, bytes(data)))
          self.lifetime_counts[key] += 1

  @staticmethod
  def _byte_stats(samples: list[tuple[int, bytes]]) -> list[dict[str, int]]:
    width = max((len(data) for _, data in samples), default=0)
    stats = []
    for index in range(width):
      values = [data[index] for _, data in samples if index < len(data)]
      changes = sum(left != right for left, right in zip(values, values[1:], strict=False))
      stats.append({"index": index, "min": min(values), "max": max(values), "changes": changes})
    return stats

  def snapshot(self, now_ns: int | None = None) -> dict[str, object]:
    now_ns = time.monotonic_ns() if now_ns is None else now_ns
    cutoff = now_ns - WINDOW_NS
    frames = []
    with self.lock:
      for key in list(self.samples):
        samples = self.samples[key]
        while samples and samples[0][0] < cutoff:
          samples.popleft()
        if not samples:
          continue
        address, source = key
        sample_list = list(samples)
        periods_ms = [(right[0] - left[0]) / 1e6 for left, right in zip(sample_list, sample_list[1:], strict=False)
                      if right[0] > left[0]]
        duration_s = (sample_list[-1][0] - sample_list[0][0]) / 1e9
        frequency_hz = (len(sample_list) - 1) / duration_s if len(sample_list) > 1 and duration_s > 0 else None
        dlcs = Counter(len(data) for _, data in sample_list)
        frames.append({
          "address": f"0x{address:X}",
          "source": source,
          "sample_count": len(sample_list),
          "total_count": self.lifetime_counts[key],
          "dlc_counts": {str(dlc): count for dlc, count in sorted(dlcs.items())},
          "frequency_hz": round(frequency_hz, 2) if frequency_hz is not None else None,
          "median_period_ms": round(median(periods_ms), 2) if periods_ms else None,
          "latest_hex": sample_list[-1][1].hex(),
          "byte_stats": self._byte_stats(sample_list),
          "recent_samples": [
            {"age_ms": round((now_ns - timestamp_ns) / 1e6, 2), "hex": data.hex()}
            for timestamp_ns, data in sample_list[-RECENT_SAMPLE_LIMIT:]
          ],
        })
      lifetime_counts = {
        f"0x{address:X}/source{source}": count
        for (address, source), count in sorted(self.lifetime_counts.items())
      }
    frames.sort(key=lambda frame: (int(frame["address"], 16), frame["source"]))
    return {
      "available": bool(frames),
      "read_only": True,
      "window_seconds": WINDOW_NS // 1_000_000_000,
      "target_addresses": [f"0x{address:X}" for address in TARGET_ADDRESSES],
      "frames": frames,
      "lifetime_counts": lifetime_counts,
    }

  def start(self) -> None:
    if self._thread is not None and self._thread.is_alive():
      return
    self._thread = threading.Thread(target=self._run, name="unknown-can-observer", daemon=True)
    self._thread.start()

  def _run(self) -> None:
    sock = messaging.sub_sock("can", conflate=False, timeout=250)
    while True:
      event = messaging.recv_one_or_none(sock)
      if event is None:
        continue
      self.update([(event.logMonoTime, [(frame.address, bytes(frame.dat), frame.src) for frame in event.can])])


_OBSERVER = UnknownCanObserver()


def start_unknown_can_observer() -> None:
  _OBSERVER.start()


def unknown_can_snapshot() -> dict[str, object]:
  return _OBSERVER.snapshot()
