"""dp - eGPU: the card's temperature and power, from the SMU metrics table.

There is no kernel driver behind this GPU: tinygrad builds its PCIe config space and BARs in
userspace through the ASM2464 controller (tinygrad/runtime/support/usb.py), so the card never
appears on the PCI bus and /sys/class/hwmon has nothing for it - the TEMP card in the sidebar
could not be fed from sysfs. The numbers live in the SMU's metrics table instead, and only the
process that owns the device can fetch them: one SMU message plus a read of the struct the PMFW
copied into DRAM, ~2 ms over the USB link. That is why this module is used from modeld_egpu and not
from the UI: a second process cannot open the device, and issuing the transfer from another thread
would race tinygrad's own USB transfers.

The struct is read through tinygrad's own generated definitions - `SmuMetricsExternal_t` is what
tinygrad's am_smi uses for this card family (MP0 not in (13,0,6), (13,0,12), (13,0,15)) - but the
table id is NOT `smu_mod.SMU_TABLE_SMU_METRICS`: that autogen enum comes from a different PMFW
variant and numbers metrics 8, while this firmware's smu14_driver_if_v14_0.h defines
TABLE_SMU_METRICS as 5. Reading table 8 returns a struct whose edge temperature is out of range (the
range check below rejects it, which is how this was caught) whereas 5 gives the real numbers.

Measured on the unit: idle after a power cycle 11 W / edge 34 C, idle once warm 21 W / 42 C, and
during the big model's benchmark socket power in the tens of watts with edge in the 40s.

A different SMU version is refused rather than parsed with another PMFW's layout, and every reading
is range checked, so a wrong guess reports nothing instead of a plausible-looking lie on the
windscreen.
"""
import time

from openpilot.common.swaglog import cloudlog

# smu14_driver_if_v14_0.h: TABLE_SMU_METRICS 5 (the #define list, not the shifted autogen enum)
TABLE_SMU_METRICS = 5

# TEMP_e, from the autogen module: TEMP_EDGE is the one the sidebar shows
TEMPERATURE_EDGE = 0
TEMPERATURE_HOTSPOT = 1
TEMPERATURE_MEM = 4
TEMPERATURE_VR = 5

PERIOD = 5.0                  # seconds between readings; each one is a USB round trip
MAX_FAILURES = 3              # then stop poking the SMU in the driving loop

# 10 kHz units in the metrics table (amdgpu multiplies by 10 to get kHz)
CLOCK_DIVISOR = 100


def supported(adev) -> bool:
  smu = getattr(adev, "smu", None)
  if smu is None or not hasattr(smu, "read_table"):
    return False
  mod = getattr(smu, "smu_mod", None)
  return (getattr(mod, "__name__", "").rsplit(".", 1)[-1] == "smu_14_0_2"
          and hasattr(mod, "SmuMetricsExternal_t"))


def _metrics(adev):
  """The metrics struct, as the PMFW just copied it into DRAM"""
  smu = adev.smu
  table = smu.read_table(smu.smu_mod.SmuMetricsExternal_t, TABLE_SMU_METRICS)
  return table.SmuMetrics


def read(adev) -> tuple[float, float] | None:
  """(celsius, watts), or None when the table did not come back sane"""
  m = _metrics(adev)
  power = m.AverageSocketPower
  temp = m.AvgTemperature[TEMPERATURE_EDGE]
  if not 10 <= temp <= 120 or not 0 < power < 1000:
    return None
  return float(temp), float(power)


def read_detail(adev) -> dict | None:
  """Everything in the table that tells you why a long run died, for the compile log.

  Not used in the driving loop: read() already answers what the sidebar needs. This one exists
  because the big model's compile dies after ~165 s of sustained load, and "was the card at 95 C
  with the fan stopped" is the question that separates a thermal death from a link death."""
  if not supported(adev):
    return None
  m = _metrics(adev)
  temps = list(m.AvgTemperature)
  if not 10 <= temps[TEMPERATURE_EDGE] <= 130:
    return None
  return {
    "sock_w": m.AverageSocketPower, "brd_w": m.AverageTotalBoardPower,
    "edge": temps[TEMPERATURE_EDGE], "hot": temps[TEMPERATURE_HOTSPOT],
    "mem": temps[TEMPERATURE_MEM], "vr": temps[TEMPERATURE_VR],
    "fan_rpm": m.AvgFanRpm, "fan_pwm": m.AvgFanPwm,
    "gfx_mhz": m.CurrClock[adev.smu.smu_mod.PPCLK_GFXCLK] // CLOCK_DIVISOR,
    "mem_mhz": m.CurrClock[adev.smu.smu_mod.PPCLK_UCLK] // CLOCK_DIVISOR,
    "gfx_busy": m.AverageGfxActivity,
  }


class Sampler:
  """Publishes the reading into dp_egpu_temp / dp_egpu_power for the onroad indicator.

  Rate limited to PERIOD and written only when a rounded value changes: those params are files on
  /data, and the big model's socket power wobbles by a watt every frame - writing each frame would
  be flash wear for no visible difference.
  """

  def __init__(self, device: str, params):
    self._device = device
    self._params = params
    self._next = 0.0
    self._last: tuple[int, int] | None = None
    self._failures = 0
    self._adev = None
    self._resolved = False

  def _open(self) -> bool:
    """The device is looked up on the first sample, not at load: by then inference has run and the
    device is certainly open, whereas resolving it during load would touch the GPU on a path that
    a timeout is watching"""
    self._resolved = True
    try:
      from tinygrad.device import Device
      adev = Device[self._device].iface.dev_impl
    except Exception:
      cloudlog.warning(f"egpu: cannot reach the device at {self._device} for its SMU metrics table")
      return False
    if not supported(adev):
      smu = getattr(adev, "smu", None)
      cloudlog.warning(f"egpu: SMU {getattr(getattr(smu, 'smu_mod', None), '__name__', None)} has no known "
                       "metrics layout, no power/temperature this drive")
      return False
    self._adev = adev
    return True

  def sample(self) -> None:
    if not self._resolved and not self._open():
      return
    if self._adev is None:
      return
    now = time.monotonic()
    if now < self._next:
      return
    self._next = now + PERIOD
    try:
      reading = read(self._adev)
    except Exception:
      self._failures += 1
      if self._failures == 1:
        cloudlog.warning("egpu: SMU metrics read failed, no power/temperature this drive")
      if self._failures >= MAX_FAILURES:
        self._adev = None
      return
    if reading is None:
      return
    temp, power = reading
    value = (int(round(power)), int(round(temp)))
    if value == self._last:
      return
    self._last = value
    self._params.put("dp_egpu_power", value[0])
    self._params.put("dp_egpu_temp", value[1])
