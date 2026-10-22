"""dp - eGPU: the card's temperature and power, from the SMU metrics table.

There is no kernel driver behind this GPU: tinygrad builds its PCIe config space and BARs in
userspace through the ASM2464 controller (tinygrad/runtime/support/usb.py), so the card never
appears on the PCI bus and /sys/class/hwmon has nothing for it - the TEMP card in the sidebar
could not be fed from sysfs. The numbers live in the SMU's metrics table instead, and only the
process that owns the device can fetch them: one SMU message plus a DRAM read over the USB link,
~2 ms. That is why this module is used from modeld_egpu and not from the UI: a second process
cannot open the device, and issuing the transfer from another thread would race tinygrad's own
USB transfers.

Offsets come from amdgpu's smu14_driver_if_v14_0.h (SmuMetrics_t), the PMFW this dock's RDNA4 card
speaks, and were checked against the live table on the unit:

  132  uint32  EnergyAccumulator
  136  uint16  AverageSocketPower       watts  (amdgpu scales it with MILLIWATT_PER_WATT)
  138  uint16  AverageTotalBoardPower   watts
  140  uint16  AvgTemperature[12]       degrees C  (amdgpu scales it with MILLIDEGREE_PER_DEGREE)
       [0] TEMP_EDGE  [1] TEMP_HOTSPOT  [2] HOTSPOT_GFX  [3] HOTSPOT_SOC  [4] TEMP_MEM  [5] VR_GFX ...

Measured on the unit while idle: socket 21 W, edge 42 C, hotspot 44 C, mem 61 C, VR 42 C, fan 0 rpm.

A different SMU version is refused rather than parsed with these offsets, and every reading is range
checked, so a wrong guess reports nothing instead of a plausible-looking lie on the windscreen.
"""
import struct
import time

from openpilot.common.swaglog import cloudlog

TABLE_SMU_METRICS = 5         # TABLE_SMU_METRICS in smu14_driver_if_v14_0.h
SOCKET_POWER_OFFSET = 136
TEMPERATURE_OFFSET = 140
TEMPERATURE_EDGE = 0
READ_SIZE = TEMPERATURE_OFFSET + 24   # AvgTemperature[TEMP_COUNT] + the fan-intake word

PERIOD = 5.0                  # seconds between readings; each one is a USB round trip
MAX_FAILURES = 3              # then stop poking the SMU in the driving loop


def supported(adev) -> bool:
  smu = getattr(adev, "smu", None)
  return smu is not None and smu.smu_mod.__name__.rsplit(".", 1)[-1] == "smu_14_0_2"


def read(adev) -> tuple[float, float] | None:
  """(celsius, watts), or None when the table did not come back sane"""
  smu = adev.smu
  smu._send_msg(smu.smu_mod.PPSMC_MSG_TransferTableSmu2Dram, TABLE_SMU_METRICS)
  raw = bytes(bytearray(adev.vram.view(smu.driver_table_paddr, READ_SIZE)[:]))
  power = struct.unpack_from("<H", raw, SOCKET_POWER_OFFSET)[0]
  temp = struct.unpack_from("<H", raw, TEMPERATURE_OFFSET + 2 * TEMPERATURE_EDGE)[0]
  if not 10 <= temp <= 120 or not 0 < power < 1000:
    return None
  return float(temp), float(power)


class Sampler:
  """Publishes the reading into dp_egpu_temp / dp_egpu_power for the onroad indicator.

  Rate limited to PERIOD and written only when a rounded value changes: those params are files on
  /data, and the big model's socket power wobbles by a watt every frame - writing each frame would
  be flash wear for no visible difference.
  """

  def __init__(self, device: str, params):
    self._params = params
    self._next = 0.0
    self._last: tuple[int, int] | None = None
    self._failures = 0
    self._adev = None
    try:
      from tinygrad.device import Device
      adev = Device[device].iface.dev_impl
    except Exception:
      return
    if supported(adev):
      self._adev = adev

  def sample(self) -> None:
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
