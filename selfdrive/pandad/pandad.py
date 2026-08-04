#!/usr/bin/env python3
# simple pandad wrapper that updates the panda first
import os
import usb1
import time
import signal
import subprocess

from panda import Panda, PandaDFU, PandaProtocolMismatch, McuType, FW_PATH
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.system.hardware import HARDWARE
from openpilot.common.swaglog import cloudlog


def get_expected_signature() -> bytes:
  fn = os.path.join(FW_PATH, McuType.H7.config.app_fn)
  return Panda.get_signature_from_firmware(fn)

def flash_panda(panda_serial: str):
  panda = Panda(panda_serial)
  fw_signature = get_expected_signature()
  internal_panda = panda.is_internal()

  panda_version = "bootstub" if panda.bootstub else panda.get_version()
  panda_signature = b"" if panda.bootstub else panda.get_signature()
  cloudlog.warning(f"Panda {panda_serial} connected, version: {panda_version}, signature {panda_signature.hex()[:16]}, expected {fw_signature.hex()[:16]}")

  if panda.bootstub or panda_signature != fw_signature:
    cloudlog.info("Panda firmware out of date, update required")
    panda.flash()
    cloudlog.info("Done flashing")

  if panda.bootstub:
    bootstub_version = panda.get_version()
    cloudlog.info(f"Flashed firmware not booting, flashing development bootloader. {bootstub_version=}, {internal_panda=}")
    if internal_panda:
      HARDWARE.recover_internal_panda()
    panda.recover(reset=(not internal_panda))
    cloudlog.info("Done flashing bootstub")

  if panda.bootstub:
    cloudlog.info("Panda still not booting, exiting")
    raise AssertionError

  panda_signature = panda.get_signature()
  if panda_signature != fw_signature:
    cloudlog.info("Version mismatch after flashing, exiting")
    raise AssertionError

  panda.close()


def wait_for_internal_panda_boot(timeout: float = 8.0, poll_interval: float = 0.5) -> bool:
  """
  After a GPIO reset, the internal panda takes a few seconds to boot its app.
  Poll until it comes back in normal mode (not bootstub) instead of immediately
  querying it and mistaking the reset boot window for a genuine firmware problem.
  Returns True if the panda came back in normal (non-bootstub) mode.
  """
  attempts = max(1, int(timeout / poll_interval))
  for _ in range(attempts):
    panda_serials = Panda.list()
    if len(panda_serials) == 1:
      try:
        with Panda(panda_serials[0]) as p:
          if not p.bootstub:
            return True
      except Exception:
        pass
    time.sleep(poll_interval)
  return False


def main() -> None:
  # signal pandad to close the relay and exit
  def signal_handler(signum, frame):
    cloudlog.info(f"Caught signal {signum}, exiting")
    nonlocal do_exit
    do_exit = True
    if process is not None:
      process.send_signal(signal.SIGINT)

  process = None
  do_exit = False
  signal.signal(signal.SIGINT, signal_handler)

  # check health for lost heartbeat
  try:
    for s in Panda.list():
      with Panda(s) as p:
        health = p.health()
        if p.is_internal() and health["heartbeat_lost"]:
          Params().put_bool("PandaHeartbeatLost", True, block=True)
          cloudlog.event("heartbeat lost", deviceState=health)
  except Exception:
    cloudlog.exception("pandad.uncaught_exception")

  count = 0
  no_internal_panda_count = 0
  while not do_exit:
    try:
      cloudlog.event("pandad.flash_and_connect", count=count)
      count += 1

      panda_serials = Panda.list()

      # Only touch the internal panda's reset/DFU lines when it's actually
      # missing. Previously this ran unconditionally every loop (alternating
      # reset/recover), which meant panda was reset or forced into DFU on
      # every single pass even when it was already up and running fine -
      # this was the main cause of the slow "Panda online" time.
      if len(panda_serials) == 0:
        no_internal_panda_count += 1
        if no_internal_panda_count >= 3:
          cloudlog.info("No pandas found, putting internal panda into DFU")
          HARDWARE.recover_internal_panda()
          time.sleep(3)  # wait to come back up
          no_internal_panda_count = 0
        else:
          cloudlog.info("No pandas found, resetting internal panda")
          HARDWARE.reset_internal_panda()
          # Wait for the panda to boot back into its normal (non-bootstub) app
          # before deciding whether it actually needs to be reflashed.
          wait_for_internal_panda_boot()

      # Flash all Pandas in DFU mode
      for serial in PandaDFU.list():
        cloudlog.info(f"Panda in DFU mode found, flashing recovery {serial}")
        PandaDFU(serial).recover()
        time.sleep(1)

      panda_serials = Panda.list()
      if len(panda_serials):
        assert len(panda_serials) == 1
        no_internal_panda_count = 0
        cloudlog.info(f"{len(panda_serials)} panda found, connecting - {panda_serials}")
        flash_panda(panda_serials[0])

        # run real pandad
        os.environ['MANAGER_DAEMON'] = 'pandad'
        process = subprocess.Popen(["./pandad"], cwd=os.path.join(BASEDIR, "selfdrive/pandad"))
        process.wait()
    # TODO: wrap all panda exceptions in a base panda exception
    except (usb1.USBErrorNoDevice, usb1.USBErrorPipe):
      # a panda was disconnected while setting everything up. let's try again
      cloudlog.exception("Panda USB exception while setting up")
    except PandaProtocolMismatch:
      cloudlog.exception("pandad.protocol_mismatch")
    except Exception:
      cloudlog.exception("pandad.uncaught_exception")


if __name__ == "__main__":
  main()
