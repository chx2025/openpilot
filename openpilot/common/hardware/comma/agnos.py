#!/usr/bin/env python3
import hashlib
import json
import lzma
import os
import struct
import subprocess
import time
from collections.abc import Generator

import requests

SPARSE_CHUNK_FMT = struct.Struct('H2xI4x')

_COMMA_HW_DIR = os.path.dirname(os.path.abspath(__file__))
AGNOS_MANIFEST_FILE = os.path.join("openpilot", "common", "hardware", "comma", "agnos.json")

# ---------------------------------------------------------------------------
# Tunables for slow / flaky networks (sp260827-c3l)
#   Problem: at ~500 KB/s the original 60s read-timeout on the streaming
#   download tripped constantly, and every retry restarted the whole 4 GB
#   AGNOS image from byte 0 (no resume). This forked version downloads each
#   partition to a local cache file *with HTTP Range resume*, then flashes
#   it. A dropped connection now continues where it left off instead of
#   restarting. Tune the numbers below to taste.
# ---------------------------------------------------------------------------
DOWNLOAD_CONNECT_TIMEOUT = 30     # seconds to establish the TCP/TLS connection
DOWNLOAD_READ_TIMEOUT = 600      # seconds of silence between data chunks before giving up
DOWNLOAD_TIMEOUT = (DOWNLOAD_CONNECT_TIMEOUT, DOWNLOAD_READ_TIMEOUT)
DOWNLOAD_RETRIES = 30             # max download attempts (each resumes from where it stopped)
DOWNLOAD_BACKOFF = 10             # seconds to wait before retrying a failed attempt
DOWNLOAD_CHUNK = 1024 * 1024      # stream chunk size
AGNOS_CACHE_DIR_ENV = "AGNOS_CACHE_DIR"
AGNOS_CACHE_DIR_DEFAULT = "/data/agnos_cache"


def is_tizi_device() -> bool:
  try:
    with open("/sys/firmware/devicetree/base/model") as f:
      return f.read().strip('\x00').split('comma ')[-1] == 'tizi'
  except OSError:
    return False


def is_mici_device() -> bool:
  try:
    with open("/sys/firmware/devicetree/base/model") as f:
      return f.read().strip('\x00').split('comma ')[-1] == 'mici'
  except OSError:
    return False


def agnos_tici_manifest_path() -> str:
  return os.path.join(_COMMA_HW_DIR, "agnos_tici.json")


def default_agnos_manifest_path(repo_root: str) -> str:
  """Resolve agnos.json for monorepo (sp) or flat openpilot installs."""
  for rel in (
    os.path.join("openpilot", "common", "hardware", "comma", "agnos.json"),
    os.path.join("openpilot", "system", "hardware", "comma", "agnos.json"),
    os.path.join("common", "hardware", "comma", "agnos.json"),
  ):
    path = os.path.join(repo_root, rel)
    if os.path.isfile(path):
      return path
  return os.path.join(_COMMA_HW_DIR, "agnos.json")


def download_partition(url: str, dest: str, label: str, cloudlog,
                       max_retries: int = DOWNLOAD_RETRIES,
                       backoff: int = DOWNLOAD_BACKOFF) -> None:
  """Download `url` to `dest` with resume support (HTTP Range).

  On a network drop the partial file is kept and the next attempt resumes
  from its current size, so a 4 GB image does not restart from zero every
  time the link stutters. Raises after `max_retries` exhausted attempts.
  """
  os.makedirs(os.path.dirname(dest) or ".", exist_ok=True)
  for attempt in range(1, max_retries + 1):
    existing = os.path.getsize(dest) if os.path.exists(dest) else 0
    headers = {'Accept-Encoding': 'identity'}
    if existing > 0:
      headers['Range'] = f'bytes={existing}-'

    try:
      with requests.get(url, stream=True, headers=headers, timeout=DOWNLOAD_TIMEOUT) as r:
        r.raise_for_status()
        # Server honored the Range request -> append; otherwise start fresh.
        if r.status_code == 206:
          mode = 'ab'
        else:
          mode = 'wb'
          existing = 0

        cloudlog.info(f"Downloading {label} (attempt {attempt}/{max_retries}, "
                      f"resuming from {existing // 1024 // 1024} MB)")
        written = existing
        last_log = written // (100 * 1024 * 1024)
        with open(dest, mode) as f:
          for data in r.iter_content(chunk_size=DOWNLOAD_CHUNK):
            if data:
              f.write(data)
              written += len(data)
              tick = written // (100 * 1024 * 1024)
              if tick > last_log:
                last_log = tick
                cloudlog.info(f"  {label} downloaded {written // 1024 // 1024} MB")
        cloudlog.info(f"Download of {label} complete ({written // 1024 // 1024} MB)")
        return

    except (requests.exceptions.RequestException, IOError) as e:
      cloudlog.warning(f"Download attempt {attempt}/{max_retries} for {label} failed: {e}")
      if attempt < max_retries:
        time.sleep(backoff)

  raise Exception(f"Failed to download {url} after {max_retries} attempts")


class StreamingDecompressor:
  """Legacy streaming reader (kept for compatibility). Prefer LocalImageReader."""
  def __init__(self, url: str) -> None:
    self.buf = b""
    self.req = requests.get(url, stream=True, headers={'Accept-Encoding': 'identity'}, timeout=DOWNLOAD_TIMEOUT)
    self.it = self.req.iter_content(chunk_size=DOWNLOAD_CHUNK)
    self.decompressor = lzma.LZMADecompressor(format=lzma.FORMAT_AUTO)
    self.eof = False
    self.sha256 = hashlib.sha256()

  def read(self, length: int) -> bytes:
    while len(self.buf) < length and not self.eof:
      if self.decompressor.needs_input:
        self.req.raise_for_status()
        try:
          compressed = next(self.it)
        except StopIteration:
          self.eof = True
          break
      else:
        compressed = b''

      self.buf += self.decompressor.decompress(compressed, max_length=length)

      if self.decompressor.eof:
        self.eof = True
        break

    result = self.buf[:length]
    self.buf = self.buf[length:]

    self.sha256.update(result)
    return result


class LocalImageReader:
  """Reads an already-downloaded .lzma partition image and decompresses on the fly.

  Provides the same `.read()` / `.sha256` API as StreamingDecompressor but
  sources the compressed bytes from a local file, so the network-fragile
  download step can be retried independently of flashing.
  """
  def __init__(self, path: str) -> None:
    self.buf = b""
    self.f = open(path, 'rb')
    self.decompressor = lzma.LZMADecompressor(format=lzma.FORMAT_AUTO)
    self.eof = False
    self.sha256 = hashlib.sha256()

  def read(self, length: int) -> bytes:
    while len(self.buf) < length and not self.eof:
      if self.decompressor.needs_input:
        compressed = self.f.read(DOWNLOAD_CHUNK)
        if not compressed:
          self.eof = True
          break
      else:
        compressed = b''

      self.buf += self.decompressor.decompress(compressed, max_length=length)

      if self.decompressor.eof:
        self.eof = True
        break

    result = self.buf[:length]
    self.buf = self.buf[length:]

    self.sha256.update(result)
    return result


def unsparsify(f: LocalImageReader | StreamingDecompressor) -> Generator[bytes, None, None]:
  # https://source.android.com/devices/bootloader/images#sparse-format
  magic = struct.unpack("I", f.read(4))[0]
  assert(magic == 0xed26ff3a)

  # Version
  major = struct.unpack("H", f.read(2))[0]
  minor = struct.unpack("H", f.read(2))[0]
  assert(major == 1 and minor == 0)

  f.read(2)  # file header size
  f.read(2)  # chunk header size

  block_sz = struct.unpack("I", f.read(4))[0]
  f.read(4)  # total blocks
  num_chunks = struct.unpack("I", f.read(4))[0]
  f.read(4)  # crc checksum

  for _ in range(num_chunks):
    chunk_type, out_blocks = SPARSE_CHUNK_FMT.unpack(f.read(12))

    if chunk_type == 0xcac1:  # Raw
      yield f.read(out_blocks * block_sz)
    elif chunk_type == 0xcac2:  # Fill
      filler = f.read(4) * (block_sz // 4)
      for _ in range(out_blocks):
        yield filler
    elif chunk_type == 0xcac3:  # Don't care
      yield b""
    else:
      raise Exception("Unhandled sparse chunk type")


# noop wrapper with same API as unsparsify() for non sparse images
def noop(f: LocalImageReader | StreamingDecompressor) -> Generator[bytes, None, None]:
  while len(chunk := f.read(1024 * 1024)) > 0:
    yield chunk


def get_target_slot_number() -> int:
  current_slot = subprocess.check_output(["abctl", "--boot_slot"], encoding='utf-8').strip()
  return 1 if current_slot == "_a" else 0


def slot_number_to_suffix(slot_number: int) -> str:
  assert slot_number in (0, 1)
  return '_a' if slot_number == 0 else '_b'


def get_partition_path(target_slot_number: int, partition: dict) -> str:
  path = f"/dev/disk/by-partlabel/{partition['name']}"

  if partition.get('has_ab', True):
    path += slot_number_to_suffix(target_slot_number)

  return path


def get_raw_hash(path: str, partition_size: int) -> str:
  raw_hash = hashlib.sha256()
  pos, chunk_size = 0, 1024 * 1024

  with open(path, 'rb+') as out:
    while pos < partition_size:
      n = min(chunk_size, partition_size - pos)
      raw_hash.update(out.read(n))
      pos += n

  return raw_hash.hexdigest().lower()


def verify_partition(target_slot_number: int, partition: dict[str, str | int], force_full_check: bool = False) -> bool:
  full_check = partition['full_check'] or force_full_check
  path = get_partition_path(target_slot_number, partition)

  if not isinstance(partition['size'], int):
    return False

  partition_size: int = partition['size']

  if not isinstance(partition['hash_raw'], str):
    return False

  partition_hash: str = partition['hash_raw']

  if full_check:
    return get_raw_hash(path, partition_size) == partition_hash.lower()
  else:
    with open(path, 'rb+') as out:
      out.seek(partition_size)
      return out.read(64) == partition_hash.lower().encode()


def clear_partition_hash(target_slot_number: int, partition: dict) -> None:
  path = get_partition_path(target_slot_number, partition)
  with open(path, 'wb+') as out:
    partition_size = partition['size']

    out.seek(partition_size)
    out.write(b"\x00" * 64)
    os.sync()


def extract_compressed_image(target_slot_number: int, partition: dict, cloudlog):
  path = get_partition_path(target_slot_number, partition)

  cache_dir = os.environ.get(AGNOS_CACHE_DIR_ENV, AGNOS_CACHE_DIR_DEFAULT)
  tmp = os.path.join(cache_dir, f"{partition['name']}.lzma")

  try:
    # 1) Resumable download to a local cache file (network-fragile step).
    download_partition(partition['url'], tmp, partition['name'], cloudlog)

    # 2) Decompress from the local file and flash to the partition.
    downloader = LocalImageReader(tmp)
    with open(path, 'wb+') as out:
      last_p = 0
      raw_hash = hashlib.sha256()
      f = unsparsify if partition['sparse'] else noop
      for chunk in f(downloader):
        raw_hash.update(chunk)
        out.write(chunk)
        p = int(out.tell() / partition['size'] * 100)
        if p != last_p:
          last_p = p
          print(f"Installing {partition['name']}: {p}", flush=True)

      if raw_hash.hexdigest().lower() != partition['hash_raw'].lower():
        raise Exception(f"Raw hash mismatch '{raw_hash.hexdigest().lower()}'")

      if downloader.sha256.hexdigest().lower() != partition['hash'].lower():
        raise Exception("Uncompressed hash mismatch")

      if out.tell() != partition['size']:
        raise Exception("Uncompressed size mismatch")

      os.sync()
  finally:
    # Always clean up the temp download so /data does not fill up.
    try:
      os.remove(tmp)
    except OSError:
      pass


def flash_partition(target_slot_number: int, partition: dict, cloudlog, standalone=False):
  cloudlog.info(f"Downloading and writing {partition['name']}")

  if verify_partition(target_slot_number, partition):
    cloudlog.info(f"Already flashed {partition['name']}")
    return

  # Clear hash before flashing in case we get interrupted
  full_check = partition['full_check']
  if not full_check:
    clear_partition_hash(target_slot_number, partition)

  path = get_partition_path(target_slot_number, partition)

  extract_compressed_image(target_slot_number, partition, cloudlog)

  # Write hash after successful flash
  if not full_check:
    with open(path, 'wb+') as out:
      out.seek(partition['size'])
      out.write(partition['hash_raw'].lower().encode())


def swap(manifest_path: str, target_slot_number: int, cloudlog) -> None:
  update = json.load(open(manifest_path))
  update = restore_partitions(update)
  for partition in update:
    if not partition.get('full_check', False):
      clear_partition_hash(target_slot_number, partition)

  while True:
    out = subprocess.check_output(f"abctl --set_active {target_slot_number}", shell=True, stderr=subprocess.STDOUT, encoding='utf8')
    if ("No such file or directory" not in out) and ("lun as boot lun" in out):
      cloudlog.info(f"Swap successful {out}")
      break
    else:
      cloudlog.error(f"Swap failed {out}")


def flash_agnos_update(manifest_path: str, target_slot_number: int, cloudlog, standalone=False) -> None:
  update = json.load(open(manifest_path))
  update = restore_partitions(update)

  cloudlog.info(f"Target slot {target_slot_number}")

  # set target slot as unbootable
  subprocess.run(f"abctl --set_unbootable {target_slot_number}", shell=True)

  for partition in update:
    success = False

    for retries in range(10):
      try:
        flash_partition(target_slot_number, partition, cloudlog, standalone)
        success = True
        break

      except requests.exceptions.RequestException:
        cloudlog.exception("Failed")
        cloudlog.info(f"Failed to download {partition['name']}, retrying ({retries})")
        time.sleep(10)

    if not success:
      cloudlog.info(f"Failed to flash {partition['name']}, aborting")
      raise Exception("Maximum retries exceeded")


def verify_agnos_update(manifest_path: str, target_slot_number: int) -> bool:
  update = json.load(open(manifest_path))
  update = restore_partitions(update)
  return all(verify_partition(target_slot_number, partition) for partition in update)


# Implementation by Rick
# This approach differs from common solutions and required extensive trial and error.
# If you reuse or adapt this function, please provide proper credit.
def restore_partitions(partitions):
  if is_tizi_device() or is_mici_device():
    return partitions

  partition_name_to_use = {'abl', 'boot'}
  partitions_to_keep = {}
  agnos_tici_path = agnos_tici_manifest_path()

  try:
    with open(agnos_tici_path, 'r') as f:
      tici_partitions = json.load(f)

    partitions_to_keep = {p['name']: p for p in tici_partitions if p.get('name') in partition_name_to_use}

  except (OSError, json.JSONDecodeError) as e:
    print(f"Warning: Could not load TICI partition data from {agnos_tici_path}. Error: {e}")
    return partitions

  return [partitions_to_keep.get(p.get('name'), p) for p in partitions]

if __name__ == "__main__":
  import argparse
  import logging

  parser = argparse.ArgumentParser(description="Flash and verify AGNOS update",
                                   formatter_class=argparse.ArgumentDefaultsHelpFormatter)

  parser.add_argument("--verify", action="store_true", help="Verify and perform swap if update ready")
  parser.add_argument("--swap", action="store_true", help="Verify and perform swap, downloads if necessary")
  parser.add_argument("manifest", help="Manifest json")
  args = parser.parse_args()

  logging.basicConfig(level=logging.INFO)

  target_slot_number = get_target_slot_number()
  if args.verify:
    if verify_agnos_update(args.manifest, target_slot_number):
      swap(args.manifest, target_slot_number, logging)
      exit(0)
    exit(1)
  elif args.swap:
    while not verify_agnos_update(args.manifest, target_slot_number):
      logging.error("Verification failed. Flashing AGNOS")
      flash_agnos_update(args.manifest, target_slot_number, logging, standalone=True)

    logging.warning(f"Verification succeeded. Swapping to slot {target_slot_number}")
    swap(args.manifest, target_slot_number, logging)
  else:
    flash_agnos_update(args.manifest, target_slot_number, logging, standalone=True)
