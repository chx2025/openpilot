"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import asyncio
import hashlib
import http.server
import os
import tempfile
import threading
import unittest
from typing import Any
from unittest import mock

import requests
from urllib3.connectionpool import HTTPConnectionPool

from openpilot.cereal import custom
from openpilot.common.test import OpenpilotTestCase
from openpilot.common.file_chunker import get_chunk_name, get_manifest_path
from openpilot.selfdrive.test.helpers import http_server_context
from openpilot.sunnypilot.models import manager as manager_module
from openpilot.sunnypilot.models.manager import ModelManagerSP

CHUNK_BODIES = [b'A' * 5000, b'B' * 5000, b'C' * 3000]
WHOLE_BODY = b'Z' * 9000


def sha256(data: bytes) -> str:
  return hashlib.sha256(data).hexdigest()


class DownloadHandler(http.server.BaseHTTPRequestHandler):
  """Serves the fixture bodies. Class attributes are reset per test."""
  request_paths: list[str] = []
  fail_paths: dict[str, int] = {}
  stall_paths: set[str] = set()
  stall_event: threading.Event | None = None

  def log_message(self, format: str, *args: Any) -> None:  # noqa: A002
    pass

  def _body_for(self, path):
    if path.endswith('.whole'):
      return WHOLE_BODY
    for i in range(len(CHUNK_BODIES)):
      if path.endswith(get_chunk_name('', i, len(CHUNK_BODIES))):
        return CHUNK_BODIES[i]
    return None

  def do_GET(self):
    type(self).request_paths.append(self.path)

    status = type(self).fail_paths.get(self.path)
    if status:
      self.send_response(status)
      self.end_headers()
      return

    body = self._body_for(self.path)
    if body is None:
      self.send_response(404)
      self.end_headers()
      return

    self.send_response(200)
    self.send_header('Content-Length', str(len(body)))
    self.end_headers()

    if self.path in type(self).stall_paths:
      # write a little, then wait so the test can cancel mid-transfer
      self.wfile.write(body[:100])
      self.wfile.flush()
      if type(self).stall_event is not None:
        type(self).stall_event.wait(timeout=5)
      self.wfile.write(body[100:])
    else:
      self.wfile.write(body)


class ManagerDownloadTestBase(OpenpilotTestCase):
  def setUp(self):
    super().setUp()
    DownloadHandler.request_paths = []
    DownloadHandler.fail_paths = {}
    DownloadHandler.stall_paths = set()
    DownloadHandler.stall_event = None

    self._tmp = tempfile.TemporaryDirectory()
    self.addCleanup(self._tmp.cleanup)
    self.dest = self._tmp.name

    self.reported: list[float] = []

    self.manager = ModelManagerSP.__new__(ModelManagerSP)
    self.manager.params = mock.MagicMock()
    self.manager.params.get.return_value = b'0'  # not cancelled
    self.manager.pm = mock.MagicMock()
    self.manager.pm.send.side_effect = self._record_progress
    self.manager.selected_bundle = None
    self.manager.active_bundle = None
    self.manager.available_models = []
    self.manager.model_fetcher = mock.MagicMock()
    self.manager.model_fetcher.is_usbgpu = False
    self.manager._chunk_size = 1024
    self.manager._download_start_times = {}

  def _record_progress(self, *args) -> None:
    """Runs on every real _report_status send."""
    artifact = getattr(self, 'artifact', None)
    if artifact is not None:
      self.reported.append(float(artifact.downloadProgress.progress))

  def make_artifact(self, chunked: bool):
    bundle = custom.ModelManagerSP.ModelBundle.new_message()
    bundle.init('models', 1)
    artifact = bundle.models[0].artifact
    artifact.fileName = 'driving_test_tinygrad.pkl'
    if chunked:
      artifact.downloadUri.uri = self.base_url + '/driving_test_tinygrad.pkl'
      artifact.downloadUri.sha256 = sha256(b''.join(CHUNK_BODIES))
      artifact.init('chunks', len(CHUNK_BODIES))
      for i, body in enumerate(CHUNK_BODIES):
        artifact.chunks[i].sha256 = sha256(body)
    else:
      artifact.downloadUri.uri = self.base_url + '/driving_test_tinygrad.pkl.whole'
      artifact.downloadUri.sha256 = sha256(WHOLE_BODY)
    self._bundle = bundle
    self.artifact = artifact
    return artifact

  def chunk_paths(self, base_path):
    return [get_chunk_name(base_path, i, len(CHUNK_BODIES)) for i in range(len(CHUNK_BODIES))]

  def assert_no_partials(self, base_path):
    leftovers = [p for p in [base_path, get_manifest_path(base_path)] + self.chunk_paths(base_path)
                 if os.path.isfile(p)]
    assert leftovers == [], f"partial files left behind: {leftovers}"


class TestManagerDownload(ManagerDownloadTestBase):
  """Exercises the real _download_file / _download_chunked against a local server."""

  def run_with_server(self, fn):
    with http_server_context(handler=DownloadHandler) as (host, port):
      self.base_url = f'http://{host}:{port}'
      return fn()

  def test_download_file_writes_exact_bytes(self):
    def body():
      artifact = self.make_artifact(chunked=False)
      path = os.path.join(self.dest, artifact.fileName)
      asyncio.run(self.manager._download_file(artifact.downloadUri.uri, path, artifact))
      with open(path, 'rb') as f:
        written = f.read()
      assert written == WHOLE_BODY
      assert sha256(written) == artifact.downloadUri.sha256
      assert artifact.fileName not in self.manager._download_start_times
    self.run_with_server(body)

  def test_download_chunked_writes_all_chunks_and_manifest(self):
    def body():
      artifact = self.make_artifact(chunked=True)
      base_path = os.path.join(self.dest, artifact.fileName)
      asyncio.run(self.manager._download_chunked(artifact.downloadUri.uri, base_path, artifact))

      for i, expected in enumerate(CHUNK_BODIES):
        with open(get_chunk_name(base_path, i, len(CHUNK_BODIES)), 'rb') as f:
          assert f.read() == expected, f"chunk {i} body mismatch"

      with open(get_manifest_path(base_path)) as f:
        assert f.read() == str(len(CHUNK_BODIES))

      assert not os.path.isfile(base_path), "base file should be removed after chunking"
      assert artifact.fileName not in self.manager._download_start_times
    self.run_with_server(body)

  def test_progress_is_monotonic_and_bounded(self):
    def body():
      artifact = self.make_artifact(chunked=True)
      base_path = os.path.join(self.dest, artifact.fileName)
      asyncio.run(self.manager._download_chunked(artifact.downloadUri.uri, base_path, artifact))

      assert self.reported, "expected progress reports"
      for a, b in zip(self.reported, self.reported[1:], strict=False):
        assert b >= a, f"progress went backwards: {a} -> {b}"
      assert max(self.reported) <= 99.0, f"chunked progress must stay <=99 until verify, got {max(self.reported)}"
    self.run_with_server(body)

  def test_cached_chunk_verification_reports_progress_without_network(self):
    def body():
      artifact = self.make_artifact(chunked=True)
      base_path = os.path.join(self.dest, artifact.fileName)
      for path, data in zip(self.chunk_paths(base_path), CHUNK_BODIES, strict=True):
        with open(path, 'wb') as f:
          f.write(data)

      asyncio.run(self.manager._process_artifact(artifact, self.dest))

      assert any(0.0 < progress < 100.0 for progress in self.reported)
      assert self.reported[-1] == 100.0
      assert artifact.downloadProgress.status == custom.ModelManagerSP.DownloadStatus.cached
      assert DownloadHandler.request_paths == []
      with open(get_manifest_path(base_path)) as f:
        assert f.read() == str(len(CHUNK_BODIES))

    self.run_with_server(body)

  def test_session_is_reused_across_chunks(self):
    """One connection pool shared across every chunk."""
    def body():
      artifact = self.make_artifact(chunked=True)
      base_path = os.path.join(self.dest, artifact.fileName)

      pools = []
      original = HTTPConnectionPool.urlopen

      def tracked(pool_self, *args, **kwargs):
        pools.append(id(pool_self))
        return original(pool_self, *args, **kwargs)

      with mock.patch.object(HTTPConnectionPool, 'urlopen', tracked):
        asyncio.run(self.manager._download_chunked(artifact.downloadUri.uri, base_path, artifact))

      assert len(pools) == len(CHUNK_BODIES), f"expected one request per chunk, got {len(pools)}"
      assert len(set(pools)) == 1, f"connection pool not reused across chunks: {len(set(pools))} pools"
    self.run_with_server(body)

  def test_http_error_propagates(self):
    def body():
      artifact = self.make_artifact(chunked=True)
      base_path = os.path.join(self.dest, artifact.fileName)
      failing = '/' + os.path.basename(get_chunk_name(artifact.downloadUri.uri, 1, len(CHUNK_BODIES)))
      DownloadHandler.fail_paths = {failing: 404}

      with self.assertRaises(requests.exceptions.HTTPError):
        asyncio.run(self.manager._download_chunked(artifact.downloadUri.uri, base_path, artifact))

      # chunk 1 failed, so its file and the manifest must not exist
      assert not os.path.isfile(get_chunk_name(base_path, 1, len(CHUNK_BODIES)))
      assert not os.path.isfile(get_manifest_path(base_path))
    self.run_with_server(body)

  def test_cancellation_mid_transfer(self):
    """Cancellation is checked inside the byte loop; it must still fire after the port."""
    def body():
      artifact = self.make_artifact(chunked=True)
      base_path = os.path.join(self.dest, artifact.fileName)
      self.manager.params.get.return_value = None  # cancelled

      with self.assertRaises(Exception) as ctx:
        asyncio.run(self.manager._download_chunked(artifact.downloadUri.uri, base_path, artifact))
      assert 'cancelled' in str(ctx.exception).lower()
      assert not os.path.isfile(get_manifest_path(base_path))
    self.run_with_server(body)

  def test_cancelled_cached_bundle_is_not_activated(self):
    """Selecting Default while a cached bundle is resolving must win the race."""
    bundle = custom.ModelManagerSP.ModelBundle.new_message()
    bundle.index = 0
    bundle.init('models', 1)
    bundle.models[0].artifact.fileName = 'already_cached.pkl'
    self.manager.params.get.return_value = None

    async def cached(*args, **kwargs):
      return None

    self.manager._process_artifact = cached
    with self.assertRaisesRegex(Exception, 'cancelled'):
      asyncio.run(self.manager._download_bundle(bundle, self.dest))

    assert self.manager.active_bundle is None
    assert not any(call.args and call.args[0] == "ModelManager_ActiveBundle" for call in self.manager.params.put.call_args_list)

  def test_activated_usbgpu_bundle_records_hardware_requirement(self):
    bundle = custom.ModelManagerSP.ModelBundle.new_message()
    bundle.index = 0
    bundle.init('models', 1)
    bundle.init('overrides', 1)
    bundle.overrides[0].key = "model_platform"
    bundle.overrides[0].value = "usbgpu"
    bundle.models[0].artifact.fileName = 'already_cached.pkl'

    async def cached(*args, **kwargs):
      return None

    self.manager._process_artifact = cached
    asyncio.run(self.manager._download_bundle(bundle, self.dest))

    self.manager.params.put_bool.assert_any_call("ModelManager_ActiveBundleRequiresUsbGpu", True, block=True)

  def test_bundle_marks_artifacts_downloading_before_cache_verification(self):
    bundle = custom.ModelManagerSP.ModelBundle.new_message()
    bundle.index = 0
    bundle.init('models', 1)
    bundle.models[0].artifact.fileName = 'already_cached.pkl'
    statuses = []

    async def inspect_status(artifact, *args, **kwargs):
      statuses.append(artifact.downloadProgress.status)

    self.manager._process_artifact = inspect_status
    asyncio.run(self.manager._download_bundle(bundle, self.dest))

    assert statuses == [custom.ModelManagerSP.DownloadStatus.downloading]

  def test_selecting_active_bundle_clears_request_without_downloading(self):
    class StopLoop(BaseException):
      pass

    class OneLoopRatekeeper:
      def __init__(self, *args, **kwargs):
        pass

      def keep_time(self):
        raise StopLoop

    bundle = custom.ModelManagerSP.ModelBundle.new_message()
    bundle.index = 4
    self.manager.model_fetcher.get_available_bundles.return_value = [bundle]
    self.manager.download = mock.MagicMock()
    self.manager.params.get.side_effect = lambda key: 4 if key == "ModelManager_DownloadIndex" else None

    with mock.patch.object(manager_module, "Ratekeeper", OneLoopRatekeeper), \
         mock.patch.object(manager_module, "validate_active_bundle"), \
         mock.patch.object(manager_module, "get_active_bundle", return_value=bundle), \
         self.assertRaises(StopLoop):
      self.manager.main_thread()

    self.manager.params.remove.assert_any_call("ModelManager_DownloadIndex")
    self.manager.download.assert_not_called()

  def test_repeat_downloads_are_stable(self):
    """Back-to-back runs must produce identical bytes and leak no start-time state."""
    def body():
      for _ in range(2):
        artifact = self.make_artifact(chunked=True)
        base_path = os.path.join(self.dest, artifact.fileName)
        asyncio.run(self.manager._download_chunked(artifact.downloadUri.uri, base_path, artifact))
        for i, expected in enumerate(CHUNK_BODIES):
          with open(get_chunk_name(base_path, i, len(CHUNK_BODIES)), 'rb') as f:
            assert f.read() == expected
        assert self.manager._download_start_times == {}
    self.run_with_server(body)


class TestManagerImports(OpenpilotTestCase):
  """Catches undeclared dependencies. aiohttp lived only in the AGNOS venv; 19.6 dropped
  it and models_manager died on device while CI stayed green."""

  def test_manager_imports(self):
    assert manager_module.ModelManagerSP is not None

  def test_no_undeclared_http_client(self):
    with open(manager_module.__file__) as f:
      src = f.read()
    assert 'import aiohttp' not in src, "aiohttp is not available on AGNOS 19.6; use requests"

  def test_download_timeout_is_explicit(self):
    connect, read = manager_module.DOWNLOAD_TIMEOUT
    assert connect > 0 and read > 0, "requests defaults to no timeout; downloads would hang forever"


@unittest.skipUnless(os.environ.get('RUN_INTEGRATION_TESTS'), 'requires external network')
class TestLiveModelManifest(OpenpilotTestCase):
  """Every artifact and chunk URL in the published manifest must resolve."""

  def test_all_manifest_urls_available(self):
    from openpilot.sunnypilot.models.fetcher import ModelFetcher

    manifest = requests.get(ModelFetcher.MODEL_URL, timeout=30).json()
    session = requests.Session()
    dead = []

    for bundle in manifest.get('bundles', []):
      for model in bundle.get('models', []):
        artifact = model['artifact']
        url = artifact['download_uri']['url']
        chunks = artifact.get('chunks', [])
        urls = ([url] if not chunks
                else [get_chunk_name(url, i, len(chunks)) for i in range(len(chunks))])
        for u in urls:
          try:
            r = session.head(u, timeout=15, allow_redirects=True)
            if r.status_code != 200:
              dead.append(f"{bundle.get('short_name')}: HTTP {r.status_code} {u}")
          except requests.RequestException as e:
            dead.append(f"{bundle.get('short_name')}: {type(e).__name__} {u}")

    assert not dead, "unreachable model URLs:\n" + "\n".join(dead)


if __name__ == '__main__':
  unittest.main()
