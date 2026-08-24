import hashlib

from openpilot.common.hardware.hw import Paths
from openpilot.sunnypilot.models.fetcher import ModelParser
from openpilot.sunnypilot.models.helpers import (
  bundle_catalog_folder,
  bundle_requires_usbgpu,
  is_bundle_version_compatible,
  migrate_active_bundle_metadata,
  usbgpu_model_ready,
)
from openpilot.sunnypilot.models.tests.test_selection import FakeParams


def bundle_json(*, index=0, ref="model-ref", selector=18, file_name="model.pkl", sha256="abc", is_big=False):
  return {
    "index": index,
    "short_name": "TEST",
    "display_name": "Test Model",
    "generation": 12,
    "environment": "development",
    "runner": "tinygrad",
    "is_20hz": True,
    "minimum_selector_version": selector,
    "ref": ref,
    "is_big": is_big,
    "overrides": {},
    "models": [{
      "type": "chunked",
      "artifact": {
        "file_name": file_name,
        "download_uri": {"url": "https://example.invalid/model.pkl", "sha256": sha256},
      },
    }],
  }


def test_selector_18_is_current_and_selector_17_remains_migration_compatible():
  assert is_bundle_version_compatible({"minimumSelectorVersion": 18})
  assert is_bundle_version_compatible({"minimumSelectorVersion": 17})
  assert not is_bundle_version_compatible({"minimumSelectorVersion": 16})


def test_catalogs_use_unique_indices_and_explicit_platform_overrides():
  small = ModelParser.parse_models({"bundles": [bundle_json(index=4)]}, index_offset=0, platform="qcom")[0]
  big = ModelParser.parse_models({"bundles": [bundle_json(index=4, is_big=True)]}, index_offset=1000, platform="usbgpu")[0]

  assert small.index == 4
  assert big.index == 1004
  assert not bundle_requires_usbgpu(small)
  assert bundle_requires_usbgpu(big)
  assert bundle_catalog_folder(small).startswith("QCOM · ")
  assert bundle_catalog_folder(big).startswith("eGPU · ")


def test_parsing_catalog_does_not_create_fake_download_manifests(monkeypatch, tmp_path):
  monkeypatch.setattr(Paths, "model_root", staticmethod(lambda: str(tmp_path)))
  data = bundle_json()
  data["models"][0]["artifact"]["chunks"] = [{"file_name": "model.pkl.chunk01of01", "sha256": "abc"}]

  ModelParser.parse_models({"bundles": [data]}, platform="qcom")

  assert list(tmp_path.iterdir()) == []


def test_identical_selector_17_active_bundle_migrates_without_download(monkeypatch, tmp_path):
  model_data = b"compiled model"
  sha = hashlib.sha256(model_data).hexdigest()
  model_path = tmp_path / "model.pkl"
  model_path.write_bytes(model_data)
  monkeypatch.setattr(Paths, "model_root", staticmethod(lambda: str(tmp_path)))

  old = ModelParser.parse_models({"bundles": [bundle_json(selector=17, sha256=sha)]}, platform="usbgpu")[0]
  new = ModelParser.parse_models({"bundles": [bundle_json(index=0, selector=18, sha256=sha, is_big=True)]},
                                 index_offset=1000, platform="usbgpu")[0]
  params = FakeParams({"ModelManager_ActiveBundle": old.to_dict(), "ModelManager_ActiveBundleRequiresUsbGpu": True})

  migrated = migrate_active_bundle_metadata(params, [new])

  assert migrated
  assert params.get("ModelManager_ActiveBundle")["minimumSelectorVersion"] == 18
  assert params.get("ModelManager_ActiveBundle")["index"] == 1000
  assert params.get_bool("ModelManager_ActiveBundleRequiresUsbGpu")


def test_changed_selector_18_artifact_does_not_replace_working_selector_17_bundle(monkeypatch, tmp_path):
  old_data = b"old compiled model"
  old_sha = hashlib.sha256(old_data).hexdigest()
  (tmp_path / "old.pkl").write_bytes(old_data)
  monkeypatch.setattr(Paths, "model_root", staticmethod(lambda: str(tmp_path)))

  old = ModelParser.parse_models({"bundles": [bundle_json(selector=17, file_name="old.pkl", sha256=old_sha)]},
                                 platform="usbgpu")[0]
  new = ModelParser.parse_models({"bundles": [bundle_json(selector=18, file_name="new.pkl", sha256="new-sha", is_big=True)]},
                                 index_offset=1000, platform="usbgpu")[0]
  params = FakeParams({"ModelManager_ActiveBundle": old.to_dict(), "ModelManager_ActiveBundleRequiresUsbGpu": True})

  migrated = migrate_active_bundle_metadata(params, [new])

  assert not migrated
  assert params.get("ModelManager_ActiveBundle")["minimumSelectorVersion"] == 17
  assert params.get("ModelManager_ActiveBundle")["models"][0]["artifact"]["fileName"] == "old.pkl"


def test_qcom_tinygrad_bundle_does_not_count_as_usbgpu_model_ready(monkeypatch, tmp_path):
  model_data = b"small qcom model"
  sha = hashlib.sha256(model_data).hexdigest()
  (tmp_path / "model.pkl").write_bytes(model_data)
  monkeypatch.setattr(Paths, "model_root", staticmethod(lambda: str(tmp_path)))
  monkeypatch.setattr("openpilot.sunnypilot.models.helpers.usbgpu_compiled", lambda: False)
  small = ModelParser.parse_models({"bundles": [bundle_json(sha256=sha)]}, platform="qcom")[0]
  params = FakeParams({"ModelManager_ActiveBundle": small.to_dict()})

  assert not usbgpu_model_ready(params)
