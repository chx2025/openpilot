import unittest
from dataclasses import dataclass, field
from pathlib import Path

from openpilot.sunnypilot.models.artifact_status import bundle_artifacts_ready, chestnut_model_ready


@dataclass
class Chunk:
  fileName: str


@dataclass
class Artifact:
  fileName: str
  chunks: list[Chunk] = field(default_factory=list)


@dataclass
class Model:
  artifact: Artifact


@dataclass
class Bundle:
  models: list[Model]


class FakeParams:
  def __init__(self, values=None):
    self.values = values or {}

  def get(self, key):
    return self.values.get(key)


class TestArtifactStatus(unittest.TestCase):
  def test_downloaded_chestnut_bundle_is_ready_without_builtin_big(self):
    from tempfile import TemporaryDirectory
    with TemporaryDirectory() as directory:
      root = Path(directory)
      name = "driving_lebowski_tinygrad.pkl"
      chunk_name = f"{name}.chunk01of01"
      (root / f"{name}.chunkmanifest").write_text("1")
      (root / chunk_name).write_bytes(b"compiled model")
      params = FakeParams({
        "ModelManager_ActiveBundleChestnut": {
          "index": 0,
          "internalName": "LM",
          "displayName": "Lebowski",
          "models": [{
            "type": "chunked",
            "artifact": {
              "fileName": name,
              "downloadUri": {"uri": "", "sha256": ""},
              "chunks": [{"fileName": chunk_name, "sha256": ""}],
            },
          }],
          "generation": 12,
          "environment": "development",
          "runner": "tinygrad",
          "is20hz": True,
          "ref": "lm",
          "minimumSelectorVersion": 18,
        },
      })

      self.assertTrue(chestnut_model_ready(params, model_root=root, builtin_ready=False))

  def test_qcom_bundle_does_not_make_chestnut_ready(self):
    params = FakeParams({
      "ModelManager_ActiveBundle": {
        "internalName": "QCOM",
        "minimumSelectorVersion": 18,
      },
    })

    self.assertFalse(chestnut_model_ready(params, model_root=Path("/missing"), builtin_ready=False))

  def test_builtin_big_is_ready_without_a_downloaded_bundle(self):
    self.assertTrue(chestnut_model_ready(FakeParams(), model_root=Path("/missing"), builtin_ready=True))

  def test_chunked_bundle_is_ready_only_after_all_chunks_exist(self):
    from tempfile import TemporaryDirectory
    with TemporaryDirectory() as directory:
      root = Path(directory)
      name = "driving_big_tinygrad.pkl"
      chunks = [Chunk(f"{name}.chunk01of02"), Chunk(f"{name}.chunk02of02")]
      bundle = Bundle([Model(Artifact(name, chunks))])
      (root / f"{name}.chunkmanifest").write_text("2")
      (root / chunks[0].fileName).write_bytes(b"first")
      self.assertFalse(bundle_artifacts_ready(bundle, root))
      (root / chunks[1].fileName).write_bytes(b"second")
      self.assertTrue(bundle_artifacts_ready(bundle, root))

  def test_manifest_alone_is_not_ready(self):
    from tempfile import TemporaryDirectory
    with TemporaryDirectory() as directory:
      root = Path(directory)
      name = "driving_big_tinygrad.pkl"
      bundle = Bundle([Model(Artifact(name, [Chunk(f"{name}.chunk01of01")]))])
      (root / f"{name}.chunkmanifest").write_text("1")
      self.assertFalse(bundle_artifacts_ready(bundle, root))

  def test_regular_artifact_must_be_nonempty(self):
    from tempfile import TemporaryDirectory
    with TemporaryDirectory() as directory:
      root = Path(directory)
      name = "driving_model.pkl"
      bundle = Bundle([Model(Artifact(name))])
      (root / name).touch()
      self.assertFalse(bundle_artifacts_ready(bundle, root))
      (root / name).write_bytes(b"model")
      self.assertTrue(bundle_artifacts_ready(bundle, root))
