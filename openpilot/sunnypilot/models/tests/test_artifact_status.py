import unittest
from dataclasses import dataclass, field
from pathlib import Path

from openpilot.sunnypilot.models.artifact_status import bundle_artifacts_ready


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


class TestArtifactStatus(unittest.TestCase):
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
