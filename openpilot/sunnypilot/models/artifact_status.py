from pathlib import Path


def bundle_artifacts_ready(bundle, model_root: str | Path) -> bool:
  root = Path(model_root)
  seen_artifact = False

  for model in getattr(bundle, "models", ()) or ():
    artifact = getattr(model, "artifact", None)
    file_name = getattr(artifact, "fileName", "") if artifact is not None else ""
    if not file_name:
      continue
    seen_artifact = True

    chunks = tuple(getattr(artifact, "chunks", ()) or ())
    if chunks:
      manifest = root / f"{file_name}.chunkmanifest"
      try:
        if int(manifest.read_text().strip()) != len(chunks):
          return False
      except (FileNotFoundError, OSError, ValueError):
        return False

      for index, chunk in enumerate(chunks, 1):
        chunk_name = getattr(chunk, "fileName", "") or f"{file_name}.chunk{index:02d}of{len(chunks):02d}"
        try:
          if (root / chunk_name).stat().st_size <= 0:
            return False
        except OSError:
          return False
    else:
      try:
        if (root / file_name).stat().st_size <= 0:
          return False
      except OSError:
        return False

  return seen_artifact


def chestnut_model_ready(params, *, model_root: str | Path | None = None,
                         builtin_ready: bool | None = None) -> bool:
  """Whether the official Chestnut slot has a loadable built-in or bundle artifact."""
  if builtin_ready is None:
    from openpilot.selfdrive.modeld.helpers import chestnut_compiled
    builtin_ready = chestnut_compiled()
  if builtin_ready:
    return True

  from openpilot.common.hardware.hw import Paths
  from openpilot.sunnypilot.models.helpers import get_selected_bundle

  bundle = get_selected_bundle(params, "chestnut")
  return bool(bundle is not None and bundle_artifacts_ready(bundle, model_root or Paths.model_root()))
