"""Reject prebuilt releases missing the DM warp artifacts required at runtime."""
from pathlib import Path
import sys


DM_WARP_ARTIFACTS = (
  "dm_warp_1928x1208_tinygrad.pkl",
  "dm_warp_1344x760_tinygrad.pkl",
)


def validate_model_artifacts(root: Path) -> None:
  models = root / "openpilot/selfdrive/modeld/models"
  missing = [name for name in DM_WARP_ARTIFACTS if not (models / name).is_file() or (models / name).stat().st_size == 0]
  if missing:
    raise RuntimeError(f"Prebuilt release is missing DM warp artifacts: {', '.join(missing)}")


if __name__ == "__main__":
  validate_model_artifacts(Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[2])
