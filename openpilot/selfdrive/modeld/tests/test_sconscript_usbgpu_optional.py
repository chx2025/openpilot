from pathlib import Path


SCONSCRIPT = Path(__file__).resolve().parents[1] / "SConscript"


def test_missing_chestnut_model_does_not_block_the_default_build():
  source = SCONSCRIPT.read_text()
  model_loop = source.split("for chestnut in [False, True] if CHESTNUT else [False]:", 1)[1]
  guard = model_loop.split("camera_res_args", 1)[0]
  assert "except FileNotFoundError:" in guard
  assert "if chestnut:" in guard
  assert "downloaded compiled bundles remain usable" in guard
  assert "continue" in guard
  assert "raise" in guard
