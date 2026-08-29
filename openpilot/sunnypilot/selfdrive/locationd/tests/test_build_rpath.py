from pathlib import Path


SCONSCRIPT = Path(__file__).parents[1] / "SConscript"


def test_locationd_uses_install_relative_runpath():
  source = SCONSCRIPT.read_text()

  assert "lenv.Literal('\\\\$$ORIGIN/models/generated')" in source
  assert 'lenv["RPATH"].append(Dir(rednose_gen_dir).abspath)' not in source
