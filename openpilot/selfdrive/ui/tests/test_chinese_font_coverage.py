import ast
from pathlib import Path

from fontTools.ttLib import TTFont

from openpilot.selfdrive.ui.onroad.alert_localizer import localized_alert_characters
from openpilot.selfdrive.ui.translations.potools import parse_po
from openpilot.system.ui.lib.font_characters import fallback_font_characters


TRANSLATIONS_DIR = Path(__file__).resolve().parents[1] / "translations"
FONT_PATH = Path(__file__).resolve().parents[1] / ".." / "assets" / "fonts" / "NotoSansCJKsc-Regular.otf"
APPLICATION_PATH = Path(__file__).resolve().parents[1] / ".." / ".." / "system" / "ui" / "lib" / "application.py"


def _runtime_extra_font_chars() -> str:
  tree = ast.parse(APPLICATION_PATH.read_text(encoding="utf-8"))
  for node in tree.body:
    if isinstance(node, ast.Assign) and any(isinstance(target, ast.Name) and target.id == "EXTRA_FONT_CHARS" for target in node.targets):
      return ast.literal_eval(node.value)
  raise AssertionError("EXTRA_FONT_CHARS is missing from application.py")


def test_simplified_chinese_font_covers_every_translated_character():
  po_path = TRANSLATIONS_DIR / "app_zh-CHS.po"
  _, entries = parse_po(po_path)
  translated_chars = {
    char
    for entry in entries
    for translation in (entry.msgstr, *entry.msgstr_plural.values())
    for char in translation
    if not char.isspace()
  }

  with TTFont(FONT_PATH) as font:
    codepoints = set(font.getBestCmap())

  missing = sorted(char for char in translated_chars if ord(char) not in codepoints)
  assert not missing, (
    f"Simplified Chinese fallback font is missing {len(missing)} translated characters: "
    + "".join(missing)
  )


def test_simplified_chinese_font_covers_every_requested_runtime_glyph():
  po_path = TRANSLATIONS_DIR / "app_zh-CHS.po"
  requested_chars = set(map(chr, range(32, 127))) | set(_runtime_extra_font_chars()) | set(po_path.read_text(encoding="utf-8"))
  requested_chars.discard("\n")  # A line break has no drawable glyph.

  with TTFont(FONT_PATH) as font:
    codepoints = set(font.getBestCmap())

  missing = sorted(char for char in requested_chars if ord(char) not in codepoints)
  assert not missing, f"Simplified Chinese fallback font is missing runtime glyphs: {''.join(missing)}"


def test_simplified_chinese_fallback_requests_every_localized_alert_glyph():
  requested_chars = fallback_font_characters("zh-CHS", _runtime_extra_font_chars())
  control_chars = {"\n", "\r", "\t"}
  alert_chars = localized_alert_characters("zh-CHS") - control_chars

  assert not (control_chars & requested_chars)
  assert alert_chars <= requested_chars

  with TTFont(FONT_PATH) as font:
    codepoints = set(font.getBestCmap())

  missing = sorted(char for char in alert_chars if not char.isspace() and ord(char) not in codepoints)
  assert not missing, f"Simplified Chinese fallback font is missing localized alert glyphs: {''.join(missing)}"


def test_simplified_chinese_fallback_keeps_regular_weight_identity():
  with TTFont(FONT_PATH) as font:
    names = {
      name_id: font["name"].getDebugName(name_id)
      for name_id in (1, 2, 6)
    }
    weight = font["OS/2"].usWeightClass

  assert names == {
    1: "Noto Sans CJK SC",
    2: "Regular",
    6: "NotoSansCJKsc-Regular",
  }
  assert weight == 400
