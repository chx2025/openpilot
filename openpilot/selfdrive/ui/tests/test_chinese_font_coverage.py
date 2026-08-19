from pathlib import Path

from fontTools.ttLib import TTFont

from openpilot.selfdrive.ui.translations.potools import parse_po


TRANSLATIONS_DIR = Path(__file__).resolve().parents[1] / "translations"
FONT_PATH = Path(__file__).resolve().parents[1] / ".." / "assets" / "fonts" / "NotoSansCJKsc-Regular.otf"


def test_simplified_chinese_font_covers_every_translated_character():
  _, entries = parse_po(TRANSLATIONS_DIR / "app_zh-CHS.po")
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
