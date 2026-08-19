import json
import re
import string
from pathlib import Path
from unittest.mock import patch


from openpilot.common.parameterized import parameterized
from openpilot.common.basedir import BASEDIR
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.ui.translations.potools import extract_strings, parse_po
from openpilot.system.ui.lib import application
from openpilot.system.ui.lib.multilang import LANGUAGES_FILE, SYSTEM_UI_DIR, TRANSLATIONS_DIR, UI_DIR, Multilang

PERCENT_PLACEHOLDER_RE = re.compile(r"%(?:n|\d+)")
BAD_ENTITY_RE = re.compile(r'@(\w+);')
LINE_NUMBER_REF_RE = re.compile(r'^#:\s+.+:\d+(?:\s|$)')
FORMATTER = string.Formatter()
PO_DIR = Path(str(TRANSLATIONS_DIR))

with LANGUAGES_FILE.open(encoding='utf-8') as f:
  TRANSLATION_LANGUAGES = json.load(f)


def extract_placeholders(text: str) -> list[str]:
  placeholders = PERCENT_PLACEHOLDER_RE.findall(text)

  try:
    parsed = list(FORMATTER.parse(text))
  except ValueError as e:
    raise AssertionError(f"invalid brace formatting in {text!r}: {e}") from e

  for _, field_name, format_spec, conversion in parsed:
    if field_name is None:
      continue

    token = "{"
    token += field_name
    if conversion:
      token += f"!{conversion}"
    if format_spec:
      token += f":{format_spec}"
    token += "}"
    placeholders.append(token)

  return sorted(placeholders)


def load_po_text(po_path: Path) -> str:
  return po_path.read_text(encoding='utf-8')


class TestTranslations(OpenpilotTestCase):
  def test_formatted_translation_looks_up_template_before_substitution(self):
    translator = Multilang.__new__(Multilang)
    translator._translations = {
      "Only works above {speed} {unit}.": "仅在 {speed} {unit} 以上有效。",
    }

    assert translator.trf("Only works above {speed} {unit}.", speed=23, unit="km/h") == "仅在 23 km/h 以上有效。"

  def test_chinese_fallback_font_is_installed_for_raygui_controls(self):
    base_font = object()
    chinese_font = object()
    with patch.object(application.multilang, "requires_font_fallback", return_value=True), \
         patch.object(application.gui_app, "fallback_font", return_value=chinese_font), \
         patch.object(application.rl, "gui_set_font") as set_font:
      application.set_gui_font(base_font)

    set_font.assert_called_once_with(chinese_font)

  def test_translation_template_covers_runtime_ui_strings(self):
    runtime_roots = (
      Path(SYSTEM_UI_DIR),
      Path(str(UI_DIR)) / "layouts",
      Path(str(UI_DIR)) / "widgets",
      Path(str(UI_DIR)) / "onroad",
      Path(str(UI_DIR)) / "sunnypilot",
    )
    source_files = [
      str(path.relative_to(BASEDIR))
      for root in runtime_roots
      for path in root.rglob("*.py")
      if "tests" not in path.parts
    ]
    runtime_msgids = {entry.msgid for entry in extract_strings(source_files, BASEDIR)}
    _, template_entries = parse_po(PO_DIR / "app.pot")
    template_msgids = {entry.msgid for entry in template_entries}

    missing = sorted(runtime_msgids - template_msgids)
    assert not missing, f"runtime UI strings missing from app.pot: {missing}"

  def test_simplified_chinese_covers_translation_template(self):
    _, template_entries = parse_po(PO_DIR / "app.pot")
    _, chinese_entries = parse_po(PO_DIR / "app_zh-CHS.po")
    translated_msgids = {
      entry.msgid for entry in chinese_entries
      if entry.msgstr or any(entry.msgstr_plural.values())
    }

    missing = sorted(entry.msgid for entry in template_entries if entry.msgid not in translated_msgids)
    assert not missing, f"Simplified Chinese translations missing for: {missing}"

  @parameterized.expand(sorted(PO_DIR.glob("app_*.po")), ids=lambda p: p.name)
  def test_translation_plural_forms_match_catalog_header(self, po_path: Path):
    header, entries = parse_po(po_path)
    assert header is not None
    match = re.search(r"nplurals=(\d+)", header.msgstr)
    assert match is not None, f"{po_path.name}: missing nplurals header"
    expected_slots = set(range(int(match.group(1))))

    for entry in entries:
      if entry.is_plural:
        assert set(entry.msgstr_plural) == expected_slots, (
          f"{po_path.name}: {entry.msgid!r} has plural slots "
          + f"{sorted(entry.msgstr_plural)}, expected {sorted(expected_slots)}"
        )

  @parameterized.expand(sorted(TRANSLATION_LANGUAGES.values()))
  def test_translation_file_exists(self, language_code: str):
    po_path = PO_DIR / f"app_{language_code}.po"
    assert po_path.exists(), f"missing translation file: {po_path}"

  @parameterized.expand(sorted(PO_DIR.glob("app_*.po")), ids=lambda p: p.name)
  def test_translation_placeholders_are_preserved(self, po_path: Path):
    _, entries = parse_po(po_path)
    language = po_path.stem.removeprefix("app_")

    for entry in entries:
      source_placeholders = extract_placeholders(entry.msgid)

      if entry.is_plural:
        plural_placeholders = extract_placeholders(entry.msgid_plural)
        message = (
          f"{language}: source plural placeholders do not match singular for "
          + f"{entry.msgid!r}: {source_placeholders} vs {plural_placeholders}"
        )
        assert plural_placeholders == source_placeholders, message

        for idx, msgstr in sorted(entry.msgstr_plural.items()):
          if not msgstr:
            continue

          translated_placeholders = extract_placeholders(msgstr)
          message = (
            f"{language}: plural form {idx} changes placeholders for {entry.msgid!r}: "
            + f"expected {source_placeholders}, got {translated_placeholders}"
          )
          assert translated_placeholders == source_placeholders, message
      else:
        if not entry.msgstr:
          continue

        translated_placeholders = extract_placeholders(entry.msgstr)
        message = (
          f"{language}: translation changes placeholders for {entry.msgid!r}: "
          + f"expected {source_placeholders}, got {translated_placeholders}"
        )
        assert translated_placeholders == source_placeholders, message

  @parameterized.expand(sorted(PO_DIR.glob("app_*.po")), ids=lambda p: p.name)
  def test_translation_refs_do_not_include_line_numbers(self, po_path: Path):
    for line in load_po_text(po_path).splitlines():
      assert not LINE_NUMBER_REF_RE.match(line), (
        f"{po_path.name}: line-number source reference found: {line}"
      )

  @parameterized.expand(sorted(PO_DIR.glob("app_*.po")), ids=lambda p: p.name)
  def test_translation_entities_are_valid(self, po_path: Path):
    matches = BAD_ENTITY_RE.findall(load_po_text(po_path))
    assert not matches, (
      f"{po_path.name}: found '@...;' entity typo(s): {', '.join(sorted(set(matches)))}"
    )
