from importlib.resources import files


def fallback_font_characters(language: str, extra_characters: str = "") -> set[str]:
  """Collect every glyph requested when loading a language fallback font."""
  translations_dir = files("openpilot.selfdrive.ui").joinpath("translations")
  characters = set(map(chr, range(32, 127))) | set(extra_characters)
  characters.update(translations_dir.joinpath(f"app_{language}.po").read_text(encoding="utf-8"))

  # Onroad alerts originate in selfdrived, outside of the normal UI PO extraction.
  from openpilot.selfdrive.ui.onroad.alert_localizer import localized_alert_characters
  characters.update(localized_alert_characters(language))
  characters.difference_update({"\n", "\r", "\t"})
  return characters
