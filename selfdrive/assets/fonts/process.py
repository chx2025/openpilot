
#!/usr/bin/env python3
from pathlib import Path
import json

import pyray as rl

FONT_DIR = Path(__file__).resolve().parent
SELFDRIVE_DIR = FONT_DIR.parents[1]
TRANSLATIONS_DIR = SELFDRIVE_DIR / "ui" / "translations"
LANGUAGES_FILE = TRANSLATIONS_DIR / "languages.json"

GLYPH_PADDING = 2
EXTRA_CHARS = "–‑✓×°§•X⚙✕◀▶✔⌫⇧␣○●↳çêüñ–‑✓×°§•€£¥"
UNIFONT_LANGUAGES = {"zh-CHT", "zh-CHS"}


def _languages():
  if not LANGUAGES_FILE.exists():
    return {}
  with LANGUAGES_FILE.open(encoding="utf-8") as f:
    return json.load(f)


def _char_sets():
  base = set(map(chr, range(32, 127))) | set(EXTRA_CHARS)
  
  # --- events.py 字元快取機制 ---
  EVENTS_CACHE = FONT_DIR / "events_chars.cache"

  if EVENTS_CACHE.exists():
      print(f"INFO: Loading events.py characters from cache: {EVENTS_CACHE.name}")
      try:
          cached_chars = EVENTS_CACHE.read_text(encoding="utf-8")
          base.update(set(cached_chars))
      except Exception as e:
          print(f"ERROR: Could not read events cache: {e}")
  else:
      # 只保留你指定需要檢查的路徑
      possible_paths = [
          Path("/data/openpilot/selfdrive/selfdrived/events.py"),
      ]

      found = False
      print("\n--- Searching for events.py ---")
      for events_path in possible_paths:
          if events_path.exists():
              print(f"SUCCESS: Found events.py at {events_path}")
              try:
                  content = events_path.read_text(encoding="utf-8")
                  chars = set(content)
                  base.update(chars)
                  
                  # 第一次萃取完成後，將字元寫入快取檔案
                  try:
                      EVENTS_CACHE.write_text("".join(chars), encoding="utf-8")
                      print(f"SUCCESS: Saved characters to cache at {EVENTS_CACHE.name}")
                  except Exception as cache_err:
                      print(f"WARNING: Could not write cache file: {cache_err}")
                      
                  print(f"SUCCESS: Added {len(chars)} characters from events.py")
                  found = True
                  break
              except Exception as e:
                  print(f"ERROR: Could not read file: {e}")

      if not found:
          print("WARNING: Could not find events.py! Chinese characters WILL BE MISSING in the output images.")
      print("-------------------------------\n")
  # --- 快取機制結束 ---

  labels = set(base)
  per_lang: dict[str, tuple[int, ...]] = {}

  for language, code in _languages().items():
    labels.update(language)
    
    lang_chars = set()
    # 支援多種 po 檔前綴
    for prefix in ("app_", "dragonpilot_"):
      po_path = TRANSLATIONS_DIR / f"{prefix}{code}.po"
      try:
        chars = set(po_path.read_text(encoding="utf-8"))
        lang_chars.update(chars)
      except FileNotFoundError:
        continue
        
    if code in UNIFONT_LANGUAGES:
      lang_chars_combined = set(base) | lang_chars
      per_lang[code] = tuple(sorted(ord(c) for c in lang_chars_combined))
    else:
      base.update(lang_chars)

  base_cp = tuple(sorted(ord(c) for c in base))
  labels_cp = tuple(sorted(ord(c) for c in labels))
  return base_cp, labels_cp, per_lang


def _glyph_metrics(glyphs, rects, codepoints):
  entries = []
  min_offset_y, max_extent = None, 0
  for idx, codepoint in enumerate(codepoints):
    glyph = glyphs[idx]
    rect = rects[idx]
    width = int(round(rect.width))
    height = int(round(rect.height))
    offset_y = int(round(glyph.offsetY))
    min_offset_y = offset_y if min_offset_y is None else min(min_offset_y, offset_y)
    max_extent = max(max_extent, offset_y + height)
    entries.append({
      "id": codepoint,
      "x": int(round(rect.x)),
      "y": int(round(rect.y)),
      "width": width,
      "height": height,
      "xoffset": int(round(glyph.offsetX)),
      "yoffset": offset_y,
      "xadvance": int(round(glyph.advanceX)),
    })

  if min_offset_y is None:
    raise RuntimeError("No glyphs were generated")

  line_height = int(round(max_extent - min_offset_y))
  base = int(round(max_extent))
  return entries, line_height, base


def _write_bmfont(path: Path, font_size: int, face: str, atlas_name: str, line_height: int, base: int, atlas_size, entries):
  if line_height != font_size:
    print("using font size for line height", atlas_name)
    line_height = font_size
  lines = [
    f"info face=\"{face}\" size=-{font_size} bold=0 italic=0 charset=\"\" unicode=1 stretchH=100 smooth=0 aa=1 padding=0,0,0,0 spacing=0,0 outline=0",
    f"common lineHeight={line_height} base={base} scaleW={atlas_size[0]} scaleH={atlas_size[1]} pages=1 packed=0 alphaChnl=0 redChnl=4 greenChnl=4 blueChnl=4",
    f"page id=0 file=\"{atlas_name}\"",
    f"chars count={len(entries)}",
  ]
  for entry in entries:
    lines.append(
      ("char id={id:<4} x={x:<5} y={y:<5} width={width:<5} height={height:<5} " +
       "xoffset={xoffset:<5} yoffset={yoffset:<5} xadvance={xadvance:<5} page=0  chnl=15").format(**entry)
    )
  path.write_text("\n".join(lines) + "\n")


def _process_font(font_path: Path, codepoints: tuple[int, ...], output_name: str | None = None):
  stem = output_name or font_path.stem
  atlas_name = f"{stem}.png"
  atlas_path = FONT_DIR / atlas_name
  fnt_path = FONT_DIR / f"{stem}.fnt"

  # 檢查圖集與字型設定檔是否已存在，存在則完全跳過
  if atlas_path.exists() and fnt_path.exists():
      print(f"INFO: Skipping {stem}, atlas already exists at {atlas_name}.")
      return

  font_size = 48 if font_path.stem.lower().startswith("opfont") else 120
  print(f"Processing {font_path.name} -> {stem} ({len(codepoints)} glyphs @ {font_size}px)...")

  data = font_path.read_bytes()
  file_buf = rl.ffi.new("unsigned char[]", data)
  cp_buffer = rl.ffi.new("int[]", codepoints)
  cp_ptr = rl.ffi.cast("int *", cp_buffer)
  glyphs = rl.load_font_data(rl.ffi.cast("unsigned char *", file_buf), len(data), font_size, cp_ptr, len(codepoints), rl.FontType.FONT_DEFAULT)
  if glyphs == rl.ffi.NULL:
    raise RuntimeError("raylib failed to load font data")

  rects_ptr = rl.ffi.new("Rectangle **")
  image = rl.gen_image_font_atlas(glyphs, rects_ptr, len(codepoints), font_size, GLYPH_PADDING, 0)
  if image.width == 0 or image.height == 0:
    raise RuntimeError("raylib returned an empty atlas")

  rects = rects_ptr[0]
  entries, line_height, base = _glyph_metrics(glyphs, rects, codepoints)

  if not rl.export_image(image, atlas_path.as_posix()):
    raise RuntimeError("Failed to export atlas image")

  _write_bmfont(fnt_path, font_size, stem, atlas_name, line_height, base, (image.width, image.height), entries)


def main():
  base_cp, labels_cp, per_lang = _char_sets()
  fonts = sorted(FONT_DIR.glob("*.ttf")) + sorted(FONT_DIR.glob("*.otf"))
  opfonts: list[Path] = []

  for font in fonts:
    if "emoji" in font.name.lower() or font.name == "unifont.otf":
      continue
    if font.stem.lower().startswith("opfont"):
      opfonts.append(font)
      continue
    _process_font(font, base_cp)

  if not opfonts:
    raise RuntimeError("OpFont not found (expected OpFont-*.otf in fonts dir)")

  for opfont_path in opfonts:
    weight = opfont_path.stem  # e.g. "OpFont-Regular"

    # Labels atlas: language display names + ASCII (for language selector)
    _process_font(opfont_path, labels_cp, output_name=f"{weight}-Labels")

    # Per-language atlases: ASCII + that language's .po chars
    for lang_code, lang_cp in per_lang.items():
      _process_font(opfont_path, lang_cp, output_name=f"{weight}-{lang_code}")

  return 0


if __name__ == "__main__":
  raise SystemExit(main())
