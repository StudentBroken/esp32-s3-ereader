#!/usr/bin/env python3
"""
EBK File Converter for ESP32-S3 E-Reader
Encodes text or document pages into the custom indexed binary container format (.ebk).

Supports:
- v2 Format (Structured Text Container - Default, ~500 KB):
    On-device font rendering, multi-chapter TOC, true Landscape/Portrait reflow.
    Header (104 bytes):
      - [0x00 - 0x03] Magic: 'EBK2' (4 bytes)
      - [0x04 - 0x05] Version: 0x0002 (2 bytes)
      - [0x06 - 0x45] Title: char[64] (64 bytes, null-terminated)
      - [0x46 - 0x65] Author: char[32] (32 bytes, null-terminated)
      - [0x66 - 0x67] Total Chapters: uint16_t (2 bytes)
    Chapter TOC (40 bytes per chapter):
      - Title: char[32] (32 bytes)
      - Offset: uint32_t (4 bytes)
      - Length: uint32_t (4 bytes)
    Payload:
      - Raw UTF-8 text per chapter

- v1 Format (Pre-rendered Bitmap Container - Legacy, ~9 MB):
    Pre-baked 400x300 1-bit monochrome canvas tiles.
"""

import struct
import sys
import os
import re
import argparse
from typing import List, Tuple

# Frame Geometry for v1
FRAME_WIDTH = 400
FRAME_HEIGHT = 300
FRAME_BUFFER_SIZE = (FRAME_WIDTH * FRAME_HEIGHT) // 8  # 15,000 bytes

V1_MAGIC_BYTES = b'EBOK'
V1_HEADER_SIZE = 40
V1_FRAME_HEADER_SIZE = 4

V2_MAGIC_BYTES = b'EBK2'
V2_HEADER_SIZE = 104
V2_TOC_ENTRY_SIZE = 40


def safe_utf8_truncate(text: str, max_bytes: int) -> bytes:
    """
    Safely truncates a Unicode string so its UTF-8 encoding fits within max_bytes
    without slicing through a multi-byte sequence, then pads with null bytes.
    """
    encoded = text.encode('utf-8')
    if len(encoded) <= max_bytes:
        return encoded.ljust(max_bytes, b'\x00')
    
    # Truncate string character by character until it fits
    while len(text) > 0 and len(text.encode('utf-8')) > max_bytes:
        text = text[:-1]
    return text.encode('utf-8').ljust(max_bytes, b'\x00')


def split_text_into_chapters(text: str) -> List[Tuple[str, str]]:
    """
    Splits document text into chapters based on Markdown headers (# Chapter),
    English chapter headings ('Chapter 1', etc.), or French chapter headings ('Chapitre 1', 'Livre I', etc.).
    Falls back to single chapter if none found.
    """
    pattern = re.compile(
        r'^(?:#+\s*|\b(?:Chapter|Chapitre|Book|Livre|Part|Partie|Tome|Acte|Scène|Scene)\s+\w+[\s:]*)(.+)$',
        re.MULTILINE | re.IGNORECASE
    )
    
    matches = list(pattern.finditer(text))
    if not matches:
        return [("Chapter 1", text.strip())]

    chapters = []
    # If there's text before the first heading, treat it as Introduction / Preface
    if matches[0].start() > 0:
        pre_text = text[:matches[0].start()].strip()
        if pre_text:
            chapters.append(("Preface", pre_text))

    for i in range(len(matches)):
        start = matches[i].start()
        end = matches[i + 1].start() if i + 1 < len(matches) else len(text)
        title = matches[i].group(0).strip().lstrip('#').strip()
        body = text[start:end].strip()
        chapters.append((title, body))

    return chapters


def build_ebk_v2(title: str, author: str, chapters: List[Tuple[str, str]], output_path: str):
    """
    Constructs and writes the compact .ebk v2 structured text container.
    Preserves full UTF-8 encoding with support for French accents and special characters.
    """
    total_chapters = len(chapters)
    title_bytes = safe_utf8_truncate(title, 64)
    author_bytes = safe_utf8_truncate(author, 32)
    version = 2

    # 1. Header (104 bytes)
    header = struct.pack("<4sH64s32sH", V2_MAGIC_BYTES, version, title_bytes, author_bytes, total_chapters)

    # 2. Compute offsets for chapter payloads
    toc_size = total_chapters * V2_TOC_ENTRY_SIZE
    first_payload_offset = V2_HEADER_SIZE + toc_size

    toc_entries = []
    payload_data = bytearray()
    current_offset = first_payload_offset

    for chap_title, chap_body in chapters:
        body_encoded = chap_body.encode('utf-8')
        length = len(body_encoded)
        chap_title_bytes = safe_utf8_truncate(chap_title, 32)

        toc_entry = struct.pack("<32sII", chap_title_bytes, current_offset, length)
        toc_entries.append(toc_entry)

        payload_data.extend(body_encoded)
        current_offset += length

    # 3. Assemble and write file
    with open(output_path, "wb") as f:
        f.write(header)
        for entry in toc_entries:
            f.write(entry)
        f.write(payload_data)

    file_size = os.path.getsize(output_path)
    print(f"[EBK v2 Text] Successfully generated: '{output_path}'")
    print(f"               Total Chapters: {total_chapters} | Total File Size: {file_size:,} bytes ({(file_size/1024):.1f} KB)")
    print(f"               Fits in 6.4 MB LittleFS: {(file_size / (6.4 * 1024 * 1024) * 100):.2f}% capacity used")


def render_text_to_v1_frames(text: str, title: str) -> List[Tuple[int, bytes]]:
    """
    Legacy v1 renderer: plain text into 400x300 1-bit bitmap frames.
    """
    try:
        from PIL import Image, ImageDraw, ImageFont
        has_pillow = True
    except ImportError:
        has_pillow = False
        print("[WARN] Pillow library not found. Falling back to dummy frames.")

    frames = []
    if has_pillow:
        try:
            font = ImageFont.truetype("Arial.ttf", 16)
        except IOError:
            try:
                font = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 16)
            except IOError:
                font = ImageFont.load_default()

        margin_x = 20
        margin_y = 20
        line_height = 22
        max_lines_per_page = (FRAME_HEIGHT - (margin_y * 2) - 30) // line_height

        paragraphs = text.splitlines()
        wrapped_lines = []
        for p in paragraphs:
            if not p.strip():
                wrapped_lines.append("")
                continue
            words = p.split()
            current_line = []
            for word in words:
                test_line = " ".join(current_line + [word])
                if len(test_line) * 9 > (FRAME_WIDTH - margin_x * 2):
                    wrapped_lines.append(" ".join(current_line))
                    current_line = [word]
                else:
                    current_line.append(word)
            if current_line:
                wrapped_lines.append(" ".join(current_line))

        pages = [wrapped_lines[i:i + max_lines_per_page] for i in range(0, len(wrapped_lines), max_lines_per_page)]
        if not pages:
            pages = [["[Empty Document]"]]

        for page_num, lines in enumerate(pages, start=1):
            img = Image.new("1", (FRAME_WIDTH, FRAME_HEIGHT), color=1)
            draw = ImageDraw.Draw(img)
            draw.text((margin_x, 8), title[:25], font=font, fill=0)
            draw.line([(margin_x, 26), (FRAME_WIDTH - margin_x, 26)], fill=0, width=1)

            y = margin_y + 16
            for line in lines:
                draw.text((margin_x, y), line, font=font, fill=0)
                y += line_height

            footer = f"Page {page_num} of {len(pages)}"
            draw.text((FRAME_WIDTH - margin_x - 80, FRAME_HEIGHT - 22), footer, font=font, fill=0)
            frames.append((page_num, img.tobytes(encoder_name="raw")))
    else:
        dummy_frame = bytearray([0xFF] * FRAME_BUFFER_SIZE)
        frames.append((1, bytes(dummy_frame)))

    return frames


def build_ebk_v1(title: str, frames: List[Tuple[int, bytes]], output_path: str):
    """
    Legacy v1 bitmap container builder.
    """
    total_frames = len(frames)
    title_bytes = title.encode('utf-8')[:31].ljust(32, b'\x00')
    font_id_size = 0x0010

    header = struct.pack("<4sH32sH", V1_MAGIC_BYTES, total_frames, title_bytes, font_id_size)
    index_table_size = total_frames * 4
    first_frame_offset = V1_HEADER_SIZE + index_table_size

    frame_offsets = []
    current_offset = first_frame_offset

    for _, payload in frames:
        frame_offsets.append(current_offset)
        current_offset += V1_FRAME_HEADER_SIZE + len(payload)

    index_table = struct.pack(f"<{total_frames}I", *frame_offsets)

    with open(output_path, "wb") as f:
        f.write(header)
        f.write(index_table)
        for page_num, payload in frames:
            frame_hdr = struct.pack("<HH", page_num, len(payload))
            f.write(frame_hdr)
            f.write(payload)

    file_size = os.path.getsize(output_path)
    print(f"[EBK v1 Bitmap] Generated: '{output_path}' ({file_size:,} bytes)")


def main():
    parser = argparse.ArgumentParser(description="Convert documents/text to ESP32-S3 .ebk container format")
    parser.add_argument("input", help="Path to input text file")
    parser.add_argument("-o", "--output", help="Output .ebk file path", default=None)
    parser.add_argument("-t", "--title", help="Book title", default=None)
    parser.add_argument("-a", "--author", help="Author name", default="Unknown")
    parser.add_argument("--v1", action="store_true", help="Generate legacy v1 pre-baked bitmap format (not recommended)")
    args = parser.parse_args()

    if not os.path.exists(args.input):
        print(f"Error: Input file '{args.input}' not found.")
        sys.exit(1)

    title = args.title or os.path.splitext(os.path.basename(args.input))[0]
    output_path = args.output or (os.path.splitext(args.input)[0] + ".ebk")

    import unicodedata

    with open(args.input, "rb") as f:
        raw_bytes = f.read()

    # Try common encodings without loss: UTF-8 with BOM, standard UTF-8, Windows-1252, ISO-8859-1/Latin-1
    text = None
    detected_encoding = None
    for enc in ["utf-8-sig", "utf-8", "cp1252", "iso-8859-1", "latin-1"]:
        try:
            text = raw_bytes.decode(enc)
            detected_encoding = enc
            break
        except UnicodeDecodeError:
            continue

    if text is None:
        text = raw_bytes.decode("utf-8", errors="replace")
        detected_encoding = "utf-8 (fallback with replacement chars)"

    print(f"[EBK] Decoded input text using '{detected_encoding}' encoding.")

    # 1. Unicode NFC Normalization (combines base letters with diacritics into precomposed characters like 'é')
    text = unicodedata.normalize('NFC', text)

    # 2. Normalize French & typographical punctuation for smooth e-ink rendering
    # French non-breaking spaces (standard U+00A0, narrow U+202F, thin U+2009)
    text = text.replace('\u00a0', ' ').replace('\u202f', ' ').replace('\u2009', ' ')
    # Curly quotes and apostrophes (e.g. l’arbre -> l'arbre, “citation” -> "citation")
    text = text.replace('’', "'").replace('‘', "'")
    text = text.replace('“', '"').replace('”', '"')
    # Typographic dashes and ellipsis
    text = text.replace('—', ' -- ').replace('–', '-')
    text = text.replace('…', '...')
    # French ligatures to ensure compatibility across all fonts
    text = text.replace('œ', 'oe').replace('Œ', 'OE').replace('æ', 'ae').replace('Æ', 'AE')

    if args.v1:
        print(f"[EBK] Converting '{title}' to legacy v1 bitmap container...")
        frames = render_text_to_v1_frames(text, title)
        build_ebk_v1(title, frames, output_path)
    else:
        print(f"[EBK] Converting '{title}' ({len(text)} chars) to v2 structured text container...")
        chapters = split_text_into_chapters(text)
        build_ebk_v2(title, args.author, chapters, output_path)


if __name__ == "__main__":
    main()
