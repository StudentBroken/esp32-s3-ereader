#pragma once

#include <stdint.h>

#pragma pack(push, 1)

// ================= V1 FORMAT (Bitmap Pre-baked: "EBOK") =================
#define EBK_MAGIC_0 'E'
#define EBK_MAGIC_1 'B'
#define EBK_MAGIC_2 'O'
#define EBK_MAGIC_3 'K'

#define EBK_TITLE_MAX_LEN 32

/**
 * EBK v1 File Header (Fixed 40 bytes)
 * Pre-baked 1-bit monochrome frames.
 */
struct EbkHeader {
    char     magic[4];                    // 0x00 - 0x03: 'E', 'B', 'O', 'K'
    uint16_t total_frames;                // 0x04 - 0x05: Total screen frames in book
    char     title[EBK_TITLE_MAX_LEN];    // 0x06 - 0x25: Null-terminated book title
    uint16_t font_id_size;                // 0x26 - 0x27: Font identifier and point size
};

/**
 * EBK v1 Frame Header (Fixed 4 bytes per frame)
 */
struct EbkFrameHeader {
    uint16_t source_page;                 // Original physical book page number
    uint16_t payload_len;                 // Length in bytes of the following frame data
};

// ================= V2 FORMAT (Structured Text Container: "EBK2") =================
#define EBK2_MAGIC_0 'E'
#define EBK2_MAGIC_1 'B'
#define EBK2_MAGIC_2 'K'
#define EBK2_MAGIC_3 '2'

#define EBK2_TITLE_MAX_LEN   64
#define EBK2_AUTHOR_MAX_LEN  32
#define EBK2_CHAP_TITLE_LEN  32

/**
 * EBK v2 File Header (Fixed 104 bytes, 0x68)
 * Stores structured UTF-8 text with chapter table of contents.
 */
struct Ebk2Header {
    char     magic[4];                    // 0x00 - 0x03: 'E', 'B', 'K', '2'
    uint16_t version;                     // 0x04 - 0x05: 0x0002
    char     title[EBK2_TITLE_MAX_LEN];   // 0x06 - 0x45: Null-terminated book title (64 bytes)
    char     author[EBK2_AUTHOR_MAX_LEN]; // 0x46 - 0x65: Null-terminated author name (32 bytes)
    uint16_t total_chapters;              // 0x66 - 0x67: Total chapters in book
};

/**
 * EBK v2 Chapter Table-of-Contents Entry (Fixed 40 bytes each)
 * Follows immediately after Ebk2Header starting at byte 0x68 (104).
 */
struct Ebk2ChapterEntry {
    char     title[EBK2_CHAP_TITLE_LEN];  // Chapter heading / title (32 bytes)
    uint32_t offset;                      // Byte offset in file where chapter text starts
    uint32_t length;                      // Byte length of chapter text
};

#pragma pack(pop)

// Display & Frame Geometry
#define EBK_FRAME_WIDTH       400
#define EBK_FRAME_HEIGHT      300
#define EBK_FRAME_BUFFER_SIZE ((EBK_FRAME_WIDTH * EBK_FRAME_HEIGHT) / 8) // 15,000 bytes for 1-bit monochrome

// Index Table offset in v1 file
#define EBK_INDEX_TABLE_OFFSET sizeof(EbkHeader)

// Helper macro to calculate byte offset of v1 frame index entry
#define EBK_FRAME_INDEX_OFFSET(frame_idx) (EBK_INDEX_TABLE_OFFSET + ((frame_idx) * sizeof(uint32_t)))

// Helper macros for v2 TOC offset calculation
#define EBK2_TOC_OFFSET sizeof(Ebk2Header)
#define EBK2_CHAPTER_ENTRY_OFFSET(chap_idx) (EBK2_TOC_OFFSET + ((chap_idx) * sizeof(Ebk2ChapterEntry)))
