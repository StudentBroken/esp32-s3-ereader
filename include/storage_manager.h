#pragma once

#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>
#include <memory>
#include "ebk_format.h"

#define MAX_STORED_BOOKS 15

enum EbkFormatVersion {
    EBK_FORMAT_UNKNOWN = 0,
    EBK_FORMAT_V1_BITMAP = 1,
    EBK_FORMAT_V2_TEXT = 2
};

/**
 * Metadata descriptor for books discovered on LittleFS storage.
 */
struct StoredBookInfo {
    char     filename[32];                 // e.g. "/sample_book.ebk"
    char     title[EBK2_TITLE_MAX_LEN + 1];// Null-terminated book title e.g. "The Time Machine"
    char     author[EBK2_AUTHOR_MAX_LEN + 1];
    uint16_t total_frames;                 // For v1: total frames; for v2: total chapters
    uint16_t total_chapters;               // For v2: total chapters
    uint8_t  format_version;               // 1: v1 (bitmap), 2: v2 (text)
    size_t   file_size;                    // File size in bytes
};

class StorageManager {
public:
    static bool begin() {
        // Mount LittleFS partition named "spiffs" defined in partitions.csv
        if (!LittleFS.begin(true, "/spiffs", 10, "spiffs")) {
            Serial.println("[FS] Failed to mount LittleFS!");
            return false;
        }
        
        Serial.printf("[FS] LittleFS mounted successfully.\n");
        Serial.printf("[FS] Total Space: %u KB | Used Space: %u KB\n", 
            (unsigned int)(LittleFS.totalBytes() / 1024), 
            (unsigned int)(LittleFS.usedBytes() / 1024));
        return true;
    }

    /**
     * Identifies the EBK format version by inspecting the first 4 magic bytes.
     */
    static EbkFormatVersion get_book_version(const char* path) {
        if (!path || !LittleFS.exists(path)) return EBK_FORMAT_UNKNOWN;

        File file = LittleFS.open(path, "r");
        if (!file) return EBK_FORMAT_UNKNOWN;

        char magic[4];
        if (file.read((uint8_t*)magic, 4) != 4) {
            file.close();
            return EBK_FORMAT_UNKNOWN;
        }
        file.close();

        if (magic[0] == EBK2_MAGIC_0 && magic[1] == EBK2_MAGIC_1 &&
            magic[2] == EBK2_MAGIC_2 && magic[3] == EBK2_MAGIC_3) {
            return EBK_FORMAT_V2_TEXT;
        }

        if (magic[0] == EBK_MAGIC_0 && magic[1] == EBK_MAGIC_1 &&
            magic[2] == EBK_MAGIC_2 && magic[3] == EBK_MAGIC_3) {
            return EBK_FORMAT_V1_BITMAP;
        }

        return EBK_FORMAT_UNKNOWN;
    }

    /**
     * Reads v1 file header (backward-compatible alias).
     */
    static bool get_book_header(const char* path, EbkHeader* out_hdr) {
        return get_v1_header(path, out_hdr);
    }

    /**
     * Reads v1 file header.
     */
    static bool get_v1_header(const char* path, EbkHeader* out_hdr) {
        if (!path || !out_hdr || !LittleFS.exists(path)) return false;
        File file = LittleFS.open(path, "r");
        if (!file) return false;

        bool ok = (file.read((uint8_t*)out_hdr, sizeof(EbkHeader)) == sizeof(EbkHeader));
        file.close();
        if (!ok) return false;

        return (out_hdr->magic[0] == EBK_MAGIC_0 && out_hdr->magic[1] == EBK_MAGIC_1 &&
                out_hdr->magic[2] == EBK_MAGIC_2 && out_hdr->magic[3] == EBK_MAGIC_3);
    }

    /**
     * Reads v2 file header.
     */
    static bool get_v2_header(const char* path, Ebk2Header* out_hdr) {
        if (!path || !out_hdr || !LittleFS.exists(path)) return false;
        File file = LittleFS.open(path, "r");
        if (!file) return false;

        bool ok = (file.read((uint8_t*)out_hdr, sizeof(Ebk2Header)) == sizeof(Ebk2Header));
        file.close();
        if (!ok) return false;

        return (out_hdr->magic[0] == EBK2_MAGIC_0 && out_hdr->magic[1] == EBK2_MAGIC_1 &&
                out_hdr->magic[2] == EBK2_MAGIC_2 && out_hdr->magic[3] == EBK2_MAGIC_3);
    }

    /**
     * Unified title reader for both v1 and v2 formats.
     */
    static bool get_book_title(const char* path, char* title_buf, size_t max_len) {
        if (!title_buf || max_len == 0) return false;
        title_buf[0] = '\0';

        EbkFormatVersion ver = get_book_version(path);
        if (ver == EBK_FORMAT_V2_TEXT) {
            Ebk2Header hdr;
            if (get_v2_header(path, &hdr)) {
                strncpy(title_buf, hdr.title, max_len - 1);
                title_buf[max_len - 1] = '\0';
                return true;
            }
        } else if (ver == EBK_FORMAT_V1_BITMAP) {
            EbkHeader hdr;
            if (get_v1_header(path, &hdr)) {
                strncpy(title_buf, hdr.title, max_len - 1);
                title_buf[max_len - 1] = '\0';
                return true;
            }
        }
        return false;
    }

    /**
     * Retrieves total pages or total chapters count depending on format.
     */
    static uint16_t get_total_units(const char* path) {
        EbkFormatVersion ver = get_book_version(path);
        if (ver == EBK_FORMAT_V2_TEXT) {
            Ebk2Header hdr;
            if (get_v2_header(path, &hdr)) return hdr.total_chapters;
        } else if (ver == EBK_FORMAT_V1_BITMAP) {
            EbkHeader hdr;
            if (get_v1_header(path, &hdr)) return hdr.total_frames;
        }
        return 0;
    }

    /**
     * Reads a TOC entry for a specific chapter in a v2 book.
     */
    static bool read_v2_chapter_entry(const char* path, uint16_t chap_idx, Ebk2ChapterEntry* out_entry) {
        if (!path || !out_entry) return false;
        File file = LittleFS.open(path, "r");
        if (!file) return false;

        Ebk2Header hdr;
        if (file.read((uint8_t*)&hdr, sizeof(Ebk2Header)) != sizeof(Ebk2Header)) {
            file.close();
            return false;
        }

        if (chap_idx >= hdr.total_chapters) {
            file.close();
            return false;
        }

        uint32_t entry_offset = EBK2_CHAPTER_ENTRY_OFFSET(chap_idx);
        if (!file.seek(entry_offset)) {
            file.close();
            return false;
        }

        bool ok = (file.read((uint8_t*)out_entry, sizeof(Ebk2ChapterEntry)) == sizeof(Ebk2ChapterEntry));
        file.close();
        return ok;
    }

    /**
     * Reads a slice of raw UTF-8 text from a v2 book.
     */
    static size_t read_v2_text_chunk(const char* path, uint32_t file_offset, char* buffer, size_t max_bytes) {
        if (!path || !buffer || max_bytes == 0) return 0;
        File file = LittleFS.open(path, "r");
        if (!file) return 0;

        if (!file.seek(file_offset)) {
            file.close();
            return 0;
        }

        size_t bytes_read = file.read((uint8_t*)buffer, max_bytes - 1);
        buffer[bytes_read] = '\0';
        file.close();
        return bytes_read;
    }

    /**
     * Reads a v1 1-bit frame bitmap.
     */
    static bool read_frame(const char* path, uint16_t frame_index, EbkFrameHeader* frame_hdr, uint8_t* buffer, size_t buffer_size) {
        File file = LittleFS.open(path, "r");
        if (!file) return false;

        EbkHeader header;
        if (file.read((uint8_t*)&header, sizeof(header)) != sizeof(header)) {
            file.close();
            return false;
        }

        if (header.magic[0] != EBK_MAGIC_0 || header.magic[1] != EBK_MAGIC_1 ||
            header.magic[2] != EBK_MAGIC_2 || header.magic[3] != EBK_MAGIC_3) {
            file.close();
            return false;
        }

        if (frame_index >= header.total_frames) {
            file.close();
            return false;
        }

        uint32_t index_offset = EBK_FRAME_INDEX_OFFSET(frame_index);
        file.seek(index_offset);
        uint32_t frame_offset = 0;
        if (file.read((uint8_t*)&frame_offset, sizeof(frame_offset)) != sizeof(frame_offset)) {
            file.close();
            return false;
        }

        file.seek(frame_offset);
        if (file.read((uint8_t*)frame_hdr, sizeof(EbkFrameHeader)) != sizeof(EbkFrameHeader)) {
            file.close();
            return false;
        }

        size_t bytes_to_read = frame_hdr->payload_len;
        if (bytes_to_read > buffer_size) {
            bytes_to_read = buffer_size;
        }

        size_t read_bytes = file.read(buffer, bytes_to_read);
        file.close();
        return (read_bytes == bytes_to_read);
    }

    /**
     * Scans LittleFS filesystem root directory and extracts metadata for all .ebk files (v1 and v2).
     */
    static int get_stored_books(StoredBookInfo* out_books, int max_books) {
        if (!out_books || max_books <= 0) return 0;
        File root = LittleFS.open("/");
        if (!root || !root.isDirectory()) return 0;

        int count = 0;
        File file = root.openNextFile();
        while (file && count < max_books) {
            String name = file.name();
            if (name.endsWith(".ebk")) {
                StoredBookInfo& b = out_books[count];
                if (!name.startsWith("/")) {
                    snprintf(b.filename, sizeof(b.filename), "/%s", name.c_str());
                } else {
                    strncpy(b.filename, name.c_str(), sizeof(b.filename) - 1);
                    b.filename[sizeof(b.filename) - 1] = '\0';
                }
                b.file_size = file.size();
                b.total_frames = 0;
                b.total_chapters = 0;
                b.title[0] = '\0';
                b.author[0] = '\0';

                EbkFormatVersion ver = get_book_version(b.filename);
                b.format_version = ver;

                if (ver == EBK_FORMAT_V2_TEXT) {
                    Ebk2Header h2;
                    if (get_v2_header(b.filename, &h2)) {
                        b.total_chapters = h2.total_chapters;
                        b.total_frames = h2.total_chapters; // Base units
                        strncpy(b.title, h2.title, sizeof(b.title) - 1);
                        b.title[sizeof(b.title) - 1] = '\0';
                        strncpy(b.author, h2.author, sizeof(b.author) - 1);
                        b.author[sizeof(b.author) - 1] = '\0';
                    }
                } else if (ver == EBK_FORMAT_V1_BITMAP) {
                    EbkHeader h1;
                    if (get_v1_header(b.filename, &h1)) {
                        b.total_frames = h1.total_frames;
                        strncpy(b.title, h1.title, sizeof(b.title) - 1);
                        b.title[sizeof(b.title) - 1] = '\0';
                    }
                }

                if (b.title[0] == '\0') {
                    const char* slash = strrchr(b.filename, '/');
                    const char* clean_name = slash ? slash + 1 : b.filename;
                    strncpy(b.title, clean_name, sizeof(b.title) - 1);
                    b.title[sizeof(b.title) - 1] = '\0';
                }
                count++;
            }
            file = root.openNextFile();
        }
        return count;
    }

    static void list_books() {
        std::unique_ptr<StoredBookInfo[]> books(new (std::nothrow) StoredBookInfo[MAX_STORED_BOOKS]);
        if (!books) return;
        int count = get_stored_books(books.get(), MAX_STORED_BOOKS);

        Serial.println("[FS] --- Stored Books ---");
        if (count == 0) {
            Serial.println("  (No .ebk books found on storage)");
        } else {
            for (int i = 0; i < count; i++) {
                const char* ver_str = (books[i].format_version == EBK_FORMAT_V2_TEXT) ? "v2 Text" : "v1 Bitmap";
                Serial.printf("  * %s [%s] - \"%s\" (%u %s, %u bytes)\n",
                              books[i].filename, ver_str, books[i].title,
                              books[i].total_frames,
                              (books[i].format_version == EBK_FORMAT_V2_TEXT) ? "chapters" : "pages",
                              (unsigned int)books[i].file_size);
            }
        }
        Serial.println("[FS] ---------------------");
    }
};
