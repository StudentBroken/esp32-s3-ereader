#pragma once
#include <stdarg.h>

/**
 * Colour screensaver: fetches pre-converted tri-colour (white/black/red) portrait images from a
 * Google Apps Script web app, caches them on LittleFS and shows one on the e-paper.
 *
 * ---------------------------------------------------------------------------
 * SSI v1 image format (what the Google Sheet stores, chunked, as base64 text)
 * ---------------------------------------------------------------------------
 * Logical image = 32-byte header + 30,000-byte payload.
 *   Payload : 300x400 px portrait, 2 bits per pixel, row-major, 4 px per byte, first pixel in the
 *             two MSBs.  00 = white, 01 = black, 10 = red, 11 = reserved (drawn as white).
 *   Header  : magic "SSI1" | u8 version=1 | u8 flags | u16 width | u16 height | u8 bpp=2 |
 *             u8 palette=0 (W,B,R) | u16 chunk_size=3000 | u16 data_chunks=10 | u32 payload_len |
 *             u32 payload_crc32 | u32 image_id | u32 header_crc32 (over the first 28 bytes).
 *
 * Transport redundancy (one sheet row per chunk: image_id, chunk_idx, crc32, base64):
 *   idx 0        header (32 B)
 *   idx 1..10    data chunks (3000 B each)
 *   idx 11       parity chunk = XOR of the 10 data chunks (rebuilds any ONE lost/corrupt chunk)
 *   idx 12       copy of the header
 *   Every chunk has its own CRC32 (over the raw bytes), the payload has a CRC32 in the header and
 *   the header has its own CRC32.  Anything still missing is re-requested (`c=` parameter).
 *
 * Wire protocol (text/plain from the Apps Script, see tools/screensaver/apps_script.gs):
 *   ?a=list&n=14&k=KEY            ->  "SS1\nL 5,9,12,...\nEND"
 *   ?a=img&id=5&k=KEY[&c=3,7]     ->  "SS1\nC <id> <idx> <crc32hex> <base64>\n...\nEND"
 */

#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <mbedtls/base64.h>
#include <vector>
#include "GxEPD2_420_Z96.h"

#define SS_WIDTH          300
#define SS_HEIGHT         400
#define SS_PAYLOAD_BYTES  30000
#define SS_CHUNK_BYTES    3000
#define SS_DATA_CHUNKS    10
#define SS_HEADER_BYTES   32
#define SS_IDX_HEADER     0
#define SS_IDX_PARITY     11
#define SS_IDX_HEADER2    12
#define SS_CHUNK_SLOTS    13
#define SS_FILE_BYTES     (SS_HEADER_BYTES + SS_PAYLOAD_BYTES)

#define SS_DIR            "/ss"
#define SS_MAX_WIFI       4
#define SS_BATTERY_MIN_PCT 25          // screensaver only runs above this
#define SS_IMAGES_PER_SYNC 14
#define SS_SYNC_EVERY_TICKS 14         // one tick = one 12 h wake -> weekly
#define SS_MAX_SYNC_ATTEMPTS 4
#define SS_SYNC_BUDGET_MS (3UL * 60UL * 1000UL)
#define SS_WAKE_INTERVAL_US (12ULL * 3600ULL * 1000000ULL)

// Photo frame mode: wake every 10 min to show the next image, sync every 3 h (18 wakes)
#define SS_PHOTO_INTERVAL_US (10ULL * 60ULL * 1000000ULL)
#define SS_PHOTO_SYNC_TICKS  18

// Red plane polarity on the controller: 0 = red (same convention as the black plane, 0 = black).
#define SS_RED_ACTIVE_LOW 0

#pragma pack(push, 1)
struct SsiHeader {
    char     magic[4];
    uint8_t  version;
    uint8_t  flags;
    uint16_t width;
    uint16_t height;
    uint8_t  bpp;
    uint8_t  palette;
    uint16_t chunk_size;
    uint16_t data_chunks;
    uint32_t payload_len;
    uint32_t payload_crc;
    uint32_t image_id;
    uint32_t header_crc;
};
#pragma pack(pop)
static_assert(sizeof(SsiHeader) == SS_HEADER_BYTES, "SSI header must be 32 bytes");

// -----------------------------------------------------------------------------
// CRC32 (IEEE 802.3, same as zlib / the web converter)
// -----------------------------------------------------------------------------
// Log line to Serial and, when set, to an on-screen progress hook (see main.cpp)
static void (*ss_log_hook)(const char*) = nullptr;
static inline void ss_log(const char* fmt, ...) {
    char buf[160];
    va_list ap; va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    Serial.print(buf);
    if (ss_log_hook) ss_log_hook(buf);
}

static inline uint32_t ss_crc32(const uint8_t* data, size_t len, uint32_t crc = 0) {
    crc = ~crc;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) crc = (crc >> 1) ^ (0xEDB88320u & (-(int32_t)(crc & 1)));
    }
    return ~crc;
}

// -----------------------------------------------------------------------------
// Persistent settings (NVS namespace "ss")
// -----------------------------------------------------------------------------
class ScreensaverConfig {
public:
    static bool enabled() {
        Preferences p; p.begin("ss", true);
        bool v = p.getUChar("on", 1) != 0;
        p.end();
        return v;
    }
    static void set_enabled(bool on) {
        Preferences p; p.begin("ss", false);
        p.putUChar("on", on ? 1 : 0);
        p.end();
    }
    static String url() {
        Preferences p; p.begin("ss", true);
        String u = p.getString("url", "");
        p.end();
        return u;
    }
    static void set_url(const String& u) {
        Preferences p; p.begin("ss", false);
        p.putString("url", u);
        // New config: sync at the next opportunity
        p.putUChar("pend", 1);
        p.putUChar("tries", 0);
        p.end();
    }
    static bool get_wifi(int slot, String& ssid, String& pass) {
        if (slot < 0 || slot >= SS_MAX_WIFI) return false;
        char ks[8], kp[8];
        snprintf(ks, sizeof(ks), "ws%d", slot);
        snprintf(kp, sizeof(kp), "wp%d", slot);
        Preferences p; p.begin("ss", true);
        ssid = p.getString(ks, "");
        pass = p.getString(kp, "");
        p.end();
        return ssid.length() > 0;
    }
    static void set_wifi(int slot, const String& ssid, const String& pass) {
        if (slot < 0 || slot >= SS_MAX_WIFI) return;
        char ks[8], kp[8];
        snprintf(ks, sizeof(ks), "ws%d", slot);
        snprintf(kp, sizeof(kp), "wp%d", slot);
        Preferences p; p.begin("ss", false);
        if (ssid.length() == 0) { p.remove(ks); p.remove(kp); }
        else { p.putString(ks, ssid); p.putString(kp, pass); }
        p.putUChar("pend", 1);
        p.putUChar("tries", 0);
        p.end();
    }
    static bool has_wifi() {
        String s, pw;
        for (int i = 0; i < SS_MAX_WIFI; i++) if (get_wifi(i, s, pw)) return true;
        return false;
    }
    /** Sync possible at all: toggle on, URL and at least one network stored. */
    static bool ready() { return enabled() && url().length() > 8 && has_wifi(); }

    // Photo frame mode (continuous slideshow): flag, 10 min wake counter, sync retry state
    static bool photo_mode() { Preferences p; p.begin("ss", true); bool v = p.getUChar("photo", 0); p.end(); return v; }
    static void set_photo_mode(bool on) {
        Preferences p; p.begin("ss", false);
        p.putUChar("photo", on ? 1 : 0);
        p.putUInt("ptick", 0);
        p.putUChar("ppend", 0);
        p.putUChar("ptries", 0);
        p.end();
    }
    static uint32_t photo_tick() { Preferences p; p.begin("ss", true); uint32_t v = p.getUInt("ptick", 0); p.end(); return v; }
    static void set_photo_tick(uint32_t v) { Preferences p; p.begin("ss", false); p.putUInt("ptick", v); p.end(); }
    static bool photo_pending() { Preferences p; p.begin("ss", true); bool v = p.getUChar("ppend", 0); p.end(); return v; }
    static uint8_t photo_tries() { Preferences p; p.begin("ss", true); uint8_t v = p.getUChar("ptries", 0); p.end(); return v; }
    static void set_photo_sync_state(bool pend, uint8_t tries) {
        Preferences p; p.begin("ss", false);
        p.putUChar("ppend", pend ? 1 : 0);
        p.putUChar("ptries", tries);
        p.end();
    }

    // Picture frame: 0 = shuffle, 1 = fixed loop in sheet order; server-ordered playlist; last id shown
    static bool frame_seq() { Preferences p; p.begin("ss", true); bool v = p.getUChar("fseq", 0); p.end(); return v; }
    static void set_frame_seq(bool on) { Preferences p; p.begin("ss", false); p.putUChar("fseq", on ? 1 : 0); p.end(); }
    static String frame_list() { Preferences p; p.begin("ss", true); String v = p.getString("flist", ""); p.end(); return v; }
    static void set_frame_list(const String& v) { Preferences p; p.begin("ss", false); p.putString("flist", v); p.end(); }
    static uint32_t frame_last() { Preferences p; p.begin("ss", true); uint32_t v = p.getUInt("flast", 0); p.end(); return v; }
    static void set_frame_last(uint32_t v) { Preferences p; p.begin("ss", false); p.putUInt("flast", v); p.end(); }

    // 12 h wake counter, sync retry state and rotation index
    static uint32_t tick() { Preferences p; p.begin("ss", true); uint32_t v = p.getUInt("tick", 0); p.end(); return v; }
    static void set_tick(uint32_t v) { Preferences p; p.begin("ss", false); p.putUInt("tick", v); p.end(); }
    static bool pending() { Preferences p; p.begin("ss", true); bool v = p.getUChar("pend", 0); p.end(); return v; }
    static uint8_t tries() { Preferences p; p.begin("ss", true); uint8_t v = p.getUChar("tries", 0); p.end(); return v; }
    static void set_sync_state(bool pend, uint8_t tries) {
        Preferences p; p.begin("ss", false);
        p.putUChar("pend", pend ? 1 : 0);
        p.putUChar("tries", tries);
        p.end();
    }
    static uint32_t shown() { Preferences p; p.begin("ss", true); uint32_t v = p.getUInt("shown", 0); p.end(); return v; }
    static void set_shown(uint32_t v) { Preferences p; p.begin("ss", false); p.putUInt("shown", v); p.end(); }
};

// -----------------------------------------------------------------------------
// Image cache on LittleFS: /ss/<id>.ssi  (32 B header + 30,000 B payload)
// -----------------------------------------------------------------------------
static inline void ss_image_path(uint32_t id, char* out, size_t n) {
    snprintf(out, n, SS_DIR "/%lu.ssi", (unsigned long)id);
}

static inline std::vector<uint32_t> ss_cached_ids() {
    std::vector<uint32_t> ids;
    File dir = LittleFS.open(SS_DIR);
    if (!dir || !dir.isDirectory()) return ids;
    File f = dir.openNextFile();
    while (f) {
        String name = f.name();
        int slash = name.lastIndexOf('/');
        if (slash >= 0) name = name.substring(slash + 1);
        if (name.endsWith(".ssi")) ids.push_back((uint32_t)strtoul(name.c_str(), nullptr, 10));
        f = dir.openNextFile();
    }
    // stable order so the rotation index is meaningful
    for (size_t i = 1; i < ids.size(); i++)
        for (size_t j = i; j > 0 && ids[j - 1] > ids[j]; j--) { uint32_t t = ids[j]; ids[j] = ids[j - 1]; ids[j - 1] = t; }
    return ids;
}

static inline bool ss_header_valid(const SsiHeader& h) {
    if (memcmp(h.magic, "SSI1", 4) != 0 || h.version != 1) return false;
    if (h.width != SS_WIDTH || h.height != SS_HEIGHT || h.bpp != 2) return false;
    if (h.chunk_size != SS_CHUNK_BYTES || h.data_chunks != SS_DATA_CHUNKS || h.payload_len != SS_PAYLOAD_BYTES) return false;
    return ss_crc32((const uint8_t*)&h, SS_HEADER_BYTES - 4) == h.header_crc;
}

// -----------------------------------------------------------------------------
// WiFi
// -----------------------------------------------------------------------------
/** Scans, picks the strongest saved network and connects. */
static inline bool ss_wifi_connect(uint32_t timeout_ms = 20000) {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true, false);
    delay(50);

    int found = WiFi.scanNetworks(false, false);
    ss_log("[SS] WiFi scan: %d networks\n", found);
    if (found <= 0) { WiFi.scanDelete(); WiFi.mode(WIFI_OFF); return false; }

    int best_slot = -1, best_rssi = -1000;
    String best_ssid, best_pass;
    for (int slot = 0; slot < SS_MAX_WIFI; slot++) {
        String ssid, pass;
        if (!ScreensaverConfig::get_wifi(slot, ssid, pass)) continue;
        for (int n = 0; n < found; n++) {
            if (WiFi.SSID(n) == ssid && WiFi.RSSI(n) > best_rssi) {
                best_rssi = WiFi.RSSI(n);
                best_slot = slot;
                best_ssid = ssid;
                best_pass = pass;
            }
        }
    }
    WiFi.scanDelete();
    if (best_slot < 0) {
        ss_log("[SS] No saved network in range.\n");
        WiFi.mode(WIFI_OFF);
        return false;
    }

    ss_log("[SS] Joining '%s' (%d dBm)...\n", best_ssid.c_str(), best_rssi);
    WiFi.begin(best_ssid.c_str(), best_pass.c_str());
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeout_ms) delay(100);
    if (WiFi.status() != WL_CONNECTED) {
        ss_log("[SS] WiFi connect timed out.\n");
        WiFi.disconnect(true, false);
        WiFi.mode(WIFI_OFF);
        return false;
    }
    ss_log("[SS] Connected, IP %s\n", WiFi.localIP().toString().c_str());
    return true;
}

static inline void ss_wifi_off() {
    WiFi.disconnect(true, false);
    WiFi.mode(WIFI_OFF);
}

// -----------------------------------------------------------------------------
// HTTP (Apps Script answers with a 302 to googleusercontent.com)
// -----------------------------------------------------------------------------
static inline bool ss_http_get(const String& url, String& body) {
    WiFiClientSecure client;
    client.setInsecure();   // payload integrity is protected by CRC32s; the key only gates the sheet
    HTTPClient http;
    http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
    http.setTimeout(20000);
    if (!http.begin(client, url)) return false;
    int code = http.GET();
    bool ok = (code == 200);
    if (ok) body = http.getString();
    else ss_log("[SS] HTTP %d\n", code);
    http.end();
    return ok;
}

static inline String ss_build_url(const String& base, const String& query) {
    return base + (base.indexOf('?') >= 0 ? "&" : "?") + query;
}

static inline bool ss_b64_decode(const char* in, size_t in_len, uint8_t* out, size_t out_cap, size_t* out_len) {
    return mbedtls_base64_decode(out, out_cap, out_len, (const unsigned char*)in, in_len) == 0;
}

// -----------------------------------------------------------------------------
// One image: download chunks, verify, repair, store
// -----------------------------------------------------------------------------
struct SsAssembly {
    uint8_t  hdr[SS_HEADER_BYTES];
    bool     have_hdr = false;
    uint8_t* payload = nullptr;                    // 30,000 B
    uint8_t* parity = nullptr;                     // 3,000 B
    bool     got[SS_CHUNK_SLOTS] = {false};
};

/** Parses one response body and stores every chunk whose CRC matches. */
static inline void ss_ingest(const String& body, uint32_t id, SsAssembly& a) {
    int pos = 0;
    uint8_t* tmp = (uint8_t*)malloc(SS_CHUNK_BYTES + 8);
    if (!tmp) return;
    while (pos < (int)body.length()) {
        int eol = body.indexOf('\n', pos);
        if (eol < 0) eol = body.length();
        if (body.charAt(pos) == 'C' && body.charAt(pos + 1) == ' ') {
            // C <id> <idx> <crc> <b64>
            char* end;
            const char* line = body.c_str() + pos;
            unsigned long rid = strtoul(line + 2, &end, 10);
            unsigned long idx = strtoul(end, &end, 10);
            unsigned long crc = strtoul(end, &end, 16);
            while (*end == ' ') end++;
            size_t b64_len = (body.c_str() + eol) - end;
            while (b64_len > 0 && (end[b64_len - 1] == '\r' || end[b64_len - 1] == ' ')) b64_len--;
            if (rid == id && idx < SS_CHUNK_SLOTS && !a.got[idx]) {
                size_t n = 0;
                size_t expect = (idx == SS_IDX_HEADER || idx == SS_IDX_HEADER2) ? SS_HEADER_BYTES : SS_CHUNK_BYTES;
                if (ss_b64_decode(end, b64_len, tmp, SS_CHUNK_BYTES + 8, &n) && n == expect &&
                    ss_crc32(tmp, n) == (uint32_t)crc) {
                    if (idx == SS_IDX_HEADER || idx == SS_IDX_HEADER2) {
                        memcpy(a.hdr, tmp, n);
                        a.have_hdr = true;
                    } else if (idx == SS_IDX_PARITY) {
                        memcpy(a.parity, tmp, n);
                    } else {
                        memcpy(a.payload + (idx - 1) * SS_CHUNK_BYTES, tmp, n);
                    }
                    a.got[idx] = true;
                } else {
                    ss_log("[SS] chunk %lu/%lu rejected (bad length/CRC)\n", rid, idx);
                }
            }
        }
        pos = eol + 1;
    }
    free(tmp);
}

/** Rebuilds a single missing data chunk from the parity chunk. */
static inline bool ss_try_parity_repair(SsAssembly& a) {
    int missing = -1;
    for (int i = 1; i <= SS_DATA_CHUNKS; i++) {
        if (!a.got[i]) { if (missing >= 0) return false; missing = i; }
    }
    if (missing < 0) return true;
    if (!a.got[SS_IDX_PARITY]) return false;
    uint8_t* dst = a.payload + (missing - 1) * SS_CHUNK_BYTES;
    memcpy(dst, a.parity, SS_CHUNK_BYTES);
    for (int i = 1; i <= SS_DATA_CHUNKS; i++) {
        if (i == missing) continue;
        const uint8_t* src = a.payload + (i - 1) * SS_CHUNK_BYTES;
        for (int b = 0; b < SS_CHUNK_BYTES; b++) dst[b] ^= src[b];
    }
    a.got[missing] = true;
    ss_log("[SS] chunk %d rebuilt from parity\n", missing);
    return true;
}

static inline bool ss_fetch_image(const String& base_url, uint32_t id) {
    SsAssembly a;
    a.payload = (uint8_t*)malloc(SS_PAYLOAD_BYTES);
    a.parity = (uint8_t*)malloc(SS_CHUNK_BYTES);
    if (!a.payload || !a.parity) { free(a.payload); free(a.parity); return false; }

    bool ok = false;
    for (int attempt = 0; attempt < 3 && !ok; attempt++) {
        String query = "a=img&id=" + String((unsigned long)id);
        if (attempt > 0) {
            String list;
            for (int i = 0; i < SS_CHUNK_SLOTS; i++) {
                bool needed = !a.got[i];
                if (i == SS_IDX_HEADER2 && a.have_hdr) needed = false;
                if (i == SS_IDX_HEADER && a.have_hdr) needed = false;
                if (needed) { if (list.length()) list += ','; list += String(i); }
            }
            if (list.length() == 0) break;
            query += "&c=" + list;
        }
        String body;
        if (!ss_http_get(ss_build_url(base_url, query), body)) continue;
        ss_ingest(body, id, a);

        bool all = a.have_hdr;
        for (int i = 1; i <= SS_DATA_CHUNKS; i++) if (!a.got[i]) all = false;
        if (!all && a.have_hdr) all = ss_try_parity_repair(a) && a.have_hdr;
        if (!all) continue;

        SsiHeader h;
        memcpy(&h, a.hdr, sizeof(h));
        if (!ss_header_valid(h) || h.image_id != id) { ss_log("[SS] header invalid\n"); a.have_hdr = false; a.got[SS_IDX_HEADER] = a.got[SS_IDX_HEADER2] = false; continue; }
        if (ss_crc32(a.payload, SS_PAYLOAD_BYTES) != h.payload_crc) {
            ss_log("[SS] payload CRC mismatch, refetching everything\n");
            memset(a.got, 0, sizeof(a.got));
            a.have_hdr = false;
            continue;
        }

        char path[32];
        ss_image_path(id, path, sizeof(path));
        File f = LittleFS.open(path, "w");
        if (f) {
            size_t w = f.write(a.hdr, SS_HEADER_BYTES) + f.write(a.payload, SS_PAYLOAD_BYTES);
            f.close();
            ok = (w == SS_FILE_BYTES);
            if (!ok) LittleFS.remove(path);
        }
    }
    free(a.payload);
    free(a.parity);
    ss_log("[SS] image %lu %s\n", (unsigned long)id, ok ? "stored" : "FAILED");
    return ok;
}

/**
 * Full sync: asks for SS_IMAGES_PER_SYNC random ids, downloads them and replaces the old cache.
 * WiFi must already be connected. Returns true only if every requested image was stored.
 */
static inline bool ss_sync(uint32_t budget_ms = SS_SYNC_BUDGET_MS, bool seq = false, uint32_t seq_after = 0) {
    String base = ScreensaverConfig::url();
    uint32_t t0 = millis();
    LittleFS.mkdir(SS_DIR);

    String body;
    // Tell the server what we already hold so it can prefer pictures we have not seen yet
    String have;
    for (uint32_t cid : ss_cached_ids()) { if (have.length()) have += ','; have += String((unsigned long)cid); }
    String list_query = "a=list&n=" + String(SS_IMAGES_PER_SYNC);
    if (seq) list_query += "&mode=seq&after=" + String((unsigned long)seq_after);   // next images in sheet order
    else if (have.length()) list_query += "&have=" + have;
    if (!ss_http_get(ss_build_url(base, list_query), body)) return false;
    int l = body.indexOf("\nL ");
    if (body.indexOf("SS1") < 0 || l < 0) { ss_log("[SS] bad list response\n"); return false; }
    int e = body.indexOf('\n', l + 3);
    String csv = body.substring(l + 3, e < 0 ? body.length() : e);
    csv.trim();

    std::vector<uint32_t> ids;
    int start = 0;
    while (start < (int)csv.length()) {
        int comma = csv.indexOf(',', start);
        if (comma < 0) comma = csv.length();
        uint32_t id = (uint32_t)strtoul(csv.substring(start, comma).c_str(), nullptr, 10);
        if (id > 0) ids.push_back(id);
        start = comma + 1;
    }
    if (ids.empty()) return false;
    ss_log("[SS] Server picked %u images\n", (unsigned)ids.size());

    // Make room first: old images that are not part of the new set are dropped up front if space is tight.
    size_t needed = ids.size() * SS_FILE_BYTES + 64 * 1024;
    if (LittleFS.totalBytes() - LittleFS.usedBytes() < needed) {
        for (uint32_t old : ss_cached_ids()) {
            char path[32]; ss_image_path(old, path, sizeof(path));
            LittleFS.remove(path);
        }
    }

    std::vector<uint32_t> stored;
    for (uint32_t id : ids) {
        if (millis() - t0 > budget_ms) { ss_log("[SS] sync budget exhausted\n"); break; }
        char path[32]; ss_image_path(id, path, sizeof(path));
        bool have = false;
        if (LittleFS.exists(path)) {            // image already cached and previously verified
            File f = LittleFS.open(path, "r");
            have = f && f.size() == SS_FILE_BYTES;
            if (f) f.close();
        }
        if (have || ss_fetch_image(base, id)) stored.push_back(id);
    }

    bool complete = (stored.size() == ids.size());
    if (complete) {
        for (uint32_t old : ss_cached_ids()) {
            bool keep = false;
            for (uint32_t id : stored) if (id == old) keep = true;
            if (!keep) { char path[32]; ss_image_path(old, path, sizeof(path)); LittleFS.remove(path); }
        }
        ScreensaverConfig::set_shown(0);
        if (seq) ScreensaverConfig::set_frame_list(csv);   // remember the loop order for the picture frame
    }
    return complete;
}

// -----------------------------------------------------------------------------
// Rendering
// -----------------------------------------------------------------------------
/** Loads a cached image and pushes it to the panel with a full tri-colour refresh. */
static inline bool ss_show_image(GxEPD2_420_Z96& epd, uint32_t id) {
    char path[32];
    ss_image_path(id, path, sizeof(path));
    File f = LittleFS.open(path, "r");
    if (!f || f.size() != SS_FILE_BYTES) { if (f) f.close(); return false; }

    SsiHeader h;
    f.read((uint8_t*)&h, sizeof(h));
    uint8_t* payload = (uint8_t*)malloc(SS_PAYLOAD_BYTES);
    uint8_t* bw = (uint8_t*)malloc(15000);
    uint8_t* red = (uint8_t*)malloc(15000);
    bool ok = payload && bw && red && ss_header_valid(h) &&
              f.read(payload, SS_PAYLOAD_BYTES) == SS_PAYLOAD_BYTES &&
              ss_crc32(payload, SS_PAYLOAD_BYTES) == h.payload_crc;
    f.close();

    if (ok) {
        memset(bw, 0xFF, 15000);
        memset(red, SS_RED_ACTIVE_LOW ? 0xFF : 0x00, 15000);
        // Portrait (300x400) -> native panel (400x300), identical to GFX rotation 1:
        // native_x = 399 - py, native_y = px.
        for (int py = 0; py < SS_HEIGHT; py++) {
            int nx = 399 - py;
            uint8_t mask = 0x80 >> (nx & 7);
            int col_byte = nx >> 3;
            for (int px = 0; px < SS_WIDTH; px++) {
                uint8_t v = (payload[(py * SS_WIDTH + px) >> 2] >> (6 - 2 * (px & 3))) & 3;
                if (v == 0 || v == 3) continue;
                int idx = px * 50 + col_byte;
                if (v == 1) bw[idx] &= ~mask;
                else if (SS_RED_ACTIVE_LOW) red[idx] &= ~mask;
                else red[idx] |= mask;
            }
        }
        ss_log("[SS] Showing image %lu (full tri-colour refresh)...\n", (unsigned long)id);
        epd.drawTriColor(bw, red);
    } else {
        ss_log("[SS] image %lu unreadable/corrupt, removing\n", (unsigned long)id);
        LittleFS.remove(path);
    }
    free(payload); free(bw); free(red);
    return ok;
}

/** Shows the next cached image in rotation. Returns false if nothing could be shown. */
static inline bool ss_show_next(GxEPD2_420_Z96& epd) {
    for (int tries = 0; tries < 3; tries++) {
        std::vector<uint32_t> ids = ss_cached_ids();
        if (ids.empty()) return false;
        uint32_t shown = ScreensaverConfig::shown();
        uint32_t id = ids[shown % ids.size()];
        ScreensaverConfig::set_shown(shown + 1);
        if (ss_show_image(epd, id)) return true;
    }
    return false;
}
