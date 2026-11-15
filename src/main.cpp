// =====================================================================
//  Cardputer-Adv  -  Off-grid Offline Map Viewer  (v2.0)
// ---------------------------------------------------------------------
//  * Offline PNG tile map from MicroSD, GPS marker, keyboard control
//  * Flicker-free: the whole frame is composed on an off-screen canvas
//    and pushed once (fixes crosshair/status-bar blinking)
//  * Smooth, eased view sliding (no more jumping)
//  * Local time (GPS + time-zone), altitude, GPS course direction
//  * Live crosshair (view-centre) latitude/longitude
//  * GPX track recording (key R) + on-device track viewer (key L)
//  * On-device settings screen for time zone (key S)
//
//  Modules: Gps | Slippy | Settings | Net | TrackRec | TrackView |
//           MapView | WiFiPortal | SettingsUi | TrackListUi | Ui | Input
// =====================================================================

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <M5Cardputer.h>
#include <math.h>
#include <time.h>
#include <vector>
#include <algorithm>
#include <utility>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

#include "config.h"

using KeyState = Keyboard_Class::KeysState;
using Gfx = lgfx::LGFXBase;   // common base of M5GFX (display) and M5Canvas (sprite)

// =====================================================================
//  THEME  (Fallout / Pip-Boy green terminal look, shared by all menus)
// =====================================================================
namespace Theme {
    template <class G> uint16_t fg(G &g)  { return g.color565(130, 255, 150); } // bright green
    template <class G> uint16_t dim(G &g) { return g.color565(0, 120, 55); }    // dim green
    template <class G> uint16_t hl(G &g)  { return g.color565(0, 60, 28); }     // row highlight
    template <class G> uint16_t bar(G &g) { return g.color565(0, 85, 38); }     // header bar
    template <class G> void scan(G &g) {
        for (int y = 0; y < SCREEN_H; y += 3) g.drawFastHLine(0, y, SCREEN_W, g.color565(0, 26, 12));
    }
    template <class G> void screen(G &g) { g.fillScreen(TFT_BLACK); scan(g); }
    template <class G> void header(G &g, const char *t) {
        screen(g);
        g.fillRect(0, 0, SCREEN_W, 16, bar(g));
        g.setTextColor(TFT_BLACK, bar(g)); g.setTextSize(1); g.setCursor(4, 4); g.print(t);
    }
    template <class G> void footer(G &g, const char *h) {
        g.setTextColor(dim(g), TFT_BLACK); g.setTextSize(1); g.setCursor(4, SCREEN_H - 9); g.print(h);
    }
}

// =====================================================================
//  NOTIFY  (toast + progress-bar overlay, shared by MapView's own tile
//  downloads and BulkDownloadUi's range download -- pulled out to free
//  functions so both can call it without duplicating the drawing code)
// =====================================================================
namespace Notify {
    inline void notice(Gfx &d, const char *msg, int y = 54) {
        int w = (int)strlen(msg)*6 + 18, x = (SCREEN_W-w)/2;
        d.fillRoundRect(x, y, w, 24, 4, TFT_BLACK);
        d.drawRoundRect(x, y, w, 24, 4, Theme::fg(d)); d.drawRoundRect(x+1, y+1, w-2, 22, 4, Theme::dim(d));
        d.setTextColor(Theme::fg(d), TFT_BLACK); d.setTextSize(1); d.setCursor(x+9, y+8); d.print(msg);
        delay(1300);
    }
    inline void progress(Gfx &d, const char *msg, int done, int total) {
        int w = 204, x = (SCREEN_W-w)/2, y = 44, h = 36;
        d.fillRoundRect(x, y, w, h, 4, TFT_BLACK);
        d.drawRoundRect(x, y, w, h, 4, Theme::fg(d)); d.drawRoundRect(x+1, y+1, w-2, h-2, 4, Theme::dim(d));
        d.setTextColor(Theme::fg(d), TFT_BLACK); d.setTextSize(1); d.setCursor(x+8, y+7); d.print(msg);
        int bw = w-16, filled = total ? (bw*done)/total : 0;
        d.drawRect(x+8, y+22, bw, 9, Theme::dim(d)); d.fillRect(x+8, y+22, filled, 9, Theme::fg(d));
    }
}

// Help key list (scrollable, big font).
// Kept under 18 chars each (checked against size-2 font: 18*12=216px, fits
// comfortably inside the 240px screen with the row's left margin).
static const char *HELP_LINES[] = {
    "; . , /  Pan",
    "g  Center GPS",
    "+/-  Zoom",
    "m  Basemap",
    "n  Tag note",
    "1  Icons on/off",
    "2  Notes list",
    "   f find  i icon",
    "   o sort  x clear",
    "3  Distance calc",
    "r  Rec track",
    "p  Pause/resume",
    "4  Tracks",
    "d  Download",
    "D  Refresh all",
    "c  Get center tile",
    "5  Bulk download",
    "w  WiFi",
    "s  Settings",
    "9  Screen off",
    "esc  Back/cancel",
    "h  Close help",
};
static const int HELP_COUNT = (int)(sizeof(HELP_LINES) / sizeof(HELP_LINES[0]));
static const int HELP_ROWS  = 5;

// =====================================================================
//  MODULE 1 - GPS
// =====================================================================
class Gps {
public:
    // source = GPS_SRC_CAP (14P header, fixed 115200) or GPS_SRC_GROVE
    // (Grove port, caller-supplied baud). Both share HardwareSerial #1 --
    // only one receiver can be active at a time, which is fine because only
    // one is ever physically attached.
    void begin(int source = GPS_SRC_DEFAULT, int groveBaud = GPS_GROVE_BAUD_DEFAULT) {
        if (GPS_DEBUG) Serial.begin(115200);
        openPort(source, groveBaud);
    }
    // Re-open the UART on the other receiver / a different baud. Safe to call
    // at any time: end() releases the pins before begin() claims the new ones,
    // and the TinyGPSPlus parser is reset so half a sentence from the old port
    // can't merge with the first bytes off the new one.
    void reconfigure(int source, int groveBaud) {
        if (source == _source && groveBaud == _groveBaud) return;
        _serial.end();
        delay(20);
        _tiny = TinyGPSPlus();
        _freshFix = false;
        openPort(source, groveBaud);
    }
    int  source()    const { return _source; }
    int  baud()      const { return _baud; }
    void update() {
        while (_serial.available() > 0) {
            char c = _serial.read();
            if (GPS_DEBUG) Serial.write(c);
            if (_tiny.encode(c) && _tiny.location.isUpdated() && _tiny.location.isValid()) _freshFix = true;
        }
    }
    bool     hasFix()       { return _tiny.location.isValid() && _tiny.satellites.value() > 0; }
    bool     consumeFresh() { bool f = _freshFix; _freshFix = false; return f; }
    double   lat()          { return _tiny.location.lat(); }
    double   lon()          { return _tiny.location.lng(); }
    uint32_t sats()         { return _tiny.satellites.isValid() ? _tiny.satellites.value() : 0; }
    uint32_t charsRx()      { return _tiny.charsProcessed(); }
    double   altitude()     { return _tiny.altitude.isValid() ? _tiny.altitude.meters() : 0.0; }
    bool     courseValid()  { return _tiny.course.isValid() && _tiny.speed.isValid(); }
    double   courseDeg()    { return _tiny.course.deg(); }
    double   speedKmph()    { return _tiny.speed.isValid() ? _tiny.speed.kmph() : 0.0; }
    const char* cardinal()  { return TinyGPSPlus::cardinal(_tiny.course.deg()); }
    bool     timeValid()    { return _tiny.time.isValid(); }
    int      hh()           { return _tiny.time.hour(); }
    int      mm()           { return _tiny.time.minute(); }
    int      ss()           { return _tiny.time.second(); }
    bool     dateValid()    { return _tiny.date.isValid() && _tiny.date.year() > 2000; }
    int      yy()           { return _tiny.date.year(); }
    int      mo()           { return _tiny.date.month(); }
    int      dd()           { return _tiny.date.day(); }
private:
    void openPort(int source, int groveBaud) {
        _source    = (source == GPS_SRC_GROVE) ? GPS_SRC_GROVE : GPS_SRC_CAP;
        _groveBaud = groveBaud > 0 ? groveBaud : GPS_GROVE_BAUD_DEFAULT;
        if (_source == GPS_SRC_GROVE) {
            _baud = _groveBaud;
            _serial.begin(_baud, SERIAL_8N1, GPS_GROVE_RX_PIN, GPS_GROVE_TX_PIN);
        } else {
            _baud = GPS_BAUD;
            _serial.begin(_baud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
        }
        if (GPS_DEBUG)
            Serial.printf("\n[gps] source=%s rx=%d tx=%d baud=%d\n",
                          _source == GPS_SRC_GROVE ? "GROVE" : "CAP",
                          _source == GPS_SRC_GROVE ? GPS_GROVE_RX_PIN : GPS_RX_PIN,
                          _source == GPS_SRC_GROVE ? GPS_GROVE_TX_PIN : GPS_TX_PIN, _baud);
    }
    HardwareSerial _serial{GPS_UART};
    TinyGPSPlus    _tiny;
    bool           _freshFix = false;
    int            _source    = GPS_SRC_DEFAULT;
    int            _baud      = GPS_BAUD;
    int            _groveBaud = GPS_GROVE_BAUD_DEFAULT;
};

// =====================================================================
//  MODULE 2 - SLIPPY MAP MATHS  (forward + inverse)
// =====================================================================
namespace Slippy {
    inline double lonToWorldPx(double lon, int z) {
        double n = (double)(1UL << z);
        return (lon + 180.0) / 360.0 * n * TILE_SIZE;
    }
    inline double latToWorldPx(double lat, int z) {
        double r = lat * M_PI / 180.0, n = (double)(1UL << z);
        double y = (1.0 - log(tan(r) + 1.0 / cos(r)) / M_PI) / 2.0;
        return y * n * TILE_SIZE;
    }
    inline double worldPxToLon(double px, int z) {
        double n = (double)(1UL << z);
        return px / (n * TILE_SIZE) * 360.0 - 180.0;
    }
    inline double worldPxToLat(double py, int z) {
        double n = (double)(1UL << z);
        double y = py / (n * TILE_SIZE);
        return atan(sinh(M_PI * (1.0 - 2.0 * y))) * 180.0 / M_PI;
    }
}

// =====================================================================
//  BASEMAPS  (switchable tile styles; each kept in its own SD folder)
//  Tiles: <root>/{z}/{x}/{y}.png (256px PNG). 'maxNative' is the provider's
//  highest rendered zoom; beyond it the app OVER-ZOOMS by scaling native
//  tiles, so you can keep zooming into e.g. OpenTopoMap past level 17.
// =====================================================================
struct BaseMap { const char *name; const char *tag; const char *root; const char *url; int maxNative; };
static const BaseMap BASEMAPS[] = {
    { "OSM Standard", "OSM",  TILE_ROOT,    TILE_URL_TEMPLATE,                                 19 },
    { "OpenTopoMap",  "TOPO", TOPO_ROOT,    "https://tile.opentopomap.org/%d/%d/%d.png",       17 },
    { "CyclOSM",      "CYCL", CYCLOSM_ROOT, "https://a.tile-cyclosm.openstreetmap.fr/cyclosm/%d/%d/%d.png", 18 },
};
static const int BASEMAP_COUNT = (int)(sizeof(BASEMAPS) / sizeof(BASEMAPS[0]));

// ---------------------------------------------------------------------
//  Draw a PNG tile from SD onto any LovyanGFX target (display OR canvas).
//  Streams straight off the open SD file handle -- pngle pulls bytes as it
//  decompresses, so this needs no single contiguous RAM buffer at all, no
//  matter how big a given tile's compressed PNG is. (A fixed-size decode
//  buffer used to live here; it kept losing to specific tiles -- e.g. a
//  CyclOSM tile can be 2x+ the size of the same area on plain OSM, since
//  its more colorful style compresses worse -- whenever that one tile's
//  file crossed the buffer size, on every single render.)
// ---------------------------------------------------------------------
template <class GFX>
static bool drawPngFromSd(GFX &g, const char *path, int x, int y, float scale = 1.0f) {
    File f = SD.open(path, FILE_READ);
    if (!f) { Serial.printf("[decode] %s: SD open failed\n", path); return false; }
    size_t sz = f.size();
    if (sz == 0 || sz > 262144) { Serial.printf("[decode] %s: bad size %u\n", path, (unsigned)sz); f.close(); return false; }
    // drawPngFile(fs::FS&, ...) hits a broken constructor call in this M5GFX
    // version's macro-generated wrapper (tries to bind an FS object where an
    // FS* is required), so build the file-backed DataWrapper directly instead
    // -- same streaming behavior, just skipping the broken overload.
    lgfx::DataWrapperT<fs::File> data(&f);
    bool pngOk = g.drawPng(&data, x, y, 0, 0, 0, 0, scale, scale);
    f.close();
    Serial.printf("[decode] %s: sz=%u streamed -> %s\n", path, (unsigned)sz, pngOk ? "OK" : "FAILED");
    return pngOk;
}

// =====================================================================
//  MODULE 3 - SETTINGS  (persisted to SD)
// =====================================================================
struct Settings {
    float  tzHours = TZ_DEFAULT_HOURS;
    int    baseDefault = 0;   // basemap index loaded on boot
    int    brightness = 180;  // 16..255 LCD backlight
    float  trackMinDistM = (float)TRACK_MIN_DIST_M;  // log a track point after this many metres
    bool   hasDefaultPos = false;                    // user has captured a boot position
    double defLat = 0, defLon = 0;                   // used when no GPS fix yet; else falls back to DEFAULT_LAT/LON
    bool   showWaypointsOnBoot = true;                // initial waypoint-icon visibility ('1' still toggles live)
    int    gpsSource = GPS_SRC_DEFAULT;               // GPS_SRC_CAP or GPS_SRC_GROVE
    int    gpsGroveBaud = GPS_GROVE_BAUD_DEFAULT;     // baud used when gpsSource == GPS_SRC_GROVE
    void load() {
        File f = SD.open(SETTINGS_PATH, FILE_READ);
        if (!f) return;
        String l = f.readStringUntil('\n'); l.trim();
        if (l.length()) tzHours = l.toFloat();
        String b = f.readStringUntil('\n'); b.trim();
        if (b.length()) baseDefault = b.toInt();
        String br = f.readStringUntil('\n'); br.trim();
        if (br.length()) brightness = br.toInt();
        if (brightness < 16) brightness = 16;
        // Fields below are appended -- older settings.txt files simply won't
        // have these lines, and readStringUntil() returns empty at EOF, so
        // the length()-guarded defaults above already handle that cleanly.
        String tm = f.readStringUntil('\n'); tm.trim();
        if (tm.length()) trackMinDistM = tm.toFloat();
        if (trackMinDistM < 3.0f) trackMinDistM = 3.0f;
        if (trackMinDistM > (float)TRACK_MIN_DIST_MAX) trackMinDistM = (float)TRACK_MIN_DIST_MAX;
        String hp = f.readStringUntil('\n'); hp.trim();
        if (hp.length()) hasDefaultPos = (hp.toInt() != 0);
        String dla = f.readStringUntil('\n'); dla.trim();
        if (dla.length()) defLat = dla.toFloat();
        String dlo = f.readStringUntil('\n'); dlo.trim();
        if (dlo.length()) defLon = dlo.toFloat();
        String wp = f.readStringUntil('\n'); wp.trim();
        if (wp.length()) showWaypointsOnBoot = (wp.toInt() != 0);
        String gs = f.readStringUntil('\n'); gs.trim();
        if (gs.length()) gpsSource = gs.toInt();
        if (gpsSource < 0 || gpsSource >= GPS_SRC_COUNT) gpsSource = GPS_SRC_DEFAULT;
        String gb = f.readStringUntil('\n'); gb.trim();
        if (gb.length()) gpsGroveBaud = gb.toInt();
        if (gpsBaudIndex() < 0) gpsGroveBaud = GPS_GROVE_BAUD_DEFAULT;   // reject junk / unsupported rates
        f.close();
    }
    // Index of gpsGroveBaud within GPS_BAUD_CHOICES, or -1 if it isn't one of them.
    int gpsBaudIndex() const {
        for (int i = 0; i < GPS_BAUD_CHOICE_COUNT; ++i)
            if (GPS_BAUD_CHOICES[i] == gpsGroveBaud) return i;
        return -1;
    }
    void save() {
        File f = SD.open(SETTINGS_PATH, FILE_WRITE);
        if (!f) return;
        f.println(tzHours, 2);
        f.println(baseDefault);
        f.println(brightness);
        f.println(trackMinDistM, 1);
        f.println(hasDefaultPos ? 1 : 0);
        f.println(defLat, 6);
        f.println(defLon, 6);
        f.println(showWaypointsOnBoot ? 1 : 0);
        f.println(gpsSource);
        f.println(gpsGroveBaud);
        f.close();
    }
};

// =====================================================================
//  MODULE 4 - NET  (WiFi + HTTPS tile downloader + saved credentials)
// =====================================================================
class Net {
public:
    void begin() {
        loadKnownNetworks();
        if (_known.empty() && strlen(WIFI_SSID) > 0) rememberNetwork(WIFI_SSID, WIFI_PASS);
        _client.setInsecure();
        _http.setReuse(true);   // keep one TLS/TCP session alive across tiles instead of a fresh handshake each time
        radioOff();   // stay off until needed (saves ~40 KB during normal use)
    }
    void radioOff() { _client.stop(); WiFi.disconnect(true); WiFi.mode(WIFI_OFF); }
    void connectTo(const char *ssid, const char *pass, bool save) {
        _enabled = true;
        WiFi.mode(WIFI_STA); WiFi.disconnect(); WiFi.begin(ssid, pass);
        if (save) rememberNetwork(ssid, pass);
    }
    bool hasCreds() const { return !_known.empty(); }

    // Known/saved networks, most-recently-used first -- for the WiFi menu's
    // "forget a network" screen.
    int    knownCount() const { return (int)_known.size(); }
    String knownSsid(int i) const { return (i >= 0 && i < (int)_known.size()) ? _known[i].ssid : String(); }
    void   forgetNetwork(int i) {
        if (i < 0 || i >= (int)_known.size()) return;
        bool wasCurrent = online() && WiFi.SSID() == _known[i].ssid;
        _known.erase(_known.begin() + i);
        saveKnownNetworks();
        if (wasCurrent) radioOff();   // don't leave it connected to a network we just forgot
    }

    // Power the radio, scan, and connect to whichever KNOWN network is both
    // in range and strongest (phone-style "remembered networks" behavior,
    // per request). Blocking, with a themed spinner. The scan adds a couple
    // of seconds up front versus blindly retrying the last-used network, but
    // means it picks the best of several remembered networks when more than
    // one is in range.
    template <class G> bool ensureConnected(G &d) {
        if (online()) return true;
        if (_known.empty()) return false;
        _enabled = true;
        WiFi.mode(WIFI_STA);

        // WiFi.scanNetworks() blocks synchronously -- no loop to animate a
        // spinner against, so just show a static message while it runs.
        { int w = 176, x = (SCREEN_W-w)/2, y = 54;
          d.fillRoundRect(x, y, w, 24, 4, TFT_BLACK); d.drawRoundRect(x, y, w, 24, 4, Theme::fg(d));
          d.setTextColor(Theme::fg(d), TFT_BLACK); d.setTextSize(1); d.setCursor(x+8, y+8);
          d.print("SCANNING WIFI..."); }

        int n = WiFi.scanNetworks();
        String bestSsid, bestPass; int bestRssi = -1000;
        for (int i = 0; i < n; ++i) {
            String ssid = WiFi.SSID(i);
            for (auto &k : _known)
                if (k.ssid == ssid && WiFi.RSSI(i) > bestRssi) { bestRssi = WiFi.RSSI(i); bestSsid = ssid; bestPass = k.pass; }
        }
        WiFi.scanDelete();
        if (bestSsid.length() == 0) return false;   // no known network is in range

        WiFi.begin(bestSsid.c_str(), bestPass.c_str());
        uint32_t t0 = millis(); int frame = 0;
        while (WiFi.status() != WL_CONNECTED && millis() - t0 < 12000UL) {
            int w = 176, x = (SCREEN_W-w)/2, y = 54;
            d.fillRoundRect(x, y, w, 24, 4, TFT_BLACK); d.drawRoundRect(x, y, w, 24, 4, Theme::fg(d));
            d.setTextColor(Theme::fg(d), TFT_BLACK); d.setTextSize(1); d.setCursor(x+8, y+8);
            char dots[5] = {0}; for (int i = 0; i < (frame%4); ++i) dots[i] = '.';
            d.printf("CONNECTING WIFI%s   ", dots);
            frame++; delay(200);
        }
        return online();
    }
    bool enabled() const { return _enabled; }
    bool online()  const { return WiFi.status() == WL_CONNECTED; }
    bool downloadTile(const char *root, const char *urlTmpl, int z, int x, int y, const char *path) {
        if (!online()) return false;
        char dir[64];
        SD.mkdir(root);
        snprintf(dir, sizeof(dir), "%s/%d", root, z);        SD.mkdir(dir);
        snprintf(dir, sizeof(dir), "%s/%d/%d", root, z, x);  SD.mkdir(dir);
        char url[160];
        snprintf(url, sizeof(url), urlTmpl, z, x, y);
        for (int attempt = 0; attempt < 3; ++attempt) {
            if (attempt) { _client.stop(); delay(250); }   // drop a half-broken connection before retrying
            _http.setUserAgent(TILE_USER_AGENT);
            _http.setConnectTimeout(6000); _http.setTimeout(15000);
            if (!_http.begin(_client, url)) continue;
            int code = _http.GET();
            if (code != HTTP_CODE_OK) { Serial.printf("[tile] GET %s -> HTTP %d\n", url, code); _http.end(); continue; }
            int expected = _http.getSize();           // Content-Length (-1 if chunked/unknown)
            File f = SD.open(path, FILE_WRITE);
            if (!f) { _http.end(); return false; }
            size_t written = _http.writeToStream(&f);
            f.close(); _http.end();
            File chk = SD.open(path);
            bool ok = false; size_t fsz = 0; bool sigok = false, sizeok = false, iendok = false;
            if (chk) {
                fsz = chk.size();
                uint8_t sig[8];
                sigok  = (chk.read(sig, 8) == 8 && sig[0]==0x89 && sig[1]==0x50 && sig[2]==0x4E && sig[3]==0x47);
                sizeok = (fsz > 100) && (expected <= 0 || fsz == (size_t)expected);
                // Content-Length is often absent (chunked HTTPS response), which used to make
                // sizeok the only real check -- a connection that dropped mid-transfer could
                // still pass (>100 bytes, valid header) and get saved as "complete" even though
                // the PNG stream was truncated, so it downloads fine but never decodes. Every
                // valid PNG ends with a fixed 12-byte IEND chunk; require it explicitly.
                if (fsz >= 12) {
                    uint8_t tail[12];
                    if (chk.seek(fsz - 12) && chk.read(tail, 12) == 12)
                        iendok = (tail[4]=='I' && tail[5]=='E' && tail[6]=='N' && tail[7]=='D');
                }
                ok = sigok && sizeok && iendok;
                chk.close();
            }
            Serial.printf("[tile] %s code=%d expected=%d written=%u fsz=%u sig=%d size=%d iend=%d -> %s\n",
                           url, code, expected, (unsigned)written, (unsigned)fsz, sigok, sizeok, iendok, ok ? "OK" : "BAD");
            if (ok) return true;
            SD.remove(path);                         // drop bad/partial, try again
        }
        return false;
    }
private:
    struct WifiCred { String ssid, pass; };
    // Move-to-front on reconnect/re-save (so a network you use often stays
    // near the front), insert-at-front for a new one, drop the oldest past
    // MAX_WIFI_NETWORKS -- a simple LRU with a bounded list.
    void rememberNetwork(const char *ssid, const char *pass) {
        for (size_t i = 0; i < _known.size(); ++i)
            if (_known[i].ssid == ssid) { _known.erase(_known.begin() + i); break; }
        WifiCred c; c.ssid = ssid; c.pass = pass;
        _known.insert(_known.begin(), c);
        if ((int)_known.size() > MAX_WIFI_NETWORKS) _known.resize(MAX_WIFI_NETWORKS);
        saveKnownNetworks();
    }
    void saveKnownNetworks() {
        File f = SD.open(WIFI_SAVE_PATH, FILE_WRITE);
        if (!f) return;
        for (auto &k : _known) { f.println(k.ssid); f.println(k.pass); }
        f.close();
    }
    void loadKnownNetworks() {
        _known.clear();
        File f = SD.open(WIFI_SAVE_PATH, FILE_READ);
        if (!f) return;
        while (f.available() && (int)_known.size() < MAX_WIFI_NETWORKS) {
            String ssid = f.readStringUntil('\n'); ssid.trim();
            if (ssid.length() == 0) break;
            String pass = f.readStringUntil('\n'); pass.trim();
            _known.push_back({ ssid, pass });
        }
        f.close();
    }
    bool _enabled = false;
    std::vector<WifiCred> _known;
    // Persistent across calls so successive tile fetches can reuse one TLS
    // session (setReuse(true) in begin()) instead of a fresh handshake per
    // tile -- that repeated handshake churn was the main source of both
    // failed downloads and heap fragmentation right after a download batch.
    WiFiClientSecure _client;
    HTTPClient       _http;
};

// =====================================================================
//  MODULE 5 - TRACK RECORDER  (writes GPX to SD)
// =====================================================================
class TrackRec {
public:
    bool recording() const { return _rec; }             // session open (recording OR paused)
    bool paused()    const { return _rec && _paused; }
    int  count()     const { return _count; }
    const String& name() const { return _name; }

    bool toggle(Gps &gps) { return _rec ? (stop(), false) : start(gps); }
    // Keeps the file open and _lastLat/_lastLon intact so the distance gate
    // picks up cleanly on resume -- only start()/stop() touch those.
    void togglePause() { if (_rec) _paused = !_paused; }

    bool start(Gps &gps) {
        SD.mkdir(TRACK_DIR);
        char fn[64];
        if (gps.dateValid() && gps.timeValid())
            snprintf(fn, sizeof(fn), "%s/trk_%04d%02d%02d_%02d%02d%02d.gpx",
                     TRACK_DIR, gps.yy(), gps.mo(), gps.dd(), gps.hh(), gps.mm(), gps.ss());
        else
            snprintf(fn, sizeof(fn), "%s/trk_%lu.gpx", TRACK_DIR, (unsigned long)millis());
        _f = SD.open(fn, FILE_WRITE);
        if (!_f) return false;
        _name = fn; _rec = true; _paused = false; _has = false; _count = 0;
        _f.print("<?xml version=\"1.0\"?>\n<gpx version=\"1.1\" creator=\"cardputer-adv-mapviewer\">\n<trk><trkseg>\n");
        _f.flush();
        return true;
    }
    void addPoint(Gps &gps, float minDistM) {
        if (!_rec || _paused || !gps.hasFix()) return;
        double la = gps.lat(), lo = gps.lon();
        if (_has && TinyGPSPlus::distanceBetween(la, lo, _lastLat, _lastLon) < minDistM) return;
        _f.printf("<trkpt lat=\"%.6f\" lon=\"%.6f\"><ele>%.1f</ele>", la, lo, gps.altitude());
        if (gps.dateValid() && gps.timeValid())
            _f.printf("<time>%04d-%02d-%02dT%02d:%02d:%02dZ</time>",
                      gps.yy(), gps.mo(), gps.dd(), gps.hh(), gps.mm(), gps.ss());
        _f.print("</trkpt>\n");
        _lastLat = la; _lastLon = lo; _has = true; _count++;
        // Flush every point, not batched -- points are already gated to one per
        // minDistM of movement (user-configurable, Settings), so this is at
        // most a few writes a minute even walking briskly, and it caps the
        // worst-case data-loss window on power loss to "this one point"
        // instead of up to 9.
        _f.flush();
    }
    void stop() {
        if (!_rec) return;
        _f.print("</trkseg></trk></gpx>\n");
        _f.close(); _rec = false; _paused = false;
    }
private:
    File   _f;
    String _name;
    double _lastLat = 0, _lastLon = 0;
    bool   _rec = false, _paused = false, _has = false;
    int    _count = 0;
};

// =====================================================================
//  MODULE 6 - TRACK VIEW  (a loaded track drawn on the map)
// =====================================================================
struct TrackView {
    std::vector<float> lat, lon;
    String name;
    bool   loaded = false;

    void clear() { lat.clear(); lon.clear(); loaded = false; name = ""; }

    // MAX_TRACK_POINTS is a fixed RAM budget (see config.h), not a "first N
    // points" cutoff: a full-day track can have several times that many
    // points, so loading the first MAX_TRACK_POINTS verbatim would show only
    // the start of the hike. Instead this streams the file TWICE off SD --
    // once to just count points (no storage), once to pick every Nth one --
    // so the loaded points are spread across the WHOLE route and the shape
    // of the entire hike is visible, just at reduced point density. RAM use
    // stays flat at MAX_TRACK_POINTS regardless of how long the hike was;
    // the cost is a bit of extra SD read time on open and slightly coarser
    // corners between the kept points (uniform decimation, not
    // shape-adaptive simplification).
    bool load(const char *path) {
        clear();
        long total = 0;
        {
            File f = SD.open(path, FILE_READ);
            if (!f) return false;
            while (f.available()) {
                String line = f.readStringUntil('\n');
                if (line.indexOf("lat=\"") >= 0 && line.indexOf("lon=\"") >= 0) total++;
            }
            f.close();
        }
        if (total <= 0) return false;
        long stride = (total + MAX_TRACK_POINTS - 1) / MAX_TRACK_POINTS;
        if (stride < 1) stride = 1;
        long cap = total < (long)MAX_TRACK_POINTS ? total : (long)MAX_TRACK_POINTS;
        lat.reserve(cap); lon.reserve(cap);

        File f = SD.open(path, FILE_READ);
        if (!f) return false;
        long idx = 0;
        while (f.available()) {
            String line = f.readStringUntil('\n');
            int a = line.indexOf("lat=\"");
            int o = line.indexOf("lon=\"");
            if (a < 0 || o < 0) continue;
            bool keep = (idx % stride == 0) || (idx == total - 1);   // always keep the endpoint
            if (keep) {
                float la = line.substring(a + 5).toFloat();
                float lo = line.substring(o + 5).toFloat();
                lat.push_back(la); lon.push_back(lo);
                if ((int)lat.size() >= MAX_TRACK_POINTS) break;
            }
            idx++;
        }
        f.close();
        name = path;
        loaded = lat.size() > 1;
        return loaded;
    }
};

// =====================================================================
//  MODULE 6b - WAYPOINTS  (tag places with an icon + note; saved to SD)
// =====================================================================
struct IconDef { const char *name; char letter; uint8_t r, g, b; };
static const IconDef ICONS[] = {
    { "Food",         'F',  60, 200,  80 },
    { "Gasoline",     'G', 255, 150,   0 },
    { "Deadly",       'S', 235, 235, 235 },
    { "Danger",       'X', 255,  60,  60 },
    { "Info",         'I',  60, 150, 255 },
    { "Water",        'W',   0, 200, 255 },
    { "Transport",    'B', 255, 210,  40 },
    { "Train",        'T', 200, 140, 255 },
    { "Hospital",     'H', 255,  90,  90 },
    { "Caution",      '!', 255, 220,   0 },
    { "Shelter",      'S', 210, 170, 100 },
    { "Power source", 'P', 255, 235,  40 },
};
static const int ICON_COUNT = (int)(sizeof(ICONS) / sizeof(ICONS[0]));

template <class GFX>
static void drawWpIcon(GFX &g, int sx, int sy, int icon) {
    if (icon < 0 || icon >= ICON_COUNT) icon = 4;
    const IconDef &d = ICONS[icon];
    uint16_t c = g.color565(d.r, d.g, d.b);
    const int R = 10;                                   // larger, more noticeable
    const uint16_t K = TFT_BLACK, W = TFT_WHITE;
    g.fillCircle(sx, sy, R, c); g.drawCircle(sx, sy, R, W); g.drawCircle(sx, sy, R - 1, K);
    switch (icon) {
        case 0: // Food - fork + knife
            g.drawFastVLine(sx-4, sy-5, 11, K); g.drawFastVLine(sx-2, sy-5, 11, K);
            g.drawFastVLine(sx,   sy-5, 5,  K); g.drawFastVLine(sx+3, sy-5, 11, K); break;
        case 1: // Gasoline - pump
            g.fillRect(sx-5, sy-5, 7, 11, K); g.drawFastVLine(sx+4, sy-2, 6, K); g.fillRect(sx+4, sy-3, 2, 1, K); break;
        case 2: // Deadly - skull
            g.fillCircle(sx, sy-2, 7, W);                        // cranium
            g.fillRect(sx-4, sy+3, 9, 4, W);                     // jaw
            g.fillCircle(sx-3, sy-2, 2, K); g.fillCircle(sx+3, sy-2, 2, K); // eye sockets
            g.fillTriangle(sx, sy, sx-1, sy+2, sx+1, sy+2, K);   // nose
            g.drawFastVLine(sx-2, sy+3, 4, K); g.drawFastVLine(sx, sy+3, 4, K); g.drawFastVLine(sx+2, sy+3, 4, K); // teeth
            break;
        case 3: // X mark
            g.drawLine(sx-5, sy-5, sx+5, sy+5, K); g.drawLine(sx-5, sy+5, sx+5, sy-5, K);
            g.drawLine(sx-4, sy-5, sx+5, sy+4, K); g.drawLine(sx-5, sy+4, sx+4, sy-5, K); break;
        case 4: // Info - i
            g.fillCircle(sx, sy-5, 1, K); g.fillRect(sx-1, sy-2, 3, 8, K); break;
        case 5: // Water - droplet
            g.fillTriangle(sx, sy-6, sx-5, sy+1, sx+5, sy+1, W); g.fillCircle(sx, sy+2, 5, W); break;
        case 6: // Bus
            g.fillRect(sx-6, sy-4, 12, 8, K);
            g.fillRect(sx-5, sy-3, 4, 4, c); g.fillRect(sx+1, sy-3, 4, 4, c);
            g.fillCircle(sx-3, sy+5, 1, K); g.fillCircle(sx+3, sy+5, 1, K); break;
        case 7: // Train
            g.fillRect(sx-5, sy-6, 10, 9, K); g.fillRect(sx-4, sy-5, 8, 3, c);
            g.fillCircle(sx-3, sy+5, 1, K); g.fillCircle(sx+3, sy+5, 1, K); break;
        case 8: // Hospital - cross
            g.fillRect(sx-2, sy-6, 4, 13, W); g.fillRect(sx-6, sy-2, 13, 4, W); break;
        case 9: // Caution - triangle + !
            g.fillTriangle(sx, sy-6, sx-6, sy+5, sx+6, sy+5, K);
            g.drawFastVLine(sx, sy-2, 4, c); g.fillCircle(sx, sy+3, 1, c); break;
        case 10: // Shelter - house
            g.fillTriangle(sx, sy-7, sx-7, sy-1, sx+7, sy-1, W); // roof
            g.fillRect(sx-5, sy-1, 11, 7, W);                    // body
            g.fillRect(sx-1, sy+1, 3, 5, K);                     // door
            break;
        case 11: // Power source - lightning bolt
            g.fillTriangle(sx+2, sy-7, sx-4, sy+1, sx+1, sy+1, K);
            g.fillTriangle(sx-2, sy+7, sx+4, sy-1, sx-1, sy-1, K);
            break;
        default:
            g.setTextColor(K); g.setTextSize(1); g.setTextDatum(middle_center);
            { char t[2] = { d.letter, 0 }; g.drawString(t, sx, sy); } g.setTextDatum(top_left); break;
    }
}

struct Waypoint { double lat, lon; int icon; String time, note; };

class WaypointStore {
public:
    void load() {
        _v.clear();
        File f = SD.open(WAYPT_PATH, FILE_READ);
        if (!f) return;
        while (f.available() && (int)_v.size() < MAX_WAYPOINTS) {
            String line = f.readStringUntil('\n'); line.trim();
            Waypoint w; if (line.length() && parse(line, w)) _v.push_back(w);
        }
        f.close();
    }
    int count() const { return (int)_v.size(); }
    // Bounds-checked: the notes UI filters/sorts through an index view, and a
    // delete can empty that view a frame before the screen stops asking for a
    // row, so an out-of-range read has to return something inert rather than
    // walk off the vector.
    const Waypoint &get(int i) const {
        static const Waypoint kNone{0, 0, 0, String(), String()};
        if (i < 0 || i >= (int)_v.size()) return kNone;
        return _v[i];
    }
    bool add(const Waypoint &w) {
        if ((int)_v.size() >= MAX_WAYPOINTS) return false;
        _v.push_back(w);
        File f = SD.open(WAYPT_PATH, FILE_APPEND);
        if (!f) return false;
        f.printf("%.6f,%.6f,%d,%s,%s\n", w.lat, w.lon, w.icon,
                 w.time.length() ? w.time.c_str() : "-", w.note.c_str());
        f.close();
        return true;
    }
    void remove(int i) {
        if (i < 0 || i >= (int)_v.size()) return;
        _v.erase(_v.begin() + i);
        File f = SD.open(WAYPT_PATH, FILE_WRITE);
        if (!f) return;
        for (auto &w : _v)
            f.printf("%.6f,%.6f,%d,%s,%s\n", w.lat, w.lon, w.icon,
                     w.time.length() ? w.time.c_str() : "-", w.note.c_str());
        f.close();
    }
    void rename(int i, const String &newNote) {
        if (i < 0 || i >= (int)_v.size()) return;
        _v[i].note = newNote;
        File f = SD.open(WAYPT_PATH, FILE_WRITE);
        if (!f) return;
        for (auto &w : _v)
            f.printf("%.6f,%.6f,%d,%s,%s\n", w.lat, w.lon, w.icon,
                     w.time.length() ? w.time.c_str() : "-", w.note.c_str());
        f.close();
    }
private:
    bool parse(const String &line, Waypoint &w) {
        int c1 = line.indexOf(','), c2 = line.indexOf(',', c1 + 1),
            c3 = line.indexOf(',', c2 + 1), c4 = line.indexOf(',', c3 + 1);
        if (c1 < 0 || c2 < 0 || c3 < 0 || c4 < 0) return false;
        w.lat  = line.substring(0, c1).toFloat();
        w.lon  = line.substring(c1 + 1, c2).toFloat();
        w.icon = line.substring(c2 + 1, c3).toInt();
        w.time = line.substring(c3 + 1, c4);
        w.note = line.substring(c4 + 1);
        return true;
    }
    std::vector<Waypoint> _v;
};

// =====================================================================
//  MODULE 7 - MAP VIEW   (buffered, scroll-blit; pan/zoom apply instantly)
// =====================================================================
struct TileRange { int rz; float f; double tileScreen; int txMin, txMax, tyMin, tyMax; double topLeftX, topLeftY; int n; };

class MapView {
public:
    void begin(int baseDefault = 0) {
        _base = (baseDefault >= 0 && baseDefault < BASEMAP_COUNT) ? baseDefault : 0;
        _zoom = MAP_ZOOM;
        _curX = _tgtX = Slippy::lonToWorldPx(DEFAULT_LON, _zoom);
        _curY = _tgtY = Slippy::latToWorldPx(DEFAULT_LAT, _zoom);
        _needFull = true; _dirty = true;
    }
    void setTrack(const TrackView *t) { _track = t; _twZoom = -1; _dirty = true; }
    void setWaypoints(const WaypointStore *w) { _wps = w; _dirty = true; }
    void toggleIcons()      { _showWps = !_showWps; _soloWp = -1; _dirty = true; }
    void setIconsVisible(bool v) { _showWps = v; _dirty = true; }   // initial state at boot, from Settings
    void setSolo(int i)     { _soloWp = i; _showWps = true; _dirty = true; }
    void nextBasemap()      { _base = (_base + 1) % BASEMAP_COUNT; _needFull = true; _dirty = true; }
    const char *baseTag() const { return BASEMAPS[_base].tag; }
    void tilesPresent(int &present, int &total) const {
        TileRange r = range(); present = 0; total = 0;
        for (int ty = r.tyMin; ty <= r.tyMax; ++ty)
            for (int tx = r.txMin; tx <= r.txMax; ++tx) {
                if (ty < 0 || ty >= r.n) continue;
                total++;
                int wx = ((tx % r.n) + r.n) % r.n;
                char path[80]; snprintf(path, sizeof(path), "%s/%d/%d/%d.png", BASEMAPS[_base].root, r.rz, wx, ty);
                if (SD.exists(path)) present++;
            }
    }

    void markDirty()         { _dirty = true; }
    bool consumeDirty()      { bool d = _dirty; _dirty = false; return d; }
    void pan(int dx, int dy) { _followGps = false; _tgtX += dx; _tgtY += dy; _curX = _tgtX; _curY = _tgtY; _dirty = true; }
    void recenterOnGps() {
        _followGps = true;
        if (_haveGps) {                         // snap now; don't fly across the map
            _curX = _tgtX = Slippy::lonToWorldPx(_gpsLon, _zoom);
            _curY = _tgtY = Slippy::latToWorldPx(_gpsLat, _zoom);
            _dirty = true;
        }
    }
    void centerOn(double la, double lo) {   // jump the view to a point (used by track viewer)
        _followGps = false;
        _curX = _tgtX = Slippy::lonToWorldPx(lo, _zoom);
        _curY = _tgtY = Slippy::latToWorldPx(la, _zoom);
        _dirty = true;
    }
    void zoomIn()            { setZoom(_zoom + 1); }
    void zoomOut()           { setZoom(_zoom - 1); }
    void onGpsFix(double la, double lo) {
        _gpsLat = la; _gpsLon = lo; _haveGps = true;
        if (_followGps) {
            _curX = _tgtX = Slippy::lonToWorldPx(lo, _zoom);
            _curY = _tgtY = Slippy::latToWorldPx(la, _zoom);
            _dirty = true;
        }
    }

    int  zoom()      const { return _zoom; }
    bool following() const { return _followGps; }
    int  originX()   const { return _originX; }   // top-left of the current screen, world px
    int  originY()   const { return _originY; }
    double centerLat() const { return Slippy::worldPxToLat(_curY, _zoom); }
    double centerLon() const { return Slippy::worldPxToLon(_curX, _zoom); }
    // Tile the crosshair currently sits in, AT THE DISPLAYED ZOOM (not the
    // over-zoom-clamped native fetch zoom) -- this is what the status bar
    // shows next to "Z<n>" and what BulkDownloadUi suggests as a starting
    // point for a range, since it's the zoom level the user is looking at.
    void centerTileXY(int &tx, int &ty) const {
        int n = 1 << _zoom;
        tx = ((int)floor(_curX / TILE_SIZE) % n + n) % n;
        ty = (int)floor(_curY / TILE_SIZE);   // not wrapped -- valid mercator range is already [0,n)
    }
    int baseIndex() const { return _base; }

    // ---- Tile drawing helpers (support scroll-blit) ----
    template <class GFX>
    void drawTilesRegion(GFX &g, const TileRange &r, int ox, int oy,
                         int x0, int y0, int x1, int y1) {
        const double ts = r.tileScreen;
        int txMin = (int)floor((ox + x0) / ts), txMax = (int)floor((ox + x1 - 1) / ts);
        int tyMin = (int)floor((oy + y0) / ts), tyMax = (int)floor((oy + y1 - 1) / ts);
        const int tsz = (int)ceil(ts);
        for (int ty = tyMin; ty <= tyMax; ++ty) {
            for (int tx = txMin; tx <= txMax; ++tx) {
                const int wx = ((tx % r.n) + r.n) % r.n;
                const int sx = (int)lround(tx * ts - ox);
                const int sy = (int)lround(ty * ts - oy);
                if (ty < 0 || ty >= r.n) { g.fillRect(sx, sy, tsz, tsz, g.color565(10,12,20)); continue; }
                char path[80];
                snprintf(path, sizeof(path), "%s/%d/%d/%d.png", baseRoot(), r.rz, wx, ty);
                bool have = SD.exists(path);
                if (!have || !drawPngFromSd(g, path, sx, sy, r.f)) {
                    g.fillRect(sx, sy, tsz, tsz, g.color565(28,30,40));
                    g.setTextColor(g.color565(90,90,100)); g.setTextSize(1);
                    g.setCursor(sx + 6, sy + 6); g.print("no tile");
                }
            }
        }
    }
    template <class GFX>
    void drawMapFull(GFX &g) {
        TileRange r = range();
        _originX = (int)lround(r.topLeftX); _originY = (int)lround(r.topLeftY);
        _originZoom = _zoom; _originBase = _base; _needFull = false;
        drawTilesRegion(g, r, _originX, _originY, 0, 0, SCREEN_W, SCREEN_H);
    }
    template <class GFX>
    void updateMapLayer(GFX &g) {
        TileRange r = range();
        if (_needFull || _originZoom != _zoom || _originBase != _base) { drawMapFull(g); return; }
        int nx = (int)lround(r.topLeftX), ny = (int)lround(r.topLeftY);
        int dx = nx - _originX, dy = ny - _originY;
        if (dx == 0 && dy == 0) return;
        if (abs(dx) >= SCREEN_W || abs(dy) >= SCREEN_H) { drawMapFull(g); return; }
        g.scroll(-dx, -dy);
        _originX = nx; _originY = ny;
        if      (dx > 0) drawTilesRegion(g, r, nx, ny, SCREEN_W - dx, 0, SCREEN_W, SCREEN_H);
        else if (dx < 0) drawTilesRegion(g, r, nx, ny, 0, 0, -dx, SCREEN_H);
        if      (dy > 0) drawTilesRegion(g, r, nx, ny, 0, SCREEN_H - dy, SCREEN_W, SCREEN_H);
        else if (dy < 0) drawTilesRegion(g, r, nx, ny, 0, 0, SCREEN_W, -dy);
    }
    template <class GFX>
    void drawOverlays(GFX &g) {
        drawTrack(g);
        drawWaypoints(g);
        drawGpsMarker(g);
        drawCenterCross(g);
    }

    void downloadVisible(Gfx &d, Net &net, bool force = false) {
        if (!net.online()) { notice(d, net.enabled() ? "WIFI NOT CONNECTED" : "PRESS w FOR WIFI"); return; }
        TileRange r = range();
        int total = 0, done = 0, ok = 0, fails = 0;
        for (int ty = r.tyMin; ty <= r.tyMax; ++ty)
            for (int tx = r.txMin; tx <= r.txMax; ++tx)
                if (ty >= 0 && ty < r.n) total++;
        { char m0[48]; snprintf(m0, sizeof(m0), "%s %s  connecting...", force?"RE-DL":"DL", baseTag()); progress(d, m0, 0, total); }
        uint32_t dl0 = millis();
        const char *err = nullptr;
        for (int ty = r.tyMin; ty <= r.tyMax && !err; ++ty) {
            for (int tx = r.txMin; tx <= r.txMax && !err; ++tx) {
                if (ty < 0 || ty >= r.n) continue;
                const int wx = ((tx % r.n) + r.n) % r.n;
                char path[80];
                snprintf(path, sizeof(path), "%s/%d/%d/%d.png", baseRoot(), r.rz, wx, ty);
                if (force || !SD.exists(path)) {
                    if (net.downloadTile(baseRoot(), baseUrl(), r.rz, wx, ty, path)) { ok++; fails = 0; }
                    else fails++;
                }
                done++;
                char msg[44];
                snprintf(msg, sizeof(msg), "%s %s z%d %d/%d ok:%d", force?"RE-DL":"DL", baseTag(), r.rz, done, total, ok);
                progress(d, msg, done, total);
                delay(120);   // be gentle to donated tile servers
                if (!net.online() || fails >= 8)          err = "CONNECTION LOST";
                else if (millis() - dl0 > 300000UL)       err = "TIMED OUT (5 min)";
            }
        }
        _needFull = true; _dirty = true;
        char m[44];
        if (err) snprintf(m, sizeof(m), "FAILED: %s", err);
        else     snprintf(m, sizeof(m), "DONE  %d/%d tiles", ok, total);
        notice(d, m, 84);   // below the progress bar (which occupies y 44..80), not on top of it
    }

    // Fetch + decode + render ONLY the tile under the crosshair (screen centre).
    // Each tile in this app is already downloaded and drawn independently --
    // one missing/failed neighbor never blanks another tile -- so this exists
    // to isolate a single tile for a fast, unambiguous OK/FAILED result instead
    // of waiting on a whole-view batch.
    void downloadCenterTile(Gfx &d, Net &net) {
        if (!net.online()) { notice(d, net.enabled() ? "WIFI NOT CONNECTED" : "PRESS w FOR WIFI"); return; }
        TileRange r = range();
        int tx = (int)floor(_curX / r.tileScreen);
        int ty = (int)floor(_curY / r.tileScreen);
        if (ty < 0 || ty >= r.n) { notice(d, "OUT OF RANGE"); return; }
        const int wx = ((tx % r.n) + r.n) % r.n;
        char path[80];
        snprintf(path, sizeof(path), "%s/%d/%d/%d.png", baseRoot(), r.rz, wx, ty);
        char m0[48]; snprintf(m0, sizeof(m0), "GET %s z%d  %d,%d", baseTag(), r.rz, wx, ty);
        progress(d, m0, 0, 1);
        bool got = net.downloadTile(baseRoot(), baseUrl(), r.rz, wx, ty, path);
        progress(d, m0, 1, 1);
        _needFull = true; _dirty = true;
        char m[44];
        snprintf(m, sizeof(m), got ? "TILE OK  %d,%d" : "TILE FAILED  %d,%d", wx, ty);
        notice(d, m, 84);
    }
private:
    TileRange range() const {
        TileRange r;
        int mn = BASEMAPS[_base].maxNative;
        r.rz = (_zoom > mn) ? mn : _zoom;          // native tile zoom to fetch
        r.f  = (float)pow(2.0, _zoom - r.rz);       // >1 when over-zooming
        r.n  = 1 << r.rz;
        r.tileScreen = (double)TILE_SIZE * r.f;
        r.topLeftX = _curX - SCREEN_W / 2.0;
        r.topLeftY = _curY - SCREEN_H / 2.0;
        // Use SCREEN_W-1 / SCREEN_H-1 (the last visible pixel, not one past it) so this
        // matches drawTilesRegion()'s own tx/tyMax math exactly -- otherwise, when the
        // view sits precisely on a tile boundary, downloadVisible() fetches one extra
        // column/row that drawMapFull() never actually looks at when rendering.
        r.txMin = (int)floor(r.topLeftX / r.tileScreen);
        r.txMax = (int)floor((r.topLeftX + SCREEN_W - 1) / r.tileScreen);
        r.tyMin = (int)floor(r.topLeftY / r.tileScreen);
        r.tyMax = (int)floor((r.topLeftY + SCREEN_H - 1) / r.tileScreen);
        return r;
    }
    const char *baseRoot() const { return BASEMAPS[_base].root; }
    const char *baseUrl()  const { return BASEMAPS[_base].url; }
    void setZoom(int z) {
        if (z < MAP_ZOOM_MIN) z = MAP_ZOOM_MIN;
        if (z > MAP_ZOOM_MAX) z = MAP_ZOOM_MAX;
        if (z == _zoom) return;
        double f = pow(2.0, z - _zoom);
        _curX *= f; _curY *= f; _tgtX *= f; _tgtY *= f;
        _zoom = z; _needFull = true; _dirty = true;
    }
    template <class GFX>
    void drawTrack(GFX &g) {
        if (!_track || !_track->loaded) return;
        const size_t n = _track->lat.size();
        if (_twZoom != _zoom || _twx.size() != n) {   // recompute pixels once per zoom
            _twx.resize(n); _twy.resize(n);
            for (size_t i = 0; i < n; ++i) {
                _twx[i] = (float)Slippy::lonToWorldPx(_track->lon[i], _zoom);
                _twy[i] = (float)Slippy::latToWorldPx(_track->lat[i], _zoom);
            }
            _twZoom = _zoom;
        }
        int px = 0, py = 0; bool have = false;
        for (size_t i = 0; i < n; ++i) {
            int x = (int)lround(_twx[i] - _originX);
            int y = (int)lround(_twy[i] - _originY);
            if (have) {
                bool off = (x < -4 && px < -4) || (x > SCREEN_W+4 && px > SCREEN_W+4) ||
                           (y < -4 && py < -4) || (y > SCREEN_H+4 && py > SCREEN_H+4);
                if (!off) g.drawWideLine(px, py, x, y, 2.2f, g.color565(255, 80, 200));
            }
            px = x; py = y; have = true;
        }
        int sx = (int)lround(_twx[0]   - _originX), sy = (int)lround(_twy[0]   - _originY);
        int ex = (int)lround(_twx[n-1] - _originX), ey = (int)lround(_twy[n-1] - _originY);
        g.fillCircle(sx, sy, 4, TFT_GREEN); g.drawCircle(sx, sy, 4, TFT_WHITE);
        g.fillRect(ex-4, ey-4, 8, 8, TFT_RED); g.drawRect(ex-4, ey-4, 8, 8, TFT_WHITE);
    }
    template <class GFX>
    void drawWaypoints(GFX &g) {
        if (!_wps || !_showWps) return;
        int a = _soloWp >= 0 ? _soloWp : 0;
        int b = _soloWp >= 0 ? _soloWp + 1 : _wps->count();
        for (int i = a; i < b && i < _wps->count(); ++i) {
            const Waypoint &w = _wps->get(i);
            int x = (int)lround(Slippy::lonToWorldPx(w.lon, _zoom) - _originX);
            int y = (int)lround(Slippy::latToWorldPx(w.lat, _zoom) - _originY);
            if (x < -8 || x > SCREEN_W + 8 || y < -8 || y > SCREEN_H + 8) continue;
            drawWpIcon(g, x, y, w.icon);
            if (_soloWp >= 0 && w.note.length()) {
                g.setTextColor(TFT_WHITE, TFT_BLACK); g.setTextSize(1); g.setCursor(x + 9, y - 4);
                String nn = w.note; if (nn.length() > 16) nn = nn.substring(0, 16); g.print(nn);
            }
        }
    }
    template <class GFX>
    void drawGpsMarker(GFX &g) {
        if (!_haveGps) return;
        int gx = (int)lround(Slippy::lonToWorldPx(_gpsLon, _zoom) - _originX);
        int gy = (int)lround(Slippy::latToWorldPx(_gpsLat, _zoom) - _originY);
        if (gx < -6 || gx > SCREEN_W + 6 || gy < -6 || gy > SCREEN_H + 6) return;
        g.fillCircle(gx, gy, 5, TFT_WHITE); g.fillCircle(gx, gy, 3, TFT_RED);
    }
    template <class GFX>
    void drawCenterCross(GFX &g) {
        const int cx = SCREEN_W/2, cy = SCREEN_H/2;
        uint16_t c = _followGps ? g.color565(0,200,255) : TFT_YELLOW;
        g.drawFastHLine(cx-8, cy, 17, c); g.drawFastVLine(cx, cy-8, 17, c);
    }
    void toast(Gfx &d, const char *msg)   { Notify::notice(d, msg); }
    void notice(Gfx &d, const char *msg, int y = 54)              { Notify::notice(d, msg, y); }
    void progress(Gfx &d, const char *msg, int done, int total)   { Notify::progress(d, msg, done, total); }
    const TrackView *_track = nullptr;
    const WaypointStore *_wps = nullptr;
    bool   _showWps = true; int _soloWp = -1;
    int    _base = 0;                                  // active basemap index
    std::vector<float> _twx, _twy; int _twZoom = -1;   // cached track pixels for current zoom
    int    _originX=0, _originY=0, _originZoom=-1, _originBase=-1; bool _needFull=true; // scroll-blit state
    double _curX=0, _curY=0, _tgtX=0, _tgtY=0, _gpsLat=0, _gpsLon=0;
    int    _zoom = MAP_ZOOM;
    bool   _haveGps=false, _followGps=true, _dirty=true;
};

// =====================================================================
//  MODULE 8 - WIFI PORTAL
// =====================================================================
class WiFiPortal {
public:
    bool active() const { return _state != HIDDEN; }
    void open() {
        _state = SCANNING; _sel = 0; _top = 0; _pass = ""; _dirty = true;
        WiFi.mode(WIFI_STA); WiFi.scanDelete();   // keep any active connection alive
        WiFi.scanNetworks(true);
    }
    void close(Net &net, MapView &mv) { WiFi.scanDelete(); net.radioOff(); _state = HIDDEN; mv.markDirty(); }
    void handleKey(const KeyState &st, Net &net, MapView &mv) {
        switch (_state) {
        case LIST:
            for (char c : st.word) {
                if      (c == KEY_UP   && _sel > 0)          _sel--;
                else if (c == KEY_DOWN && _sel < _count - 1) _sel++;
                else if (c == 'k')                           { _ksel = 0; _state = KNOWN; }
                else if (c == KEY_WIFI || c == KEY_BACK)          { close(net, mv); return; }
            }
            if (st.enter && _count > 0) { _ssid = WiFi.SSID(_sel); _pass = ""; _state = PASSWORD; }
            _dirty = true; break;
        case PASSWORD:
            if (st.del && _pass.length()) _pass.remove(_pass.length() - 1);
            for (char c : st.word) { if (c == KEY_BACK) { _state = LIST; break; } if (c >= 32 && c < 127) _pass += c; }
            if (st.enter) { net.connectTo(_ssid.c_str(), _pass.c_str(), true); _state = CONNECTING; _t0 = millis(); }
            _dirty = true; break;
        case RESULT: if (st.enter || !st.word.empty()) close(net, mv); break;

        case KNOWN:
            for (char c : st.word) {
                if      (c == KEY_UP   && _ksel > 0)                  _ksel--;
                else if (c == KEY_DOWN && _ksel < net.knownCount()-1) _ksel++;
                else if (c == KEY_BACK || c == 'k')                        { _state = LIST; }
            }
            if (st.enter && net.knownCount() > 0) _state = KNOWN_CONFIRM;
            _dirty = true; break;

        case KNOWN_CONFIRM:
            for (char c : st.word) {
                if (c == 'y') {
                    net.forgetNetwork(_ksel);
                    if (_ksel >= net.knownCount()) _ksel = net.knownCount() > 0 ? net.knownCount()-1 : 0;
                    _state = KNOWN; _dirty = true; return;
                }
                if (c == 'n' || c == KEY_BACK) { _state = KNOWN; _dirty = true; return; }
            }
            break;
        default: break;
        }
    }
    void update(Gfx &d, Net &net) {
        switch (_state) {
        case SCANNING: {
            int r = WiFi.scanComplete();
            if (r >= 0) { _count = r; _state = LIST; _dirty = true; }
            else        { drawCenter(d, "Scanning WiFi..."); }
            if (_state == LIST && _dirty) { drawList(d); _dirty = false; }
            break; }
        case LIST:          if (_dirty) { drawList(d); _dirty = false; } break;
        case PASSWORD:      if (_dirty) { drawPassword(d); _dirty = false; } break;
        case CONNECTING:
            if (WiFi.status() == WL_CONNECTED) { _state = RESULT; _ok = true;  _dirty = true; }
            else if (millis() - _t0 > 15000)   { _state = RESULT; _ok = false; _dirty = true; }
            else drawCenter(d, "Connecting...");
            break;
        case RESULT:        if (_dirty) { drawResult(d); _dirty = false; } break;
        case KNOWN:          if (_dirty) { drawKnown(d, net); _dirty = false; } break;
        case KNOWN_CONFIRM:  if (_dirty) { drawKnownConfirm(d, net); _dirty = false; } break;
        default: break;
        }
    }
private:
    void header(Gfx &d, const char *t) { Theme::header(d, t); }
    void footer(Gfx &d, const char *h) { Theme::footer(d, h); }
    void drawCenter(Gfx &d, const char *m) {
        Theme::screen(d); d.setTextColor(Theme::fg(d)); d.setTextSize(1);
        d.setTextDatum(middle_center); d.drawString(m, SCREEN_W/2, SCREEN_H/2); d.setTextDatum(top_left);
    }
    void drawList(Gfx &d) {
        header(d, "Select WiFi network");
        if (_count == 0) { d.setTextColor(TFT_ORANGE); d.setCursor(6,40); d.print("No networks. 'w' rescan"); }
        else {
            const int rows = 6, rowH = 16;
            if (_sel < _top) _top = _sel;
            if (_sel >= _top + rows) _top = _sel - rows + 1;
            for (int i = 0; i < rows && (_top+i) < _count; ++i) {
                int idx = _top+i, y = 18 + i*rowH; bool sd = (idx==_sel);
                if (sd) d.fillRect(0,y,SCREEN_W,rowH,d.color565(0,60,28));
                d.setTextColor(sd?TFT_WHITE:TFT_LIGHTGREY, sd?d.color565(0,60,28):TFT_BLACK);
                d.setCursor(6,y+4);
                String s = WiFi.SSID(idx); if (s.length()>24) s=s.substring(0,24);
                d.printf("%s%s", (WiFi.encryptionType(idx)!=WIFI_AUTH_OPEN)?"*":" ", s.c_str());
                d.setCursor(SCREEN_W-30,y+4); d.printf("%d", WiFi.RSSI(idx));
            }
        }
        footer(d, "ENTER pick  k known  esc cancel");
    }
    void drawPassword(Gfx &d) {
        header(d, "Password");
        d.setTextColor(TFT_CYAN); d.setCursor(6,24); d.printf("SSID: %s", _ssid.c_str());
        d.setTextColor(TFT_WHITE); d.setCursor(6,48); d.print("Key: ");
        d.print(_pass); d.print('_');   // shown in the clear so you can see what you typed
        footer(d, "type  DEL erase  ENTER join  esc back");
    }
    void drawResult(Gfx &d) {
        header(d, _ok ? "Connected" : "Failed");
        d.setTextColor(_ok?TFT_GREEN:TFT_RED); d.setCursor(6,30);
        if (_ok) { d.printf("%s", _ssid.c_str()); d.setTextColor(TFT_WHITE); d.setCursor(6,50); d.printf("IP %s", WiFi.localIP().toString().c_str()); }
        else     { d.print("Could not connect."); d.setTextColor(TFT_WHITE); d.setCursor(6,50); d.print("Check password / signal."); }
        footer(d, "press any key to continue");
    }
    void drawKnown(Gfx &d, Net &net) {
        header(d, "Known networks");
        int n = net.knownCount();
        if (n == 0) { d.setTextColor(TFT_ORANGE); d.setCursor(6,40); d.print("None saved yet"); }
        else {
            const int rows = 6, rowH = 16;
            if (_ksel < _ktop) _ktop = _ksel;
            if (_ksel >= _ktop + rows) _ktop = _ksel - rows + 1;
            for (int i = 0; i < rows && (_ktop+i) < n; ++i) {
                int idx = _ktop+i, y = 18 + i*rowH; bool sd = (idx==_ksel);
                if (sd) d.fillRect(0,y,SCREEN_W,rowH,d.color565(0,60,28));
                d.setTextColor(sd?TFT_WHITE:TFT_LIGHTGREY, sd?d.color565(0,60,28):TFT_BLACK);
                d.setCursor(6,y+4); d.print(net.knownSsid(idx));
            }
        }
        footer(d, "ENTER forget  esc back");
    }
    void drawKnownConfirm(Gfx &d, Net &net) {
        header(d, "Forget network");
        d.setTextColor(TFT_WHITE,TFT_BLACK); d.setCursor(6,34); d.print(net.knownSsid(_ksel));
        d.setTextColor(TFT_RED,TFT_BLACK); d.setCursor(6,56); d.print("Forget this network?");
        footer(d, "y = yes    n = no");
    }
    enum State { HIDDEN, SCANNING, LIST, PASSWORD, CONNECTING, RESULT, KNOWN, KNOWN_CONFIRM };
    State _state = HIDDEN; int _count=0,_sel=0,_top=0; bool _dirty=true,_ok=false; uint32_t _t0=0; String _ssid,_pass;
    int _ksel=0, _ktop=0;
};

// =====================================================================
//  MODULE 9 - SETTINGS UI  (time zone, basemap, GPS receiver, ...)
//  More rows than fit on 135px, so the list scrolls a 6-row window.
// =====================================================================
class SettingsUi {
public:
    bool active() const { return _open; }
    void open(Settings &s) {
        _open = true; _dirty = true; _row = 0; _top = 0; _editingPos = false; _posErr = false;
        _tz = s.tzHours; _base = s.baseDefault; _bri = s.brightness;
        _trackM = s.trackMinDistM;
        _hasDef = s.hasDefaultPos; _defLat = s.defLat; _defLon = s.defLon;
        _showWpBoot = s.showWaypointsOnBoot;
        _gpsSrc = s.gpsSource;
        _gpsBaudIdx = s.gpsBaudIndex();
        if (_gpsBaudIdx < 0) _gpsBaudIdx = 1;   // GPS_BAUD_CHOICES[1] == 9600
    }
    void handleKey(const KeyState &st, Settings &s, MapView &mv, Gps &gps) {
        if (_editingPos) {   // manual "lat,lon" text entry, same idiom as rename/note fields
            if (st.del && _posEdit.length()) _posEdit.remove(_posEdit.length() - 1);
            for (char c : st.word) {
                if (c == KEY_BACK) { _editingPos = false; _dirty = true; return; }
                if ((c >= '0' && c <= '9') || c == '-' || c == '.' || c == ',') _posEdit += c;
            }
            if (st.enter) {
                double la, lo;
                if (parseLatLon(_posEdit, la, lo)) {
                    _hasDef = true; _defLat = la; _defLon = lo;
                    _editingPos = false; _posErr = false;
                } else _posErr = true;   // stay in the field so they can fix it
            }
            _dirty = true; return;
        }
        for (char c : st.word) {
            if      (c == KEY_UP   && _row > 0)        _row--;
            else if (c == KEY_DOWN && _row < ROWS - 1) _row++;
            else if (c == KEY_LEFT)  {
                if      (_row == 0) _tz -= 0.5f;
                else if (_row == 1) _base = (_base + BASEMAP_COUNT - 1) % BASEMAP_COUNT;
                else if (_row == 2) { _bri -= 16; if (_bri < 16) _bri = 16; M5Cardputer.Display.setBrightness(_bri); }
                else if (_row == 3) { _trackM -= 1.0f; if (_trackM < 3.0f) _trackM = 3.0f; }
                else if (_row == 5) _showWpBoot = !_showWpBoot;
                else if (_row == 6) _gpsSrc = (_gpsSrc + GPS_SRC_COUNT - 1) % GPS_SRC_COUNT;
                else if (_row == 7 && _gpsSrc == GPS_SRC_GROVE)
                    _gpsBaudIdx = (_gpsBaudIdx + GPS_BAUD_CHOICE_COUNT - 1) % GPS_BAUD_CHOICE_COUNT;
            }
            else if (c == KEY_RIGHT) {
                if      (_row == 0) _tz += 0.5f;
                else if (_row == 1) _base = (_base + 1) % BASEMAP_COUNT;
                else if (_row == 2) { _bri += 16; if (_bri > 255) _bri = 255; M5Cardputer.Display.setBrightness(_bri); }
                else if (_row == 3) { _trackM += 1.0f; if (_trackM > (float)TRACK_MIN_DIST_MAX) _trackM = (float)TRACK_MIN_DIST_MAX; }
                else if (_row == 5) _showWpBoot = !_showWpBoot;
                else if (_row == 6) _gpsSrc = (_gpsSrc + 1) % GPS_SRC_COUNT;
                else if (_row == 7 && _gpsSrc == GPS_SRC_GROVE)
                    _gpsBaudIdx = (_gpsBaudIdx + 1) % GPS_BAUD_CHOICE_COUNT;
            }
            else if (c == 'c' && _row == 4) { _hasDef = true; _defLat = mv.centerLat(); _defLon = mv.centerLon(); }
            else if (c == 'e' && _row == 4) {
                char b[24]; if (_hasDef) snprintf(b, sizeof(b), "%.6f,%.6f", _defLat, _defLon); else b[0] = 0;
                _posEdit = b; _posErr = false; _editingPos = true;
            }
            else if (c == 'x' && _row == 4) { _hasDef = false; }   // clear -> falls back to London
            else if (c == KEY_BACK || c == KEY_SETTINGS) { close(s, mv, gps, false); return; }
        }
        if (_tz < -12.0f) _tz = -12.0f;
        if (_tz >  14.0f) _tz =  14.0f;
        if (st.enter) { close(s, mv, gps, true); return; }
        _dirty = true;
    }
    void update(Gfx &d) {
        if (!_dirty) return; _dirty = false;
        if (_editingPos) { drawPosEdit(d); return; }
        Theme::header(d, "SETTINGS");
        // Keep the cursor inside the visible window.
        if (_row < _top)              _top = _row;
        if (_row >= _top + VIS_ROWS)  _top = _row - VIS_ROWS + 1;
        char b[24];
        row(d, 0, "Time zone",   (snprintf(b, sizeof(b), "%+.1f h", _tz), b));
        row(d, 1, "Boot map",    BASEMAPS[_base].name);
        row(d, 2, "Brightness",  (snprintf(b, sizeof(b), "%d%%", (_bri * 100) / 255), b));
        row(d, 3, "Track pt (m)",(snprintf(b, sizeof(b), "%.0f m", _trackM), b));
        if (_hasDef) snprintf(b, sizeof(b), "%.5f,%.5f", _defLat, _defLon);
        else         snprintf(b, sizeof(b), "not set (London)");
        row(d, 4, "Default pos", b);
        row(d, 5, "Wpts on boot", _showWpBoot ? "on" : "off");
        row(d, 6, "GPS module",  _gpsSrc == GPS_SRC_GROVE ? "Grove UART" : "LoRa Cap");
        if (_gpsSrc == GPS_SRC_GROVE) snprintf(b, sizeof(b), "%d", GPS_BAUD_CHOICES[_gpsBaudIdx]);
        else                          snprintf(b, sizeof(b), "%d (fixed)", GPS_BAUD);
        row(d, 7, "GPS baud",    b);
        // Scroll hints in the gutter so it's obvious there's more above/below.
        d.setTextColor(Theme::dim(d), TFT_BLACK); d.setTextSize(1);
        if (_top > 0)                    { d.setCursor(SCREEN_W - 8, 20);  d.print('^'); }
        if (_top + VIS_ROWS < ROWS)      { d.setCursor(SCREEN_W - 8, 105); d.print('v'); }
        switch (_row) {
            case 3:  Theme::footer(d, "left/right +-1m  bigger=fewer pts"); break;
            case 4:  Theme::footer(d, "c capture  e type  x clear"); break;
            case 5:  Theme::footer(d, "left/right toggle  ENTER save"); break;
            case 6:  Theme::footer(d, "Grove: TX->G1 RX->G2  ENTER save"); break;
            case 7:  Theme::footer(d, _gpsSrc == GPS_SRC_GROVE ? "left/right baud  ENTER save"
                                                               : "Cap module is fixed at 115200"); break;
            default: Theme::footer(d, "up/down row  left/right set  ENTER save"); break;
        }
    }
private:
    static constexpr int ROWS     = 8;   // total settings rows
    static constexpr int VIS_ROWS = 6;   // rows that fit between header and footer

    void row(Gfx &d, int idx, const char *label, const char *val) {
        if (idx < _top || idx >= _top + VIS_ROWS) return;
        int y = 20 + (idx - _top) * 17; bool sel = (_row == idx);
        if (sel) d.fillRect(0, y - 3, SCREEN_W, 15, Theme::hl(d));
        d.setTextSize(1);
        d.setTextColor(sel ? Theme::fg(d) : Theme::dim(d), sel ? Theme::hl(d) : TFT_BLACK);
        d.setCursor(8, y); d.print(label);
        d.setTextColor(Theme::fg(d), sel ? Theme::hl(d) : TFT_BLACK);
        d.setCursor(112, y); d.print(val);
    }
    void drawPosEdit(Gfx &d) {
        Theme::header(d, "Default position");
        d.setTextColor(TFT_WHITE, TFT_BLACK); d.setCursor(6, 26); d.print("Enter as lat,lon:");
        d.setTextColor(TFT_CYAN, TFT_BLACK); d.setCursor(6, 46); d.print(_posEdit); d.print('_');
        if (_posErr) { d.setTextColor(TFT_RED, TFT_BLACK); d.setCursor(6, 66); d.print("Invalid -- need lat,lon in range"); }
        else         { d.setTextColor(Theme::dim(d), TFT_BLACK); d.setCursor(6, 66); d.print("lat -90..90   lon -180..180"); }
        Theme::footer(d, "type  DEL erase  ENTER save  esc back");
    }
    // "lat,lon" -> validated doubles. Reuses toFloat() (not toDouble()) for
    // the same reason the rest of this codebase's coordinate parsing does --
    // see parseTrackInfo/TrackView::load -- float's ~7 significant digits
    // comfortably covers 6-decimal lat/lon at GPS-grade precision.
    static bool parseLatLon(const String &in, double &la, double &lo) {
        int c = in.indexOf(',');
        if (c < 0) return false;
        String a = in.substring(0, c), b = in.substring(c + 1);
        a.trim(); b.trim();
        if (a.length() == 0 || b.length() == 0) return false;
        double la2 = a.toFloat(), lo2 = b.toFloat();
        if (la2 < -90.0 || la2 > 90.0)   return false;
        if (lo2 < -180.0 || lo2 > 180.0) return false;
        la = la2; lo = lo2; return true;
    }
    void close(Settings &s, MapView &mv, Gps &gps, bool save) {
        if (save) {
            s.tzHours = _tz; s.baseDefault = _base; s.brightness = _bri;
            s.trackMinDistM = _trackM;
            s.hasDefaultPos = _hasDef; s.defLat = _defLat; s.defLon = _defLon;
            s.showWaypointsOnBoot = _showWpBoot;
            s.gpsSource    = _gpsSrc;
            s.gpsGroveBaud = GPS_BAUD_CHOICES[_gpsBaudIdx];
            s.save();
            // Apply the receiver change immediately -- no reboot needed.
            gps.reconfigure(s.gpsSource, s.gpsGroveBaud);
        }
        else M5Cardputer.Display.setBrightness(s.brightness);   // revert live preview
        _open = false; mv.markDirty();
    }
    bool   _open = false, _dirty = true;
    int    _row = 0, _top = 0, _base = 0, _bri = 180;
    int    _gpsSrc = GPS_SRC_DEFAULT, _gpsBaudIdx = 1;
    float  _tz = 0.0f, _trackM = (float)TRACK_MIN_DIST_M;
    bool   _hasDef = false;
    double _defLat = 0, _defLon = 0;
    bool   _showWpBoot = true;
    bool   _editingPos = false, _posErr = false;
    String _posEdit;
};

// =====================================================================
//  MODULE 10 - TRACK MANAGER UI  (browse / view / info / rename / delete)
// =====================================================================
struct TrackInfo {
    bool   ok = false; String name; int points = 0; double distKm = 0;
    double slat=0, slon=0, sele=0, elat=0, elon=0, eele=0;
    String stime, etime; long durSec = -1;
};

static long trackDurationSec(const String &a, const String &b) {
    struct tm ta{}, tb{};
    if (sscanf(a.c_str(), "%d-%d-%dT%d:%d:%dZ", &ta.tm_year,&ta.tm_mon,&ta.tm_mday,&ta.tm_hour,&ta.tm_min,&ta.tm_sec) != 6) return -1;
    if (sscanf(b.c_str(), "%d-%d-%dT%d:%d:%dZ", &tb.tm_year,&tb.tm_mon,&tb.tm_mday,&tb.tm_hour,&tb.tm_min,&tb.tm_sec) != 6) return -1;
    ta.tm_year-=1900; ta.tm_mon-=1; tb.tm_year-=1900; tb.tm_mon-=1;
    time_t x = mktime(&ta), y = mktime(&tb);
    if (x < 0 || y < 0) return -1;
    return (long)difftime(y, x);
}

static bool parseTrackInfo(const char *path, TrackInfo &ti) {
    ti = TrackInfo();
    File f = SD.open(path, FILE_READ);
    if (!f) return false;
    bool first = true, hasPrev = false; double plat=0, plon=0;
    while (f.available()) {
        String line = f.readStringUntil('\n');
        int a = line.indexOf("lat=\""), o = line.indexOf("lon=\"");
        if (a < 0 || o < 0) continue;
        double la = line.substring(a+5).toFloat();
        double lo = line.substring(o+5).toFloat();
        double ele = 0; int e = line.indexOf("<ele>"); if (e >= 0) ele = line.substring(e+5).toFloat();
        String tm; int t = line.indexOf("<time>"); if (t >= 0) { int t2 = line.indexOf("</time>", t); if (t2 > 0) tm = line.substring(t+6, t2); }
        if (first) { ti.slat=la; ti.slon=lo; ti.sele=ele; ti.stime=tm; first=false; }
        ti.elat=la; ti.elon=lo; ti.eele=ele; if (tm.length()) ti.etime=tm;
        if (hasPrev) ti.distKm += TinyGPSPlus::distanceBetween(plat,plon,la,lo)/1000.0;
        plat=la; plon=lo; hasPrev=true; ti.points++;
    }
    f.close();
    ti.name = path;
    ti.durSec = trackDurationSec(ti.stime, ti.etime);
    ti.ok = ti.points > 0;
    return ti.ok;
}

class TrackListUi {
public:
    bool active() const { return _open; }
    void open(MapView &mv) { _open = true; _state = LIST; _dirty = true; _sel = 0; _top = 0; scan(); mv.markDirty(); }

    void handleKey(const KeyState &st, TrackView &tv, MapView &mv) {
        switch (_state) {
        case LIST:
            for (char c : st.word) {
                if      (c == KEY_UP   && _sel > 0)                     _sel--;
                else if (c == KEY_DOWN && _sel < (int)_files.size()-1)  _sel++;
                else if (c == 'x')  { tv.clear(); mv.setTrack(&tv); _open = false; mv.markDirty(); return; } // Main map
                else if (c == KEY_BACK || c == KEY_TRACKS) { _open = false; mv.markDirty(); return; }
            }
            if (st.enter && !_files.empty()) { _asel = 0; _state = ACTIONS; }
            _dirty = true; break;

        case ACTIONS:
            for (char c : st.word) {
                if      (c == KEY_UP   && _asel > 0) _asel--;
                else if (c == KEY_DOWN && _asel < 4) _asel++;
                else if (c == KEY_BACK)                   { _state = LIST; }
            }
            if (st.enter) doAction(tv, mv);
            _dirty = true; break;

        case INFO:
            if (st.enter || st.del || !st.word.empty()) { _state = ACTIONS; _dirty = true; }
            break;

        case RENAME:
            if (st.del && _edit.length()) _edit.remove(_edit.length()-1);
            for (char c : st.word) {
                if (c == KEY_BACK) { _state = ACTIONS; _dirty = true; return; }
                if (c >= 32 && c < 127 && c != '/') _edit += c;
            }
            if (st.enter && _edit.length()) { doRename(tv, mv); }
            _dirty = true; break;

        case CONFIRM:
            for (char c : st.word) {
                if (c == 'y') { doDelete(tv, mv); return; }
                if (c == 'n' || c == KEY_BACK) { _state = ACTIONS; _dirty = true; return; }
            }
            break;
        }
    }

    void update(Gfx &d) {
        if (!_dirty) return; _dirty = false;
        switch (_state) {
        case LIST:    drawList(d); break;
        case ACTIONS: drawActions(d); break;
        case INFO:    drawInfo(d); break;
        case RENAME:  drawRename(d); break;
        case CONFIRM: drawConfirm(d); break;
        }
    }
private:
    const String& cur() const { return _files[_sel]; }
    String baseName(const String &path) const {
        String n = path; int sl = n.lastIndexOf('/'); if (sl >= 0) n = n.substring(sl+1); return n;
    }

    void doAction(TrackView &tv, MapView &mv) {
        switch (_asel) {
        case 0: // View on map
            if (tv.load(cur().c_str())) { mv.setTrack(&tv); mv.centerOn(tv.lat[0], tv.lon[0]); }
            _open = false; mv.markDirty(); break;
        case 1: parseTrackInfo(cur().c_str(), _info); _state = INFO; break;
        case 2: { String b = baseName(cur()); if (b.endsWith(".gpx")) b = b.substring(0, b.length()-4); _edit = b; _state = RENAME; break; }
        case 3: _state = CONFIRM; break;
        case 4: _state = LIST; break;
        }
    }
    void doRename(TrackView &tv, MapView &mv) {
        String np = String(TRACK_DIR) + "/" + _edit + ".gpx";
        String old = cur();
        if (SD.rename(old, np)) {
            if (tv.loaded && tv.name == old) { tv.name = np; }
        }
        scan(); if (_sel >= (int)_files.size()) _sel = _files.empty() ? 0 : _files.size()-1;
        _state = LIST; mv.markDirty();
    }
    void doDelete(TrackView &tv, MapView &mv) {
        String old = cur();
        SD.remove(old);
        if (tv.loaded && tv.name == old) { tv.clear(); mv.setTrack(&tv); }
        scan(); if (_sel >= (int)_files.size()) _sel = _files.empty() ? 0 : _files.size()-1;
        _state = LIST; _dirty = true;
    }

    void header(Gfx &d, const char *t) { Theme::header(d, t); }
    void footer(Gfx &d, const char *h) { Theme::footer(d, h); }

    void drawList(Gfx &d) {
        header(d, "Saved tracks");
        if (_files.empty()) { d.setTextColor(TFT_ORANGE); d.setCursor(6,40); d.print("No tracks recorded yet"); }
        else {
            const int rows = 6, rowH = 15;
            if (_sel < _top) _top = _sel;
            if (_sel >= _top + rows) _top = _sel - rows + 1;
            for (int i = 0; i < rows && (_top+i) < (int)_files.size(); ++i) {
                int idx = _top+i, y = 16 + i*rowH; bool sd = (idx==_sel);
                if (sd) d.fillRect(0,y,SCREEN_W,rowH,d.color565(0,60,28));
                d.setTextColor(sd?TFT_WHITE:TFT_LIGHTGREY, sd?d.color565(0,60,28):TFT_BLACK);
                d.setCursor(6,y+3);
                String n = baseName(_files[idx]); if (n.length()>34) n = n.substring(0,34);
                d.print(n);
            }
        }
        footer(d, "ENTER open  x Main map  esc back");
    }
    void drawActions(Gfx &d) {
        header(d, baseName(cur()).c_str());
        const char *items[5] = { "View on map", "Info", "Rename", "Delete", "Back" };
        for (int i = 0; i < 5; ++i) {
            int y = 18 + i*16; bool sd = (i==_asel);
            if (sd) d.fillRect(0,y,SCREEN_W,16,d.color565(0,60,28));
            uint16_t col = (i==3) ? TFT_RED : (sd?TFT_WHITE:TFT_LIGHTGREY);
            d.setTextColor(col, sd?d.color565(0,60,28):TFT_BLACK);
            d.setCursor(10,y+4); d.print(items[i]);
        }
        footer(d, "up/down  ENTER select  esc back");
    }
    void drawInfo(Gfx &d) {
        header(d, "TRACK INFO");
        if (!_info.ok) { d.setTextColor(TFT_RED); d.setCursor(6,40); d.print("Could not read track"); footer(d,"any key back"); return; }
        const int lx = 6, vx = 92, y0 = 18, rh = 10;
        int row = 0;
        auto R = [&](const char *label, const char *val) {
            int y = y0 + row * rh;
            d.setTextSize(1);
            d.setTextColor(Theme::dim(d), TFT_BLACK); d.setCursor(lx, y); d.print(label);
            d.setTextColor(Theme::fg(d),  TFT_BLACK); d.setCursor(vx, y); d.print(val);
            d.drawFastHLine(0, y + rh - 1, SCREEN_W, d.color565(0,45,22));
            row++;
        };
        char b[28];
        snprintf(b, sizeof(b), "%d", _info.points);       R("Points",    b);
        snprintf(b, sizeof(b), "%.2f km", _info.distKm);  R("Distance",  b);
        if (_info.durSec >= 0) snprintf(b, sizeof(b), "%02ld:%02ld:%02ld", _info.durSec/3600, (_info.durSec%3600)/60, _info.durSec%60);
        else                   snprintf(b, sizeof(b), "--");
        R("Duration",  b);
        R("Start UTC", _info.stime.length() ? _info.stime.c_str() : "--");
        snprintf(b, sizeof(b), "%.5f", _info.slat);       R("Start lat", b);
        snprintf(b, sizeof(b), "%.5f", _info.slon);       R("Start lon", b);
        snprintf(b, sizeof(b), "%.0f m", _info.sele);     R("Start alt", b);
        R("End UTC",   _info.etime.length() ? _info.etime.c_str() : "--");
        snprintf(b, sizeof(b), "%.5f", _info.elat);       R("End lat",   b);
        snprintf(b, sizeof(b), "%.5f", _info.elon);       R("End lon",   b);
        snprintf(b, sizeof(b), "%.0f m", _info.eele);     R("End alt",   b);
        footer(d, "any key back");
    }
    void drawRename(Gfx &d) {
        header(d, "Rename track");
        d.setTextColor(TFT_WHITE,TFT_BLACK); d.setCursor(6,30); d.print("New name:");
        d.setTextColor(TFT_CYAN,TFT_BLACK); d.setCursor(6,48); d.print(_edit); d.print('_');
        d.setTextColor(TFT_DARKGREY,TFT_BLACK); d.setCursor(6,66); d.print(".gpx added automatically");
        footer(d, "type  DEL erase  ENTER save  esc back");
    }
    void drawConfirm(Gfx &d) {
        header(d, "Delete track");
        d.setTextColor(TFT_WHITE,TFT_BLACK); d.setCursor(6,34); d.print(baseName(cur()));
        d.setTextColor(TFT_RED,TFT_BLACK); d.setCursor(6,56); d.print("Delete permanently?");
        footer(d, "y = yes    n = no");
    }
    void scan() {
        _files.clear();
        File dir = SD.open(TRACK_DIR);
        if (!dir) return;
        File e;
        while ((e = dir.openNextFile())) {
            if (!e.isDirectory()) {
                String n = e.name();
                if (n.endsWith(".gpx")) { if (n[0] != '/') n = String(TRACK_DIR) + "/" + n; _files.push_back(n); }
            }
            e.close();
            if ((int)_files.size() >= 64) break;
        }
        dir.close();
    }
    enum State { LIST, ACTIONS, INFO, RENAME, CONFIRM };
    State _state = LIST;
    bool  _open = false, _dirty = true;
    int   _sel = 0, _top = 0, _asel = 0;
    std::vector<String> _files;
    TrackInfo _info;
    String _edit;
};

// =====================================================================
//  MODULE 11 - UI OVERLAY  (composed onto the canvas)
// =====================================================================
namespace Ui {
    static void localTime(Gps &gps, const Settings &s, int &lh, int &lm) {
        int total = gps.hh()*60 + gps.mm() + (int)lround(s.tzHours*60.0);
        total = ((total % 1440) + 1440) % 1440;
        lh = total / 60; lm = total % 60;
    }
    template <class GFX>
    void drawStatusBar(GFX &g, Gps &gps, const MapView &mv, const Net &net, const Settings &s, const TrackRec &rec) {
        const int barH = 26, y0 = SCREEN_H - barH;
        g.fillRect(0, y0, SCREEN_W, barH, TFT_BLACK);
        g.drawFastHLine(0, y0, SCREEN_W, g.color565(50,50,60));
        g.setTextSize(1);

        // Row 1: time | altitude | direction
        g.setCursor(3, y0 + 3);
        if (gps.timeValid()) { int lh, lm; localTime(gps, s, lh, lm); g.setTextColor(TFT_WHITE, TFT_BLACK); g.printf("%02d:%02d", lh, lm); }
        else                 { g.setTextColor(TFT_DARKGREY, TFT_BLACK); g.print("--:--"); }
        g.setTextColor(TFT_LIGHTGREY, TFT_BLACK); g.setCursor(48, y0 + 3);
        if (gps.hasFix()) g.printf("%4.0fm", gps.altitude()); else g.print("  --m");
        g.setCursor(104, y0 + 3);
        if (gps.courseValid() && gps.speedKmph() >= SPEED_MIN_KMPH) { g.setTextColor(TFT_CYAN, TFT_BLACK); g.printf("dir %s", gps.cardinal()); }
        else { g.setTextColor(TFT_DARKGREY, TFT_BLACK); g.print("dir --"); }
        if (rec.recording()) {
            g.setTextColor(rec.paused() ? TFT_ORANGE : TFT_RED, TFT_BLACK);
            g.setCursor(SCREEN_W-30, y0+3);
            g.printf("%s%d", rec.paused() ? "PAU" : "REC", rec.count());
        }

        // Row 2: crosshair lat,lon | sats | zoom (+ tile x,y when it fits) | wifi
        g.setCursor(3, y0 + 15);
        g.setTextColor(gps.hasFix()?TFT_GREEN:TFT_ORANGE, TFT_BLACK);
        g.printf("%9.5f,%9.5f", mv.centerLat(), mv.centerLon());
        // "%9.5f,%9.5f" is always exactly 19 chars (sign+2+dot+5, twice, plus
        // the comma), so the left text's right edge is a fixed pixel offset --
        // no need to measure it, just compute the room left for the right side.
        const int leftEndX = 3 + 19*6;
        const char *wifi = !net.enabled() ? "--" : (net.online() ? "W" : ".");
        int tx, ty; mv.centerTileXY(tx, ty);
        char withTiles[40], short_[28];
        snprintf(withTiles, sizeof(withTiles), "%s Z%d %d,%d S%02lu %s", mv.baseTag(), mv.zoom(),
                 tx, ty, (unsigned long)gps.sats(), wifi);
        snprintf(short_, sizeof(short_), "%s Z%d S%02lu %s", mv.baseTag(), mv.zoom(),
                 (unsigned long)gps.sats(), wifi);
        // Tile numbers grow with zoom (up to 6 digits each near z19), so this
        // falls back to the plain zoom/sats/wifi line rather than let long
        // digit strings run into the lat/lon on the left.
        const char *right = withTiles;
        if (leftEndX + 6 + (int)strlen(withTiles)*6 + 3 > SCREEN_W) right = short_;
        g.setTextColor(TFT_DARKGREY, TFT_BLACK);
        g.setCursor(SCREEN_W - (int)strlen(right)*6 - 3, y0 + 15); g.print(right);
    }
    void splash(Gfx &d, const char *msg) {
        d.fillScreen(TFT_BLACK); d.setTextColor(TFT_WHITE, TFT_BLACK); d.setTextSize(1);
        d.setTextDatum(middle_center); d.drawString(msg, SCREEN_W/2, SCREEN_H/2); d.setTextDatum(top_left);
    }
    void drawHelp(Gfx &d, int scroll) {
        Theme::header(d, "HELP  -  KEYS");
        if (scroll < 0) scroll = 0;
        // Size 2 for readability. Every HELP_LINES entry is kept under 18
        // chars (checked above) so the widest one -- "c  Get center tile"
        // at 18*12=216px -- still clears the 240px screen with margin to
        // spare. rh=19 leaves a real gap below each 16px-tall glyph row
        // instead of crowding them; only 5 rows fit at this size, so the
        // list scrolls (below) -- readable beats cramming more per screen.
        const int y0 = 20, rh = 19;
        d.setTextSize(2);
        for (int i = 0; i < HELP_ROWS && (scroll + i) < HELP_COUNT; ++i) {
            d.setTextColor(Theme::fg(d), TFT_BLACK);
            d.setCursor(6, y0 + i * rh); d.print(HELP_LINES[scroll + i]);
        }
        d.setTextColor(Theme::dim(d), TFT_BLACK);
        if (scroll > 0)                      { d.setCursor(SCREEN_W - 10, y0 - 2);         d.print("^"); }
        if (scroll + HELP_ROWS < HELP_COUNT) { d.setCursor(SCREEN_W - 10, y0 + HELP_ROWS*rh); d.print("v"); }
        Theme::footer(d, "up/down scroll   h close");
    }
}

// =====================================================================
//  MODULE 10b - WAYPOINT UI  (pick icon -> type note -> save at crosshair)
// =====================================================================
class WaypointUi {
public:
    bool active() const { return _st != HIDDEN; }
    void open(MapView &mv, Gps &gps) {
        _st = ICON; _sel = 0; _note = ""; _dirty = true;
        _lat = mv.centerLat(); _lon = mv.centerLon();
        if (gps.dateValid() && gps.timeValid()) {
            char b[24]; snprintf(b, sizeof(b), "%04d-%02d-%02dT%02d:%02d:%02dZ",
                gps.yy(), gps.mo(), gps.dd(), gps.hh(), gps.mm(), gps.ss());
            _time = b;
        } else _time = "";
    }
    void handleKey(const KeyState &st, WaypointStore &ws, MapView &mv) {
        if (_st == ICON) {
            for (char c : st.word) {
                if      (c == KEY_LEFT  && _sel > 0)              _sel--;
                else if (c == KEY_RIGHT && _sel < ICON_COUNT - 1) _sel++;
                else if (c == KEY_UP    && _sel >= 6)             _sel -= 6;
                else if (c == KEY_DOWN  && _sel + 6 < ICON_COUNT) _sel += 6;
                else if (c == KEY_BACK)                               { _st = HIDDEN; mv.markDirty(); return; }
            }
            if (st.enter) _st = NOTE;
            _dirty = true;
        } else if (_st == NOTE) {
            if (st.del && _note.length()) _note.remove(_note.length() - 1);
            for (char c : st.word) {
                if (c == KEY_BACK) { _st = ICON; break; }
                if (c >= 32 && c < 127 && c != ',') _note += c;
            }
            if (st.enter) {
                Waypoint w; w.lat = _lat; w.lon = _lon; w.icon = _sel; w.time = _time; w.note = _note;
                ws.add(w); _st = HIDDEN; mv.markDirty(); return;
            }
            _dirty = true;
        }
    }
    void update(Gfx &d) {
        if (!_dirty) return; _dirty = false;
        if (_st == ICON) drawIcons(d); else if (_st == NOTE) drawNote(d);
    }
private:
    void drawIcons(Gfx &d) {
        Theme::header(d, "TAG - PICK ICON");
        for (int i = 0; i < ICON_COUNT; ++i) {
            int x = 20 + (i % 6) * 40, y = 44 + (i / 6) * 42;
            if (i == _sel) d.drawRoundRect(x-15, y-15, 30, 30, 4, Theme::fg(d));
            drawWpIcon(d, x, y, i);
        }
        d.setTextColor(Theme::fg(d), TFT_BLACK); d.setTextSize(1); d.setTextDatum(middle_center);
        d.drawString(ICONS[_sel].name, SCREEN_W/2, 118); d.setTextDatum(top_left);
        Theme::footer(d, "arrows pick  ENTER note  esc cancel");
    }
    void drawNote(Gfx &d) {
        char h[24]; snprintf(h, sizeof(h), "NOTE - %s", ICONS[_sel].name); Theme::header(d, h);
        drawWpIcon(d, 16, 46, _sel);
        d.setTextColor(Theme::fg(d), TFT_BLACK); d.setTextSize(1); d.setCursor(34, 42); d.print(_note); d.print('_');
        d.setTextColor(Theme::dim(d), TFT_BLACK); d.setCursor(6, 66); d.printf("%.5f, %.5f", _lat, _lon);
        Theme::footer(d, "type  DEL erase  ENTER save  esc back");
    }
    enum St { HIDDEN, ICON, NOTE };
    St _st = HIDDEN; int _sel = 0; bool _dirty = true;
    double _lat = 0, _lon = 0; String _time, _note;
};

// =====================================================================
//  MODULE 10c - NOTES UI  (browse / search / filter / sort / view /
//  rename / delete -- mirrors TrackListUi's LIST -> ACTIONS ->
//  {INFO,RENAME,CONFIRM} flow so the controls feel the same as the
//  track list, plus a SEARCH state on top)
//
//  Filtering and sorting never touch WaypointStore or the SD file: a
//  _view vector of store indices is rebuilt on demand, so waypoints.csv
//  keeps its original insertion order and "Saved" sort is free.
// =====================================================================
class NotesUi {
public:
    enum Sort { SORT_SAVED = 0, SORT_NEWEST, SORT_OLDEST, SORT_NEAREST, SORT_COUNT };
    static const char *sortName(int s) {
        switch (s) { case SORT_NEWEST: return "newest";
                     case SORT_OLDEST: return "oldest";
                     case SORT_NEAREST: return "nearest";
                     default: return "saved"; }
    }

    bool active() const { return _open; }
    // refLat/refLon anchor the "nearest" sort -- the caller passes the live
    // GPS fix when there is one, else the map centre (refIsGps says which,
    // purely so the header can be honest about it).
    void open(MapView &mv, WaypointStore &ws, double refLat, double refLon, bool refIsGps) {
        _open = true; _dirty = true; _sel = 0; _top = 0; _pickMode = false; _state = LIST;
        _refLat = refLat; _refLon = refLon; _refIsGps = refIsGps;
        rebuild(ws);
        mv.markDirty();
    }
    // Same list, but ENTER hands the picked waypoint's coords back via
    // consumePicked() instead of opening the actions menu -- used by the
    // distance calculator to pick a waypoint as one of its points. Search,
    // filter and sort all still work here.
    void openForPick(MapView &mv, WaypointStore &ws, double refLat, double refLon, bool refIsGps) {
        open(mv, ws, refLat, refLon, refIsGps); _pickMode = true;
    }
    bool consumePicked(double &lat, double &lon) {
        if (!_picked) return false;
        _picked = false; lat = _pickedLat; lon = _pickedLon; return true;
    }

    void handleKey(const KeyState &st, WaypointStore &ws, MapView &mv) {
        if (_state == SEARCH) {                       // live-filtering text field
            if (st.del && _query.length()) { _query.remove(_query.length()-1); rebuild(ws); }
            for (char c : st.word) {
                if (c == KEY_BACK) { _query = ""; rebuild(ws); _state = LIST; _dirty = true; return; }
                if (c >= 32 && c < 127) { _query += c; rebuild(ws); }
            }
            if (st.enter) _state = LIST;              // keep the query, go back to the list
            _dirty = true; return;
        }

        if (_pickMode) {
            for (char c : st.word) {
                if      (c == KEY_UP   && _sel > 0)              _sel--;
                else if (c == KEY_DOWN && _sel < viewCount()-1)  _sel++;
                else if (c == KEY_WP_FIND)  { _state = SEARCH; _dirty = true; return; }
                else if (c == KEY_WP_ICON)  { cycleIcon(ws); }
                else if (c == KEY_WP_SORT)  { cycleSort(ws); }
                else if (c == KEY_WP_CLEAR) { clearFilters(ws); }
                else if (c == KEY_BACK || c == KEY_NOTES) { _open = false; mv.markDirty(); return; }
            }
            if (st.enter && viewCount() > 0) {
                const Waypoint &w = ws.get(realIdx());
                _picked = true; _pickedLat = w.lat; _pickedLon = w.lon;
                _open = false; mv.markDirty(); return;
            }
            _dirty = true; return;
        }

        switch (_state) {
        case LIST:
            for (char c : st.word) {
                if      (c == KEY_UP   && _sel > 0)              _sel--;
                else if (c == KEY_DOWN && _sel < viewCount()-1)  _sel++;
                else if (c == KEY_WP_FIND)  { _state = SEARCH; _dirty = true; return; }
                else if (c == KEY_WP_ICON)  { cycleIcon(ws); }
                else if (c == KEY_WP_SORT)  { cycleSort(ws); }
                else if (c == KEY_WP_CLEAR) { clearFilters(ws); }
                else if (c == KEY_BACK || c == KEY_NOTES) { _open = false; mv.markDirty(); return; }
            }
            if (st.enter && viewCount() > 0) { _asel = 0; _state = ACTIONS; }
            _dirty = true; break;

        case ACTIONS:
            for (char c : st.word) {
                if      (c == KEY_UP   && _asel > 0) _asel--;
                else if (c == KEY_DOWN && _asel < 4) _asel++;
                else if (c == KEY_BACK)              { _state = LIST; }
            }
            if (st.enter) doAction(ws, mv);
            _dirty = true; break;

        case INFO:
            if (st.enter || st.del || !st.word.empty()) { _state = ACTIONS; _dirty = true; }
            break;

        case RENAME:
            if (st.del && _edit.length()) _edit.remove(_edit.length()-1);
            for (char c : st.word) {
                if (c == KEY_BACK) { _state = ACTIONS; _dirty = true; return; }
                if (c >= 32 && c < 127 && c != ',') _edit += c;
            }
            if (st.enter) { ws.rename(realIdx(), _edit); rebuild(ws); _state = LIST; }
            _dirty = true; break;

        case CONFIRM:
            for (char c : st.word) {
                if (c == 'y') { doDelete(ws); return; }
                if (c == 'n' || c == KEY_BACK) { _state = ACTIONS; _dirty = true; return; }
            }
            break;

        default: break;
        }
    }

    void update(Gfx &d, WaypointStore &ws) {
        if (!_dirty) return; _dirty = false;
        if (_state == SEARCH) { drawSearch(d, ws); return; }
        if (_pickMode) { drawList(d, ws); return; }
        switch (_state) {
        case LIST:    drawList(d, ws); break;
        case ACTIONS: drawActions(d, ws); break;
        case INFO:    drawInfo(d, ws); break;
        case RENAME:  drawRename(d); break;
        case CONFIRM: drawConfirm(d, ws); break;
        default: break;
        }
    }

private:
    // ---- view (filter + sort) -------------------------------------------
    int  viewCount() const { return (int)_view.size(); }
    int  realIdx()   const { return (_sel >= 0 && _sel < (int)_view.size()) ? _view[_sel] : 0; }
    bool filtersActive() const { return _query.length() > 0 || _iconFilter >= 0 || _sort != SORT_SAVED; }

    static String lower(const String &s) { String t = s; t.toLowerCase(); return t; }

    // Case-insensitive substring match against the note text. Empty query
    // matches everything, so the unfiltered list costs one strcmp-free pass.
    bool matches(const Waypoint &w) const {
        if (_iconFilter >= 0 && w.icon != _iconFilter) return false;
        if (_query.length() == 0) return true;
        return lower(w.note).indexOf(lower(_query)) >= 0;
    }

    void rebuild(WaypointStore &ws) {
        int prevReal = (_sel >= 0 && _sel < (int)_view.size()) ? _view[_sel] : -1;
        _view.clear();
        for (int i = 0; i < ws.count(); ++i)
            if (matches(ws.get(i))) _view.push_back((uint16_t)i);

        if (_sort != SORT_SAVED) {
            std::vector<uint16_t> &v = _view;
            const int      mode = _sort;
            const double   rla  = _refLat, rlo = _refLon;
            std::sort(v.begin(), v.end(), [&](uint16_t a, uint16_t b) {
                const Waypoint &wa = ws.get(a), &wb = ws.get(b);
                if (mode == SORT_NEAREST) {
                    double da = TinyGPSPlus::distanceBetween(rla, rlo, wa.lat, wa.lon);
                    double db = TinyGPSPlus::distanceBetween(rla, rlo, wb.lat, wb.lon);
                    if (da != db) return da < db;
                    return a < b;
                }
                // ISO-8601 timestamps compare correctly as plain strings.
                // Waypoints saved without a GPS clock have "" or "-"; keep
                // those at the bottom under either time sort rather than
                // letting them masquerade as the oldest entries.
                bool ea = (wa.time.length() == 0 || wa.time == "-");
                bool eb = (wb.time.length() == 0 || wb.time == "-");
                if (ea != eb) return !ea;
                if (ea && eb) return a < b;
                if (wa.time == wb.time) return a < b;
                return (mode == SORT_NEWEST) ? (wa.time > wb.time) : (wa.time < wb.time);
            });
        }

        // Keep the cursor on the same waypoint across a rebuild when it
        // survived the filter; otherwise clamp it into range.
        _sel = 0;
        if (prevReal >= 0)
            for (int i = 0; i < (int)_view.size(); ++i) if (_view[i] == prevReal) { _sel = i; break; }
        if (_sel >= (int)_view.size()) _sel = _view.empty() ? 0 : (int)_view.size() - 1;
        _top = 0; _dirty = true;
    }

    void cycleIcon(WaypointStore &ws)  { _iconFilter = (_iconFilter + 2 > ICON_COUNT) ? -1 : _iconFilter + 1; rebuild(ws); }
    void cycleSort(WaypointStore &ws)  { _sort = (_sort + 1) % SORT_COUNT; rebuild(ws); }
    void clearFilters(WaypointStore &ws) { _query = ""; _iconFilter = -1; _sort = SORT_SAVED; rebuild(ws); }

    // ---- actions ---------------------------------------------------------
    void doAction(WaypointStore &ws, MapView &mv) {
        switch (_asel) {
        case 0: { // Go to
            int ri = realIdx();
            const Waypoint &w = ws.get(ri);
            mv.centerOn(w.lat, w.lon); mv.setSolo(ri);
            _open = false; mv.markDirty(); break;
        }
        case 1: _state = INFO; break;
        case 2: _edit = ws.get(realIdx()).note; _state = RENAME; break;
        case 3: _state = CONFIRM; break;
        case 4: _state = LIST; break;
        }
    }
    void doDelete(WaypointStore &ws) {
        int keep = _sel;
        ws.remove(realIdx());
        _sel = 0; _view.clear();            // force rebuild() to re-scan from scratch
        rebuild(ws);
        if (keep >= (int)_view.size()) keep = _view.empty() ? 0 : (int)_view.size() - 1;
        _sel = keep;
        _state = LIST; _dirty = true;
    }

    // ---- drawing ---------------------------------------------------------
    // "1.2k" / "840m" -- short enough to sit at the right edge of a row.
    static void fmtDist(char *out, size_t n, double metres) {
        if (metres < 1000.0) snprintf(out, n, "%dm", (int)(metres + 0.5));
        else                 snprintf(out, n, "%.1fk", metres / 1000.0);
    }

    void drawFilterBar(Gfx &d, WaypointStore &ws) {
        char b[52];
        String q;
        if (_query.length()) { q = "\""; q += _query; q += "\""; }
        const char *ic = (_iconFilter >= 0) ? ICONS[_iconFilter].name : "";
        snprintf(b, sizeof(b), "%d/%d %s%s%s%s", viewCount(), ws.count(),
                 sortName(_sort), q.length() ? " " : "", q.c_str(), ic);
        d.setTextColor(Theme::fg(d), TFT_BLACK); d.setTextSize(1);
        d.setCursor(4, 19); d.print(b);
        if (_iconFilter >= 0) drawWpIcon(d, SCREEN_W - 14, 24, _iconFilter);
    }

    void drawList(Gfx &d, WaypointStore &ws) {
        Theme::header(d, _pickMode ? "PICK A WAYPOINT" : "WAYPOINTS / NOTES");
        const bool bar  = filtersActive();
        const int  y0   = bar ? 30 : 18;
        const int  rowH = 16;
        const int  rows = bar ? 5 : 6;
        if (bar) drawFilterBar(d, ws);

        if (ws.count() == 0) {
            d.setTextColor(TFT_ORANGE); d.setCursor(6, 40); d.print("No waypoints yet (press n)");
        } else if (viewCount() == 0) {
            d.setTextColor(TFT_ORANGE); d.setCursor(6, 52); d.print("No match -- x clears filters");
        } else {
            if (_sel < _top) _top = _sel;
            if (_sel >= _top + rows) _top = _sel - rows + 1;
            for (int i = 0; i < rows && (_top+i) < viewCount(); ++i) {
                int vi = _top+i, y = y0 + i*rowH; bool sd = (vi == _sel);
                if (sd) d.fillRect(0, y, SCREEN_W, rowH, Theme::hl(d));
                const Waypoint &w = ws.get(_view[vi]);
                drawWpIcon(d, 12, y+7, w.icon);
                d.setTextColor(sd?Theme::fg(d):Theme::dim(d), sd?Theme::hl(d):TFT_BLACK);

                // Nearest sort earns its keep only if the distance is visible,
                // so that mode trades ~6 characters of note for the number.
                char dist[10] = {0};
                int  maxChars = 30;
                if (_sort == SORT_NEAREST) {
                    fmtDist(dist, sizeof(dist), TinyGPSPlus::distanceBetween(_refLat, _refLon, w.lat, w.lon));
                    maxChars = 24;
                }
                d.setCursor(28, y+4);
                String t = w.note.length() ? w.note : String("(no note)");
                if ((int)t.length() > maxChars) t = t.substring(0, maxChars);
                d.print(t);
                if (dist[0]) { d.setCursor(SCREEN_W - 6 - (int)strlen(dist)*6, y+4); d.print(dist); }
            }
        }
        // The footer is one 40-char line at size 1 (240px / 6px per char, less
        // the 4px margin), so this lists the keys without their verbs; 'x'
        // (clear) lives in the help screen and in the no-match hint above.
        char f[48];
        snprintf(f, sizeof(f), "ENTER  f find  i icon  o sort  %s", KEY_BACK_NAME);
        Theme::footer(d, f);
    }

    void drawSearch(Gfx &d, WaypointStore &ws) {
        Theme::header(d, "FIND WAYPOINT");
        d.setTextColor(TFT_WHITE, TFT_BLACK); d.setCursor(6, 26); d.print("Search notes:");
        d.setTextColor(TFT_CYAN,  TFT_BLACK); d.setCursor(6, 44); d.print(_query); d.print('_');
        d.setTextColor(Theme::fg(d), TFT_BLACK); d.setCursor(6, 66);
        d.printf("%d of %d match", viewCount(), ws.count());
        // Preview the top hit so it's obvious the filter is live.
        if (viewCount() > 0) {
            const Waypoint &w = ws.get(_view[0]);
            drawWpIcon(d, 12, 92, w.icon);
            d.setTextColor(Theme::dim(d), TFT_BLACK); d.setCursor(28, 88);
            String t = w.note.length() ? w.note : String("(no note)");
            if (t.length() > 30) t = t.substring(0, 30);
            d.print(t);
        }
        char f[52];
        snprintf(f, sizeof(f), "type  DEL erase  ENTER list  %s clear", KEY_BACK_NAME);
        Theme::footer(d, f);
    }

    void drawActions(Gfx &d, WaypointStore &ws) {
        const Waypoint &w = ws.get(realIdx());
        String t = w.note.length() ? w.note : String("(no note)");
        Theme::header(d, t.c_str());
        const char *items[5] = { "Go to", "Info", "Rename", "Delete", "Back" };
        for (int i = 0; i < 5; ++i) {
            int y = 18 + i*16; bool sd = (i==_asel);
            if (sd) d.fillRect(0,y,SCREEN_W,16,Theme::hl(d));
            uint16_t col = (i==3) ? TFT_RED : (sd?Theme::fg(d):Theme::dim(d));
            d.setTextColor(col, sd?Theme::hl(d):TFT_BLACK);
            d.setCursor(10,y+4); d.print(items[i]);
        }
        char f[44]; snprintf(f, sizeof(f), "up/down  ENTER select  %s back", KEY_BACK_NAME);
        Theme::footer(d, f);
    }
    void drawInfo(Gfx &d, WaypointStore &ws) {
        const Waypoint &w = ws.get(realIdx());
        Theme::header(d, "WAYPOINT INFO");
        const int lx = 6, vx = 92, y0 = 20, rh = 14;
        int row = 0;
        auto R = [&](const char *label, const char *val) {
            int y = y0 + row * rh;
            d.setTextSize(1);
            d.setTextColor(Theme::dim(d), TFT_BLACK); d.setCursor(lx, y); d.print(label);
            d.setTextColor(Theme::fg(d),  TFT_BLACK); d.setCursor(vx, y); d.print(val);
            row++;
        };
        R("Name", w.note.length() ? w.note.c_str() : "(no note)");
        R("Icon", ICONS[(w.icon >= 0 && w.icon < ICON_COUNT) ? w.icon : 0].name);
        char b[24];
        snprintf(b, sizeof(b), "%.6f", w.lat); R("Lat", b);
        snprintf(b, sizeof(b), "%.6f", w.lon); R("Lon", b);
        R("Created UTC", w.time.length() ? w.time.c_str() : "--");
        char db[10]; fmtDist(db, sizeof(db), TinyGPSPlus::distanceBetween(_refLat, _refLon, w.lat, w.lon));
        char dl[24]; snprintf(dl, sizeof(dl), "%s %s", db, _refIsGps ? "(GPS)" : "(view)");
        R("Distance", dl);
        Theme::footer(d, "any key back");
    }
    void drawRename(Gfx &d) {
        Theme::header(d, "Rename waypoint");
        d.setTextColor(TFT_WHITE,TFT_BLACK); d.setCursor(6,30); d.print("New name:");
        d.setTextColor(TFT_CYAN,TFT_BLACK); d.setCursor(6,48); d.print(_edit); d.print('_');
        char f[48]; snprintf(f, sizeof(f), "type  DEL erase  ENTER save  %s back", KEY_BACK_NAME);
        Theme::footer(d, f);
    }
    void drawConfirm(Gfx &d, WaypointStore &ws) {
        const Waypoint &w = ws.get(realIdx());
        Theme::header(d, "Delete waypoint");
        d.setTextColor(TFT_WHITE,TFT_BLACK); d.setCursor(6,34); d.print(w.note.length() ? w.note : String("(no note)"));
        d.setTextColor(TFT_RED,TFT_BLACK); d.setCursor(6,56); d.print("Delete permanently?");
        Theme::footer(d, "y = yes    n = no");
    }

    enum State { LIST, ACTIONS, INFO, RENAME, CONFIRM, SEARCH };
    State _state = LIST;
    bool  _open = false, _dirty = true; int _sel = 0, _top = 0, _asel = 0;
    bool  _pickMode = false, _picked = false;
    double _pickedLat = 0, _pickedLon = 0;
    String _edit;

    std::vector<uint16_t> _view;          // store indices after filter+sort
    String _query;                        // note substring, case-insensitive
    int    _iconFilter = -1;              // -1 = all icons
    int    _sort = SORT_SAVED;
    double _refLat = DEFAULT_LAT, _refLon = DEFAULT_LON;
    bool   _refIsGps = false;
};

// =====================================================================
//  MODULE 10c - DISTANCE CALCULATOR
//  '3' toggles; picks up to 10 points (crosshair or a saved waypoint via
//  '2'), draws a line through them on the map, and shows each segment's
//  distance plus the running total in km/mi. Storage is a fixed 10-entry
//  array of {lat,lon} doubles (160 bytes) -- no SD I/O, no vector, no
//  growth -- so heap impact is nil regardless of how many points are used.
// =====================================================================
class DistanceCalc {
public:
    static constexpr int MAX_PTS = 10;

    bool active()  const { return _active; }
    bool picking() const { return _active && !_finished && _n < MAX_PTS; }

    void toggle(MapView &mv) {
        _active = !_active;
        _n = 0; _finished = false;
        mv.markDirty();
    }
    void useCrosshair(MapView &mv) {
        if (!picking()) return;
        addPoint(mv.centerLat(), mv.centerLon());
        mv.markDirty();
    }
    void useWaypoint(MapView &mv, double lat, double lon) {
        if (!picking()) return;
        addPoint(lat, lon);
        mv.markDirty();
    }
    // Stop adding points early (needs >=2) and show the segment breakdown.
    void finish(MapView &mv) {
        if (_active && !_finished && _n >= 2) { _finished = true; mv.markDirty(); }
    }

    // Screen-space line through the picked points -- same drawing style as
    // the recorded-track line, just a different color so the two aren't
    // confused when both are visible at once.
    template <class GFX>
    void drawPath(GFX &g, int originX, int originY, int zoom) const {
        if (_n == 0) return;
        int px = 0, py = 0; bool have = false;
        uint16_t col = g.color565(255, 210, 60);
        for (int i = 0; i < _n; ++i) {
            int x = (int)lround(Slippy::lonToWorldPx(_pts[i].lon, zoom) - originX);
            int y = (int)lround(Slippy::latToWorldPx(_pts[i].lat, zoom) - originY);
            if (have) g.drawWideLine(px, py, x, y, 2.2f, col);
            g.fillCircle(x, y, 3, col); g.drawCircle(x, y, 3, TFT_BLACK);
            px = x; py = y; have = true;
        }
    }

    template <class GFX>
    void drawBanner(GFX &g) const {
        if (!_active) return;
        if (!_finished) {
            char msg[48];
            snprintf(msg, sizeof(msg), "PT %c: ENTER=crosshair 2=waypt%s",
                     (char)('A' + _n), _n >= 2 ? "  esc done" : "");
            g.fillRect(0, 0, SCREEN_W, 10, TFT_BLACK);
            g.setTextColor(Theme::fg(g), TFT_BLACK); g.setTextSize(1);
            g.setCursor(2, 1); g.print(msg);
            return;
        }
        // Result: one line per segment, then the total -- small font so up
        // to 9 segments (10 points) still fits without scrolling.
        const int rh = 9;
        int rows = (_n - 1) + 1;
        g.fillRect(0, 0, SCREEN_W, rows * rh + 2, TFT_BLACK);
        g.setTextSize(1);
        double totalKm = 0;
        int y = 1;
        for (int i = 0; i < _n - 1; ++i) {
            double km = haversineKm(_pts[i].lat, _pts[i].lon, _pts[i+1].lat, _pts[i+1].lon);
            totalKm += km;
            g.setTextColor(Theme::fg(g), TFT_BLACK); g.setCursor(2, y);
            g.printf("%c-%c %.2fkm/%.2fmi", (char)('A'+i), (char)('A'+i+1), km, km * 0.621371);
            y += rh;
        }
        g.setTextColor(TFT_YELLOW, TFT_BLACK); g.setCursor(2, y);
        g.printf("TOT %.2fkm/%.2fmi (3 exit)", totalKm, totalKm * 0.621371);
    }
private:
    struct Pt { double lat = 0, lon = 0; };
    void addPoint(double lat, double lon) {
        if (_n >= MAX_PTS) return;
        _pts[_n].lat = lat; _pts[_n].lon = lon; _n++;
        if (_n >= MAX_PTS) _finished = true;
    }
    static double haversineKm(double lat1, double lon1, double lat2, double lon2) {
        constexpr double R = 6371.0088; // mean Earth radius, km
        double dLat = (lat2 - lat1) * M_PI / 180.0;
        double dLon = (lon2 - lon1) * M_PI / 180.0;
        double a = sin(dLat/2)*sin(dLat/2)
                 + cos(lat1*M_PI/180.0)*cos(lat2*M_PI/180.0)*sin(dLon/2)*sin(dLon/2);
        return R * 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    }
    bool _active = false, _finished = false;
    int  _n = 0;
    Pt   _pts[MAX_PTS];
};

// =====================================================================
//  MODULE 10d - BULK DOWNLOAD UI  ('5')
//  Wizard: pick a basemap -> zoom (native levels only, no over-zoom
//  fakes) -> X1/X2/Y1/Y2 tile-index range -> confirm -> download, with a
//  progress bar and esc-to-cancel. Every numeric field is validated
//  against 0..(2^zoom-1) for the CHOSEN zoom before it's accepted, since
//  that range is exactly what "does this tile coordinate exist" means on
//  a slippy map -- there's no server round-trip needed to check it.
// =====================================================================
class BulkDownloadUi {
public:
    bool active() const { return _open; }

    void open(MapView &mv) {
        _open = true; _dirty = true; _state = BASEMAP;
        _wantDownload = false;
        _baseSel = mv.baseIndex();
        _zoomSel = mv.zoom();
        _centerLat = mv.centerLat(); _centerLon = mv.centerLon();
        clampZoom();
        _fieldErr = "";
    }

    void handleKey(const KeyState &st, MapView &mv) {
        switch (_state) {
        case BASEMAP:
            for (char c : st.word) {
                if      (c == KEY_LEFT)  { _baseSel = (_baseSel + BASEMAP_COUNT - 1) % BASEMAP_COUNT; clampZoom(); }
                else if (c == KEY_RIGHT) { _baseSel = (_baseSel + 1) % BASEMAP_COUNT; clampZoom(); }
                else if (c == KEY_BACK)  { close(mv); return; }
            }
            if (st.enter) { _state = ZOOM; _dirty = true; }
            _dirty = true; break;

        case ZOOM: {
            int mn = MAP_ZOOM_MIN, mx = BASEMAPS[_baseSel].maxNative;
            for (char c : st.word) {
                if      (c == KEY_LEFT  && _zoomSel > mn) _zoomSel--;
                else if (c == KEY_RIGHT && _zoomSel < mx) _zoomSel++;
                else if (c == KEY_BACK)                   { _state = BASEMAP; _dirty = true; return; }
            }
            if (st.enter) { enterField(X1); }
            _dirty = true; break;
        }

        case X1: case X2: case Y1: case Y2:
            if (st.del && _edit.length()) _edit.remove(_edit.length() - 1);
            for (char c : st.word) {
                if (c == KEY_BACK) { backFromField(); return; }
                if (c >= '0' && c <= '9' && _edit.length() < 7) _edit += c;
            }
            if (st.enter) { if (acceptField()) return; }
            _dirty = true; break;

        case CONFIRM:
            for (char c : st.word) {
                if (c == 'n' || c == KEY_BACK) { reopenField(Y2); return; }
            }
            if (st.enter && !tooBig()) { _wantDownload = true; _dirty = true; }
            break;

        case DONE:
            if (st.enter || st.del || !st.word.empty()) close(mv);
            break;

        default: break;   // DOWNLOADING: input ignored, runDownload() polls esc itself
        }
    }

    // Input::poll() checks this right after handleKey(); if true it wraps
    // runDownload() with the same free-buffers/connect-WiFi/restore-buffers
    // dance the 'd'/'D'/'c' keys already use, then clears the flag.
    bool consumeWantDownload() { bool w = _wantDownload; _wantDownload = false; return w; }

    // The actual network loop -- mirrors MapView::downloadVisible() (skip
    // tiles already on SD, abort after 8 consecutive failures or the time
    // cap, themed progress bar) but over an explicit X/Y range instead of
    // the current viewport, and pollable esc-cancel since a bulk batch can
    // run for minutes.
    void runDownload(Gfx &d, Net &net) {
        _state = DOWNLOADING; _dirty = false;
        const BaseMap &bm = BASEMAPS[_baseSel];
        // Guaranteed <= BULK_DL_MAX_TILES by the tooBig() gate in handleKey()
        // before _wantDownload is ever set, so the down-cast to int is safe.
        int total = (int)tileCount();
        _dlOk = 0; _dlFail = 0; _dlSkip = 0; int done = 0;
        if (!net.online()) { Notify::notice(d, net.enabled() ? "WIFI NOT CONNECTED" : "PRESS w FOR WIFI"); _state = CONFIRM; _dirty = true; return; }

        char m0[48]; snprintf(m0, sizeof(m0), "DL %s z%d  connecting...", bm.tag, _zoomSel);
        Notify::progress(d, m0, 0, total);
        uint32_t t0 = millis();
        int consecFails = 0; const char *err = nullptr; bool cancelled = false;

        for (int ty = _y1; ty <= _y2 && !err && !cancelled; ++ty) {
            for (int tx = _x1; tx <= _x2 && !err && !cancelled; ++tx) {
                char path[80];
                snprintf(path, sizeof(path), "%s/%d/%d/%d.png", bm.root, _zoomSel, tx, ty);
                if (SD.exists(path)) { _dlSkip++; }
                else if (net.downloadTile(bm.root, bm.url, _zoomSel, tx, ty, path)) { _dlOk++; consecFails = 0; }
                else { _dlFail++; consecFails++; }
                done++;
                char msg[48];
                snprintf(msg, sizeof(msg), "DL %s z%d %d/%d ok:%d", bm.tag, _zoomSel, done, total, _dlOk);
                Notify::progress(d, msg, done, total);
                delay(120);   // be gentle to donated tile servers, same pace as downloadVisible

                // Poll for esc without a full Input::poll() re-entry (which
                // would also try to act on this same keypress once we return
                // to the map) -- just enough to break the loop cleanly.
                M5Cardputer.update();
                if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed()) {
                    auto st = M5Cardputer.Keyboard.keysState();
                    for (char c : st.word) if (c == KEY_BACK) cancelled = true;
                }
                if (!net.online() || consecFails >= 8) err = "CONNECTION LOST";
                else if (millis() - t0 > 1800000UL)     err = "TIMED OUT (30 min)";
            }
        }
        _dlErr = cancelled ? "CANCELLED" : err;
        _state = DONE; _dirty = true;
    }

    void update(Gfx &d) {
        if (!_dirty) return; _dirty = false;
        switch (_state) {
        case BASEMAP: drawBasemap(d); break;
        case ZOOM:    drawZoom(d);    break;
        case X1: case X2: case Y1: case Y2: drawField(d); break;
        case CONFIRM: drawConfirm(d); break;
        case DONE:    drawDone(d);    break;
        default: break;
        }
    }

private:
    enum State { HIDDEN, BASEMAP, ZOOM, X1, X2, Y1, Y2, CONFIRM, DOWNLOADING, DONE };

    void close(MapView &mv) { _open = false; mv.markDirty(); }
    void clampZoom() {
        int mx = BASEMAPS[_baseSel].maxNative;
        if (_zoomSel < MAP_ZOOM_MIN) _zoomSel = MAP_ZOOM_MIN;
        if (_zoomSel > mx) _zoomSel = mx;
    }
    int maxIndex() const { return (1 << _zoomSel) - 1; }   // valid tile index range is [0, maxIndex]

    // Tile the wizard's reference point (the map centre at open() time)
    // falls in at the chosen zoom -- prefilled into X1/Y1 so a lone ENTER
    // starts a 1x1 box right where the user was looking; X2/Y2 default to
    // the same value too, extend from there.
    void suggestTile(int &tx, int &ty) const {
        int n = 1 << _zoomSel;
        tx = ((int)floor(Slippy::lonToWorldPx(_centerLon, _zoomSel) / TILE_SIZE) % n + n) % n;
        ty = (int)floor(Slippy::latToWorldPx(_centerLat, _zoomSel) / TILE_SIZE);
        if (ty < 0) ty = 0; if (ty > n - 1) ty = n - 1;
    }
    void enterField(State s) {
        _state = s; _fieldErr = "";
        int stx, sty; suggestTile(stx, sty);
        int def = (s == X1 || s == X2) ? stx : sty;
        if (s == X2) def = _x1; else if (s == Y2) def = _y1;   // X2/Y2 continue from X1/Y1, not the raw suggestion
        char b[8]; snprintf(b, sizeof(b), "%d", def);
        _edit = b; _dirty = true;
    }
    void backFromField() {
        switch (_state) {
        case X2: reopenField(X1); break;
        case Y1: reopenField(X2); break;
        case Y2: reopenField(Y1); break;
        default: _state = ZOOM; _fieldErr = ""; _dirty = true; break;   // X1 -> back to the ZOOM step
        }
    }
    // Re-enter a field that was already filled in (going backward from a
    // later step) -- prefill with what's already stored instead of the
    // suggestion, so backing up to fix one field doesn't lose the others.
    void reopenField(State s) {
        _state = s; _fieldErr = "";
        int v = (s == X1) ? _x1 : (s == X2) ? _x2 : (s == Y1) ? _y1 : _y2;
        char b[8]; snprintf(b, sizeof(b), "%d", v);
        _edit = b; _dirty = true;
    }
    // Returns true if it already handled navigation (so handleKey should
    // stop processing this keystroke); false means fall through to _dirty.
    bool acceptField() {
        int v = _edit.toInt();
        if (_edit.length() == 0 || v < 0 || v > maxIndex()) {
            char b[40]; snprintf(b, sizeof(b), "must be 0..%d at z%d", maxIndex(), _zoomSel);
            _fieldErr = b; _dirty = true; return true;
        }
        switch (_state) {
        case X1: _x1 = v; enterField(X2); return true;
        case X2: _x2 = v; enterField(Y1); return true;
        case Y1: _y1 = v; enterField(Y2); return true;
        case Y2:
            _y2 = v;
            if (_x1 > _x2) std::swap(_x1, _x2);   // friendlier than rejecting a reversed range
            if (_y1 > _y2) std::swap(_y1, _y2);
            _state = CONFIRM; _dirty = true; return true;
        default: return false;
        }
    }
    // Tile counts are validated per-axis against maxIndex() (up to ~5.2e5 at
    // z19), but the PRODUCT of two such axes can exceed INT32_MAX -- an
    // accidental huge box must not wrap around into a small/negative number
    // and slip past BULK_DL_MAX_TILES, so this adds in 64-bit.
    int64_t tileCount() const {
        return (int64_t)(_x2 - _x1 + 1) * (int64_t)(_y2 - _y1 + 1);
    }
    bool tooBig() const { return tileCount() > BULK_DL_MAX_TILES; }

    void drawBasemap(Gfx &d) {
        Theme::header(d, "BULK DOWNLOAD 1/6");
        d.setTextColor(TFT_WHITE, TFT_BLACK); d.setTextSize(1); d.setCursor(6, 30); d.print("Basemap:");
        for (int i = 0; i < BASEMAP_COUNT; ++i) {
            int y = 46 + i * 16; bool sel = (i == _baseSel);
            if (sel) d.fillRect(0, y, SCREEN_W, 16, Theme::hl(d));
            d.setTextColor(sel ? Theme::fg(d) : Theme::dim(d), sel ? Theme::hl(d) : TFT_BLACK);
            d.setCursor(10, y + 4); d.printf("%s  (native max z%d)", BASEMAPS[i].name, BASEMAPS[i].maxNative);
        }
        char f[40]; snprintf(f, sizeof(f), "left/right pick  ENTER next  %s cancel", KEY_BACK_NAME);
        Theme::footer(d, f);
    }
    void drawZoom(Gfx &d) {
        Theme::header(d, "BULK DOWNLOAD 2/6");
        d.setTextColor(TFT_WHITE, TFT_BLACK); d.setTextSize(1); d.setCursor(6, 28);
        d.printf("%s zoom level:", BASEMAPS[_baseSel].name);
        d.setTextColor(Theme::fg(d), TFT_BLACK); d.setTextSize(2); d.setCursor(6, 44);
        d.printf("Z%d", _zoomSel);
        int tx, ty; suggestTile(tx, ty);
        d.setTextSize(1); d.setTextColor(Theme::dim(d), TFT_BLACK); d.setCursor(6, 70);
        d.printf("view centre here = tile %d,%d", tx, ty);
        d.setCursor(6, 84); d.printf("valid range 0..%d", maxIndex());
        char f[48]; snprintf(f, sizeof(f), "left/right zoom  ENTER next  %s back", KEY_BACK_NAME);
        Theme::footer(d, f);
    }
    void drawField(Gfx &d) {
        static const char *kTitle[] = { "", "", "", "X1 (west edge)", "X2 (east edge)", "Y1 (north edge)", "Y2 (south edge)" };
        static const char *kStep[]  = { "", "", "", "3/6", "4/6", "5/6", "6/6" };
        char hdr[24]; snprintf(hdr, sizeof(hdr), "BULK DOWNLOAD %s", kStep[_state]);
        Theme::header(d, hdr);
        d.setTextColor(TFT_WHITE, TFT_BLACK); d.setTextSize(1); d.setCursor(6, 26); d.print(kTitle[_state]);
        d.setCursor(6, 40); d.printf("z%d valid range: 0..%d", _zoomSel, maxIndex());
        d.setTextColor(TFT_CYAN, TFT_BLACK); d.setTextSize(2); d.setCursor(6, 56); d.print(_edit); d.print('_');
        d.setTextSize(1);
        if (_fieldErr.length()) { d.setTextColor(TFT_RED, TFT_BLACK); d.setCursor(6, 84); d.print(_fieldErr); }
        char f[48]; snprintf(f, sizeof(f), "type  DEL erase  ENTER next  %s back", KEY_BACK_NAME);
        Theme::footer(d, f);
    }
    void drawConfirm(Gfx &d) {
        Theme::header(d, "BULK DOWNLOAD - CONFIRM");
        const BaseMap &bm = BASEMAPS[_baseSel];
        // This screen is exactly where an oversized box shows up, so the
        // count can't be a plain int here (see tileCount()'s comment) --
        // print via double/%.0f rather than depend on %lld support.
        double total = (double)tileCount();
        d.setTextSize(1); d.setTextColor(Theme::fg(d), TFT_BLACK);
        d.setCursor(6, 20); d.printf("%s  z%d", bm.name, _zoomSel);
        d.setCursor(6, 34); d.printf("x %d..%d   y %d..%d", _x1, _x2, _y1, _y2);
        bool big = tooBig();
        d.setTextColor(big ? TFT_RED : Theme::fg(d), TFT_BLACK);
        d.setCursor(6, 52); d.printf("%.0f tiles%s", total, big ? "  (too many!)" : "");
        if (big) {
            d.setTextColor(TFT_ORANGE, TFT_BLACK); d.setCursor(6, 68);
            d.printf("max %d -- narrow the range", BULK_DL_MAX_TILES);
            char f[40]; snprintf(f, sizeof(f), "%s / n back to Y2", KEY_BACK_NAME);
            Theme::footer(d, f);
        } else {
            d.setTextColor(Theme::dim(d), TFT_BLACK); d.setCursor(6, 68);
            d.print("existing tiles are skipped, not overwritten");
            char f[40]; snprintf(f, sizeof(f), "ENTER download  %s / n back", KEY_BACK_NAME);
            Theme::footer(d, f);
        }
    }
    void drawDone(Gfx &d) {
        Theme::header(d, _dlErr ? "BULK DOWNLOAD - STOPPED" : "BULK DOWNLOAD - DONE");
        d.setTextSize(1); d.setTextColor(Theme::fg(d), TFT_BLACK);
        d.setCursor(6, 30); d.printf("ok:%d  skipped:%d  failed:%d", _dlOk, _dlSkip, _dlFail);
        if (_dlErr) { d.setTextColor(TFT_ORANGE, TFT_BLACK); d.setCursor(6, 46); d.print(_dlErr); }
        Theme::footer(d, "any key: back to map");
    }

    bool  _open = false, _dirty = true, _wantDownload = false;
    State _state = HIDDEN;
    int   _baseSel = 0, _zoomSel = MAP_ZOOM;
    double _centerLat = DEFAULT_LAT, _centerLon = DEFAULT_LON;
    int   _x1 = 0, _x2 = 0, _y1 = 0, _y2 = 0;
    String _edit, _fieldErr;
    int   _dlOk = 0, _dlFail = 0, _dlSkip = 0;
    const char *_dlErr = nullptr;
};

// =====================================================================
//  BOOT SCREEN  (Fallout / Pip-Boy style, self-contained)
// =====================================================================
static void bootScreen(Gfx &d, bool sdOk, Gps &gps, MapView &gMap) {
    uint16_t grn = d.color565(120,255,150), dgr = d.color565(0,120,55), amb = d.color565(255,190,70);
    d.setTextDatum(top_left);
    d.fillScreen(TFT_BLACK);
    for (int y = 0; y < SCREEN_H; y += 3) d.drawFastHLine(0, y, SCREEN_W, d.color565(0,28,12));
    int cx = 40, cy = 52;
    d.fillCircle(cx, cy, 26, dgr); d.fillCircle(cx, cy, 22, TFT_BLACK);
    d.drawCircle(cx, cy, 26, grn); d.drawCircle(cx, cy, 17, grn); d.fillCircle(cx, cy, 4, grn);
    for (int a = 0; a < 360; a += 30) { float r = a * 0.01745f;
        d.drawLine(cx+cos(r)*8, cy+sin(r)*8, cx+cos(r)*25, cy+sin(r)*25, grn); }
    d.setTextColor(grn, TFT_BLACK); d.setTextSize(2); d.setCursor(76, 24); d.print("PIP-BOY");
    d.setTextSize(1); d.setTextColor(dgr, TFT_BLACK); d.setCursor(76, 44); d.print("MAP MODULE v3");
    char gpsLine[32];
    snprintf(gpsLine, sizeof(gpsLine), "> GPS XCVR....%s",
             gps.source() == GPS_SRC_GROVE ? "GROVE" : "CAP");
    const char *L[] = {
        "VAULT-TEC UNIFIED OS",
        "(C)2287 ROBCO / VAULT-TEC",
        "> DISPLAY........OK",
        sdOk ? "> HOLOTAPE(SD)...OK" : "> HOLOTAPE(SD)...FAIL",
        gpsLine
    };
    int y = 80; d.setTextSize(1);
    for (int i = 0; i < 5; ++i) {
        d.setTextColor((!sdOk && i == 3) ? d.color565(255,110,50) : grn, TFT_BLACK);
        const char *p = L[i]; int x = 6;
        while (*p) { char t[2] = { *p++, 0 }; d.drawString(t, x, y); x += 6; delay(7); }
        y += 9; delay(150);
    }
    delay(500);

    // Acquire a GPS fix so the map opens on YOUR location (not the default view).
    uint32_t t0 = millis(); int frame = 0; bool got = false;
    while (millis() - t0 < 25000UL) {
        gps.update();
        M5Cardputer.update();
        if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed()) break;
        if (gps.hasFix()) { gMap.onGpsFix(gps.lat(), gps.lon()); got = true; break; }
        d.fillRect(0, 100, SCREEN_W, 35, TFT_BLACK);
        d.setTextColor(amb, TFT_BLACK); d.setTextSize(1); d.setCursor(6, 104);
        d.print("ACQUIRING SATELLITE UPLINK");
        char dots[5] = {0}; for (int i = 0; i < (frame % 4); ++i) dots[i] = '.';
        d.setTextColor(grn, TFT_BLACK); d.setCursor(6, 116);
        d.printf("SATS:%02lu  RX:%lu%s   ", (unsigned long)gps.sats(), (unsigned long)gps.charsRx(), dots);
        d.setTextColor(dgr, TFT_BLACK); d.setCursor(6, 126); d.print("(press any key to skip)");
        frame++; delay(200);
    }
    d.fillRect(0, 100, SCREEN_W, 35, TFT_BLACK);
    d.setTextColor(got ? grn : amb, TFT_BLACK); d.setTextSize(1); d.setCursor(6, 112);
    d.print(got ? "> POSITION LOCKED" : "> NO FIX - DEFAULT VIEW");
    delay(800);
}

// =====================================================================
//  GLOBALS
// =====================================================================
Gps         gps;
Net         net;
Settings    settings;
MapView     gMap;
WiFiPortal  portal;
SettingsUi  settingsUi;
TrackListUi trackListUi;
WaypointUi  waypointUi;
NotesUi     notesUi;
DistanceCalc gDist;
BulkDownloadUi bulkUi;
TrackRec    recorder;
TrackView   track;
WaypointStore wstore;

M5Canvas    canvas(&M5Cardputer.Display);      // final frame compositor
M5Canvas    mapCanvas(&M5Cardputer.Display);   // persistent tile layer (scroll-blit)
bool        gHelp = false, gHelpDraw = false, gCanvasOk = false, gScrollOk = false;
int         gHelpScroll = 0;
bool        gDebug = false;
bool        gScreenOn = true;
uint32_t    tIdle = 0;

// Screen off: kill the backlight and skip all rendering (loop() still runs
// GPS parsing + track recording every iteration -- see loop() below -- only
// the display/compose work is skipped). Backlight is the dominant power draw
// on a small TFT, so this is the real lever for battery life on a long track.
static void sleepScreen() {
    gScreenOn = false;
    M5Cardputer.Display.setBrightness(0);
}
static void wakeScreen() {
    gScreenOn = true;
    M5Cardputer.Display.setBrightness(settings.brightness);
    gMap.markDirty();
}

// Free big buffers so a WiFi/TLS download has enough RAM, then restore them.
static void freeBigBuffers() {
    canvas.deleteSprite();
    mapCanvas.deleteSprite();
    gCanvasOk = false; gScrollOk = false;
}
// A big sprite can transiently fail right after a download batch, while the
// WiFi/BT driver is still releasing its internal buffers. A couple of short
// retries lets that settle instead of leaving the canvas permanently down
// (this board has no PSRAM escape valve, so contiguous 64KB blocks are tight).
static bool createSpriteRetry(M5Canvas &spr) {
    for (int attempt = 0; attempt < 3; ++attempt) {
        if (attempt) delay(60);
        if (spr.createSprite(SCREEN_W, SCREEN_H) != nullptr) return true;
    }
    return false;
}
static void allocBigBuffers() {
    // setPsram(true) prefers PSRAM for sprite storage and silently falls
    // back to internal RAM if none is present, so this is always safe.
    canvas.setColorDepth(16);    mapCanvas.setColorDepth(16);
    canvas.setPsram(true);       mapCanvas.setPsram(true);
    gCanvasOk = createSpriteRetry(canvas);
    gScrollOk = createSpriteRetry(mapCanvas);
    gMap.markDirty();
}

// =====================================================================
//  MODULE 12 - INPUT
// =====================================================================
namespace Input {
    void poll() {
        M5Cardputer.update();
        if (M5.BtnA.wasPressed()) { ESP.restart(); }   // BtnG0 = reboot
        if (!M5Cardputer.Keyboard.isChange())  return;
        if (!M5Cardputer.Keyboard.isPressed()) return;
        auto st = M5Cardputer.Keyboard.keysState();

        // Screen is off: any key just wakes it (this keypress isn't also
        // acted on -- avoids e.g. a pocket press waking AND panning/zooming
        // in the same motion). Press again once the screen is back on.
        if (!gScreenOn) { wakeScreen(); return; }

        if (portal.active())      { portal.handleKey(st, net, gMap); return; }
        if (settingsUi.active())  { settingsUi.handleKey(st, settings, gMap, gps); return; }
        if (trackListUi.active()) { trackListUi.handleKey(st, track, gMap); return; }
        if (waypointUi.active())  { waypointUi.handleKey(st, wstore, gMap); return; }
        if (notesUi.active())     { notesUi.handleKey(st, wstore, gMap); return; }
        if (bulkUi.active())      {
            bulkUi.handleKey(st, gMap);
            if (bulkUi.consumeWantDownload()) {
                freeBigBuffers();
                bool wasOnline = net.online();
                if (!wasOnline) net.ensureConnected(M5Cardputer.Display);
                bulkUi.runDownload(M5Cardputer.Display, net);
                if (!wasOnline) net.radioOff();
                allocBigBuffers();
            }
            return;
        }
        if (gHelp) {
            for (char c : st.word) {
                if      (c == KEY_HELP || c == KEY_BACK)                          { gHelp = false; gMap.markDirty(); return; }
                else if (c == KEY_UP   && gHelpScroll > 0)                   { gHelpScroll--; gHelpDraw = true; }
                else if (c == KEY_DOWN && gHelpScroll < HELP_COUNT-HELP_ROWS){ gHelpScroll++; gHelpDraw = true; }
            }
            return;
        }

        for (char c : st.word) {
            if      (c == KEY_UP)        gMap.pan(0, -PAN_STEP_PX);
            else if (c == KEY_DOWN)      gMap.pan(0,  PAN_STEP_PX);
            else if (c == KEY_LEFT)      gMap.pan(-PAN_STEP_PX, 0);
            else if (c == KEY_RIGHT)     gMap.pan( PAN_STEP_PX, 0);
            else if (c == KEY_RECENTER)  gMap.recenterOnGps();
            else if (c == KEY_ZOOM_IN  || c == '+') gMap.zoomIn();
            else if (c == KEY_ZOOM_OUT || c == '_') gMap.zoomOut();
            else if (c == KEY_DOWNLOAD || c == KEY_REFRESH) {
                bool force = (c == KEY_REFRESH);
                freeBigBuffers();                                     // free ~176KB for TLS
                bool wasOnline = net.online();
                if (!wasOnline) net.ensureConnected(M5Cardputer.Display);  // power radio + connect
                gMap.downloadVisible(M5Cardputer.Display, net, force);
                if (!wasOnline) net.radioOff();                       // radio back off to save RAM
                allocBigBuffers();                                    // restore frame/scroll/tile buffers
            }
            else if (c == KEY_CENTER_DL) {
                freeBigBuffers();
                bool wasOnline = net.online();
                if (!wasOnline) net.ensureConnected(M5Cardputer.Display);
                gMap.downloadCenterTile(M5Cardputer.Display, net);
                if (!wasOnline) net.radioOff();
                allocBigBuffers();
            }
            else if (c == KEY_WIFI)      portal.open();
            else if (c == KEY_SETTINGS)  settingsUi.open(settings);
            else if (c == KEY_BASEMAP)   gMap.nextBasemap();
            else if (c == KEY_TRACKS)    trackListUi.open(gMap);
            else if (c == KEY_RECORD)    { recorder.toggle(gps); gMap.markDirty(); }
            else if (c == KEY_PAUSE)     { recorder.togglePause(); gMap.markDirty(); }
            else if (c == KEY_TAG)       waypointUi.open(gMap, gps);
            else if (c == KEY_ICONS)     gMap.toggleIcons();
            else if (c == KEY_NOTES)     {
                // "Nearest" sorting anchors on the live GPS fix; with no fix
                // the map centre is the next most useful reference point.
                bool  fix = gps.hasFix();
                double rla = fix ? gps.lat() : gMap.centerLat();
                double rlo = fix ? gps.lon() : gMap.centerLon();
                if (gDist.picking()) notesUi.openForPick(gMap, wstore, rla, rlo, fix);
                else                 notesUi.open(gMap, wstore, rla, rlo, fix);
            }
            else if (c == KEY_DISTANCE)  gDist.toggle(gMap);
            else if (c == KEY_BACK && gDist.picking()) gDist.finish(gMap);   // done adding points early
            else if (c == KEY_BULK_DL)   bulkUi.open(gMap);
            else if (c == KEY_DEBUG)     { gDebug = !gDebug; gMap.markDirty(); }
            else if (c == KEY_SCREEN)    { sleepScreen(); return; }
            else if (c == KEY_HELP)      { gHelp = true; gHelpDraw = true; gHelpScroll = 0; }
        }
        if (st.enter && gDist.picking()) gDist.useCrosshair(gMap);
    }
}

// =====================================================================
//  SETUP / LOOP
// =====================================================================
static bool mountSd() {
    pinMode(LORA_NSS_PIN, OUTPUT);   // keep idle SX1262 de-selected on the shared bus
    digitalWrite(LORA_NSS_PIN, HIGH);
    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    if (!SD.begin(SD_CS_PIN, SPI, SD_SPI_HZ)) return false;
    // Everything else (settings/waypoints/wifi files, tracks/, the basemap
    // folders) lives under this one root. SD.mkdir() on most ESP32 SD
    // backends only creates a single level, so this has to exist before any
    // SD.open(..., FILE_WRITE) or per-basemap SD.mkdir() below can succeed --
    // harmless no-op if it's already there.
    SD.mkdir(MAP_ROOT);
    return true;
}

// If the device died mid-recording (battery, crash, reset) TrackRec::stop()
// never ran, so the file is missing its closing </trkseg></trk></gpx> tags.
// Every <trkpt> written before the crash is still intact and this app's own
// viewer/info parser doesn't care about the wrapper tags -- but strict GPX
// tools elsewhere (Garmin/Strava/QGIS/etc.) may reject an unclosed file. Heal
// any such file once at boot by appending the closing tags if they're missing.
static void healUnclosedTracks() {
    File dir = SD.open(TRACK_DIR);
    if (!dir) return;
    File e;
    while ((e = dir.openNextFile())) {
        if (!e.isDirectory()) {
            String n = e.name();
            if (n[0] != '/') n = String(TRACK_DIR) + "/" + n;
            size_t sz = e.size();
            bool closed = false;
            if (n.endsWith(".gpx") && sz > 0) {
                // Look for "</gpx>" anywhere in the tail rather than requiring
                // an exact trailing match -- stop() writes a trailing '\n'
                // after it, which would otherwise shift it out of a fixed
                // last-N-bytes window and false-negative on every clean file.
                size_t tailLen = sz < 32 ? sz : 32;
                char tail[33];
                e.seek(sz - tailLen);
                size_t rd = e.read((uint8_t *)tail, tailLen);
                tail[rd] = 0;
                closed = (strstr(tail, "</gpx>") != nullptr);
            }
            e.close();
            if (n.endsWith(".gpx") && sz > 0 && !closed) {
                File w = SD.open(n, FILE_APPEND);
                if (w) { w.print("</trkseg></trk></gpx>\n"); w.close(); }
            }
        } else e.close();
    }
    dir.close();
}

template <class G>
static void drawDebugHud(G &g) {
    int present = 0, total = 0; gMap.tilesPresent(present, total);
    g.fillRect(0, 17, 158, 34, TFT_BLACK); g.drawRect(0, 17, 158, 34, g.color565(0,120,55));
    g.setTextColor(g.color565(130,255,150), TFT_BLACK); g.setTextSize(1);
    g.setCursor(3, 20); g.printf("heap %uk blk %uk", (unsigned)(ESP.getFreeHeap()/1024), (unsigned)(ESP.getMaxAllocHeap()/1024));
    g.setCursor(3, 30); g.printf("cv%d sc%d", gCanvasOk?1:0, gScrollOk?1:0);
    g.setCursor(3, 40); g.printf("tiles %d/%d z%d %s", present, total, gMap.zoom(), gMap.baseTag());
}

static void composeAndPush() {
    if (gCanvasOk && gScrollOk && USE_SCROLL_BLIT) {
        gMap.updateMapLayer(mapCanvas);            // scroll + decode only new edges
        mapCanvas.pushSprite(&canvas, 0, 0);
        gMap.drawOverlays(canvas);
        gDist.drawPath(canvas, gMap.originX(), gMap.originY(), gMap.zoom());
        Ui::drawStatusBar(canvas, gps, gMap, net, settings, recorder);
        if (gDebug) drawDebugHud(canvas);
        gDist.drawBanner(canvas);
        canvas.pushSprite(0, 0);
    } else if (gCanvasOk) {
        gMap.drawMapFull(canvas);
        gMap.drawOverlays(canvas);
        gDist.drawPath(canvas, gMap.originX(), gMap.originY(), gMap.zoom());
        Ui::drawStatusBar(canvas, gps, gMap, net, settings, recorder);
        if (gDebug) drawDebugHud(canvas);
        gDist.drawBanner(canvas);
        canvas.pushSprite(0, 0);
    } else {
        auto &d = M5Cardputer.Display;
        d.startWrite();
        gMap.drawMapFull(d);
        gMap.drawOverlays(d);
        gDist.drawPath(d, gMap.originX(), gMap.originY(), gMap.zoom());
        Ui::drawStatusBar(d, gps, gMap, net, settings, recorder);
        if (gDebug) drawDebugHud(d);
        gDist.drawBanner(d);
        d.endWrite();
    }
}

void setup() {
    auto cfg = M5.config();
    M5Cardputer.begin(cfg, true);
    auto &d = M5Cardputer.Display;
    d.setRotation(SCREEN_ROTATION);
    d.setColorDepth(16);
    d.fillScreen(TFT_BLACK);

    bool sdOk = mountSd();
    if (sdOk) healUnclosedTracks();
    settings.load();
    d.setBrightness(settings.brightness);
    gps.begin(settings.gpsSource, settings.gpsGroveBaud);

    // Frame buffers before WiFi grabs heap. Also prefer PSRAM for these.
    // (Tile PNGs are now decoded by streaming straight off the SD file --
    // see drawPngFromSd -- so there's no separate decode buffer to size or
    // allocate here at all.)
    canvas.setColorDepth(16);
    canvas.setPsram(true);
    gCanvasOk = canvas.createSprite(SCREEN_W, SCREEN_H) != nullptr;
    mapCanvas.setColorDepth(16);
    mapCanvas.setPsram(true);
    gScrollOk = mapCanvas.createSprite(SCREEN_W, SCREEN_H) != nullptr;

    net.begin();
    wstore.load();
    gMap.begin(settings.baseDefault);
    // begin() sets the view to DEFAULT_LAT/LON (London); override it with the
    // user's saved default, if any, before bootScreen() tries for a real fix.
    // A real fix (below) still wins over either one.
    if (settings.hasDefaultPos) gMap.centerOn(settings.defLat, settings.defLon);
    gMap.setTrack(&track);
    gMap.setWaypoints(&wstore);
    gMap.setIconsVisible(settings.showWaypointsOnBoot);

    bootScreen(d, sdOk, gps, gMap);   // animation + acquire current position
    tIdle = millis();
}

void loop() {
    const uint32_t now = millis();

    gps.update();
    Input::poll();
    { double lat, lon; if (notesUi.consumePicked(lat, lon)) gDist.useWaypoint(gMap, lat, lon); }
    if (gps.consumeFresh()) { gMap.onGpsFix(gps.lat(), gps.lon()); recorder.addPoint(gps, settings.trackMinDistM); }

    // Screen off: GPS parsing + track recording above already ran this tick
    // regardless -- only the display/compose work below is skipped.
    if (!gScreenOn) return;

    if (portal.active() || settingsUi.active() || trackListUi.active() ||
        waypointUi.active() || notesUi.active() || bulkUi.active() || gHelp) {
        Gfx &mg = gCanvasOk ? static_cast<Gfx&>(canvas) : static_cast<Gfx&>(M5Cardputer.Display);
        if      (portal.active())      portal.update(mg, net);
        else if (settingsUi.active())  settingsUi.update(mg);
        else if (trackListUi.active()) trackListUi.update(mg);
        else if (waypointUi.active())  waypointUi.update(mg);
        else if (notesUi.active())     notesUi.update(mg, wstore);
        else if (bulkUi.active())      bulkUi.update(mg);
        else if (gHelp)                { if (gHelpDraw) { Ui::drawHelp(mg, gHelpScroll); gHelpDraw = false; } }
        if (gCanvasOk) canvas.pushSprite(0, 0);
        return;
    }

    bool dirty  = gMap.consumeDirty();
    if (dirty) {
        composeAndPush();
        tIdle = now;
    } else if ((now - tIdle) >= IDLE_REFRESH_MS) {
        Ui::drawStatusBar(M5Cardputer.Display, gps, gMap, net, settings, recorder);
        tIdle = now;
    }
}
