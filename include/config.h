#pragma once
// =====================================================================
//  config.h  -  Central pin map & tunables for the Cardputer-Adv
//  Everything hardware-specific lives here so main.cpp stays portable.
// =====================================================================

// ---------------------------------------------------------------------
//  DISPLAY  (handled internally by M5GFX -> just documented here)
//  ST7789V2, 240 x 135, landscape rotation = 1
// ---------------------------------------------------------------------
static constexpr int  SCREEN_W        = 240;
static constexpr int  SCREEN_H        = 135;
static constexpr int  SCREEN_ROTATION = 1;

// ---------------------------------------------------------------------
//  GPS  -  TWO selectable receivers, switched at runtime in Settings.
//  ESP32 RX  <- GPS TX ,  ESP32 TX -> GPS RX   (for both options)
//
//  0) CAP LORA  - AT6668 / ATGM336H on the Cap LoRa-1262 module (stacked
//     on the 14P header). Fixed 115200 baud.
//  1) GROVE     - any generic NMEA receiver on the HY2.0-4P Grove port,
//     e.g. a GY-NEO6MV2 (u-blox NEO-6M). Baud is user-selectable because
//     NEO-6M ships at 9600 but is often reconfigured.
//
//  GROVE WIRING (HY2.0-4P, looking at the port: GND, 5V, G2, G1):
//      NEO-6M VCC -> 5V     (the GY-NEO6MV2 breakout regulates to 3.3V;
//                            do NOT feed 5V to a bare 3.3V-only module)
//      NEO-6M GND -> GND
//      NEO-6M TX  -> G1     (Cardputer RX -- this is the wire that matters)
//      NEO-6M RX  -> G2     (Cardputer TX -- optional, only for config)
//  If you see 0 sats and RX stays at 0, swap the TX/RX wires first.
//  NOTE: G1/G2 are 3.3V logic. The NEO-6M's TX idles at 3.3V, so it is a
//  direct connection; never drive these pins from a 5V-logic device.
// ---------------------------------------------------------------------
static constexpr int  GPS_SRC_CAP   = 0;   // Cap LoRa-1262 module (14P header)
static constexpr int  GPS_SRC_GROVE = 1;   // Grove port UART (NEO-6M etc.)
static constexpr int  GPS_SRC_COUNT = 2;
static constexpr int  GPS_SRC_DEFAULT = GPS_SRC_CAP;

static constexpr int  GPS_RX_PIN = 15;   // ESP32 RX (listens) = HAL_PIN_GPS_RX on the Cap module
static constexpr int  GPS_TX_PIN = 13;   // ESP32 TX (sends)    = HAL_PIN_GPS_TX on the Cap module
static constexpr int  GPS_BAUD   = 115200; // Cap LoRa-1262 ATGM336H runs at 115200 (per M5 UserDemo)

static constexpr int  GPS_GROVE_RX_PIN = 1;   // ESP32 RX (listens) <- GPS TX   [Grove pin 4 = G1]
static constexpr int  GPS_GROVE_TX_PIN = 2;   // ESP32 TX (sends)   -> GPS RX   [Grove pin 3 = G2]
static constexpr int  GPS_GROVE_BAUD_DEFAULT = 9600;   // u-blox NEO-6M factory default

// Baud choices offered on the Settings screen for the Grove receiver.
static constexpr int  GPS_BAUD_CHOICES[] = { 4800, 9600, 19200, 38400, 57600, 115200 };
static constexpr int  GPS_BAUD_CHOICE_COUNT = (int)(sizeof(GPS_BAUD_CHOICES) / sizeof(GPS_BAUD_CHOICES[0]));

static constexpr int  GPS_UART   = 1;    // HardwareSerial #1
static constexpr bool GPS_DEBUG  = true; // echo raw NMEA to USB Serial @115200 for wiring checks

// ---------------------------------------------------------------------
//  MicroSD  (shared VSPI bus - values confirmed for Cardputer / -Adv)
// ---------------------------------------------------------------------
static constexpr int  SD_SCK_PIN  = 40;
static constexpr int  SD_MISO_PIN = 39;
static constexpr int  SD_MOSI_PIN = 14;
static constexpr int  SD_CS_PIN   = 12;
static constexpr uint32_t SD_SPI_HZ = 20000000UL;   // 20 MHz, drop to 10 MHz if flaky

// ---------------------------------------------------------------------
//  LoRa SX1262  (present on the Cap module - NOT used by the map viewer)
//  REAL pins from the M5 Cardputer-Adv UserDemo (hal_config.h):
//      NSS=5  RST=3  BUSY=6  DIO1=4   (SPI bus shared with SD, separate CS)
//  There is NO SD/LoRa pin conflict. We simply keep NSS HIGH so the idle
//  SX1262 stays de-selected on the shared SPI bus.
//
//  IMPORTANT: GPIO 11 is the KEYBOARD interrupt (HAL_PIN_KEYBOARD_INT) and
//  GPIO 13/15 are the GPS UART. Do NOT drive these as GPIO.
// ---------------------------------------------------------------------
static constexpr int  LORA_NSS_PIN   = 5;
static constexpr int  LORA_RESET_PIN = 3;
static constexpr int  LORA_BUSY_PIN  = 6;
static constexpr int  LORA_DIO1_PIN  = 4;

// ---------------------------------------------------------------------
//  SD CARD LAYOUT  -  everything the app reads/writes lives under one
//  root folder so the card stays tidy. Existing cards need their current
//  /tiles, /topo, /cyclosm, /tracks, /waypoints.csv, /settings.txt and
//  /wifi.txt moved inside MAP_ROOT by hand; the firmware creates MAP_ROOT
//  itself at boot but does not migrate old root-level files for you.
// ---------------------------------------------------------------------
static constexpr char  MAP_ROOT[]   = "/offgridmap";

// ---------------------------------------------------------------------
//  MAP / TILE ENGINE
//  Tiles are standard 256x256 OSM PNGs at  <MAP_ROOT>/tiles/{z}/{x}/{y}.png
// ---------------------------------------------------------------------
static constexpr int   TILE_SIZE    = 256;
static constexpr int   MAP_ZOOM     = 14;         // INITIAL zoom level
static constexpr int   MAP_ZOOM_MIN = 1;          // clamp for +/- keys; z1 is a 2x2 tile world,
                                                   // valid on all 3 sources (z0 also exists but is
                                                   // a single 256x256 tile for the whole planet --
                                                   // not much use zoomed in on a 240x135 screen)
static constexpr int   MAP_ZOOM_MAX = 19;         // OSM's native max; over-zoom scales it for
                                                   // Topo(17)/CyclOSM(18) beyond their own native max
static constexpr char  TILE_ROOT[]    = "/offgridmap/tiles";     // OSM Standard basemap
static constexpr char  TOPO_ROOT[]    = "/offgridmap/topo";      // OpenTopoMap basemap
static constexpr char  CYCLOSM_ROOT[] = "/offgridmap/cyclosm";   // CyclOSM basemap

// ---------------------------------------------------------------------
//  WIFI  (for on-device tile downloading)
//  Leave SSID empty ("") to disable WiFi entirely and run pure-offline.
// ---------------------------------------------------------------------
static constexpr char  WIFI_SSID[] = "";              // <-- your network
static constexpr char  WIFI_PASS[] = "";              // <-- your password

// Tile source used when downloading on-device. Order: z, x, y.
// Respect the OSM tile policy; for heavy use point this at a provider
// you may bulk-download from.
static constexpr char  TILE_URL_TEMPLATE[] = "https://tile.openstreetmap.org/%d/%d/%d.png";
static constexpr char  TILE_USER_AGENT[]   = "cardputer-adv-mapviewer/1.2";

// Where remembered WiFi credentials are stored on the SD card (now a list of
// known networks, most-recently-used first -- see Net::rememberNetwork).
static constexpr char  WIFI_SAVE_PATH[]    = "/offgridmap/wifi.txt";
static constexpr int   MAX_WIFI_NETWORKS   = 16;   // sliding-window cap; oldest dropped past this

// ---------------------------------------------------------------------
//  UI / TIMING  (all non-blocking, millis() based)
// ---------------------------------------------------------------------
static constexpr uint32_t RENDER_INTERVAL_MS    = 300;   // redraw cadence
static constexpr uint32_t GPS_STATS_INTERVAL_MS = 500;   // overlay refresh
static constexpr int      PAN_STEP_PX           = 32;    // keyboard pan step

// ---------------------------------------------------------------------
//  KEY MAP  (Cardputer Fn layer for arrows; adjust to taste)
//
//  KEY_BACK is the physical ESC key -- the Cardputer-Adv keyboard reports
//  it as a backtick character. Everything on screen calls it "esc" because
//  that is what is printed on the keycap; there is no separate button to
//  press and no other way to type a backtick, so the two are the same key.
// ---------------------------------------------------------------------
static constexpr char KEY_BACK      = '`';   // ESC keycap -> reported as '`'
static constexpr char KEY_BACK_NAME[] = "esc";  // how it is labelled on screen

static constexpr char KEY_UP        = ';';
static constexpr char KEY_DOWN      = '.';
static constexpr char KEY_LEFT      = ',';
static constexpr char KEY_RIGHT     = '/';
static constexpr char KEY_RECENTER  = 'g';   // snap view back onto GPS
static constexpr char KEY_ZOOM_IN   = '=';   // also accepts '+'
static constexpr char KEY_ZOOM_OUT  = '-';   // also accepts '_'
static constexpr char KEY_DOWNLOAD  = 'd';   // download MISSING tiles for current view
static constexpr char KEY_REFRESH   = 'D';   // (Shift+D) force re-download current view, overwrite existing
static constexpr char KEY_CENTER_DL = 'c';   // fetch + render ONLY the tile under the crosshair
static constexpr char KEY_WIFI      = 'w';   // open the WiFi scan/connect menu
static constexpr char KEY_HELP      = 'h';   // show the on-screen help / key list
static constexpr char KEY_RECORD    = 'r';   // start/stop recording a GPX track
static constexpr char KEY_PAUSE     = 'p';   // pause/resume the active recording
static constexpr char KEY_TRACKS    = '4';   // list & view saved tracks
static constexpr char KEY_SETTINGS  = 's';   // open settings (time zone)
static constexpr char KEY_BASEMAP   = 'm';   // cycle basemap style (OSM / topo / cyclosm)
static constexpr char KEY_TAG       = 'n';   // drop a waypoint (icon + note) at the crosshair
static constexpr char KEY_ICONS     = '1';   // toggle waypoint icons on/off
static constexpr char KEY_NOTES     = '2';   // list saved waypoints / notes
// --- inside the waypoints/notes list only ---
static constexpr char KEY_WP_FIND   = 'f';   // type a search string (matches the note text)
static constexpr char KEY_WP_ICON   = 'i';   // cycle the icon-category filter
static constexpr char KEY_WP_SORT   = 'o';   // cycle sort order (saved/newest/oldest/nearest)
static constexpr char KEY_WP_CLEAR  = 'x';   // clear search + icon filter, back to saved order
static constexpr char KEY_DISTANCE  = '3';   // toggle distance-calculator mode
static constexpr char KEY_DEBUG     = '0';   // toggle on-screen debug HUD
static constexpr char KEY_SCREEN    = '9';   // screen on/off (backlight + redraw), battery saver
static constexpr char KEY_BULK_DL   = '5';   // bulk-download a basemap/zoom/X-Y tile range

// Sanity cap on a single bulk-download request (BulkDownloadUi, key 5).
// At ~150-300ms per fetched tile a batch this size is already a long sit;
// bigger areas should be split into a few smaller requests instead.
static constexpr int  BULK_DL_MAX_TILES = 4000;

// ---------------------------------------------------------------------
//  DEFAULT VIEW  (shown before the first GPS fix arrives)
// ---------------------------------------------------------------------
static constexpr double DEFAULT_LAT = 51.5074;
static constexpr double DEFAULT_LON = -0.1278;
// ---------------------------------------------------------------------
//  TIME / TRACKS / ANIMATION  (v2.0 features)
// ---------------------------------------------------------------------
static constexpr float  TZ_DEFAULT_HOURS   = 0.0f;      // startup time-zone offset (settable on device)
static constexpr char   SETTINGS_PATH[]    = "/offgridmap/settings.txt";
static constexpr char   TRACK_DIR[]        = "/offgridmap/tracks";
static constexpr double TRACK_MIN_DIST_M   = 3.0;       // default; user-adjustable 3..50m in Settings
static constexpr double TRACK_MIN_DIST_MAX = 50.0;
static constexpr int    MAX_TRACK_POINTS   = 4000;      // cap when drawing a loaded track
static constexpr double SPEED_MIN_KMPH     = 1.5;       // below this, course/direction shows '--'

static constexpr uint32_t IDLE_REFRESH_MS  = 1000;      // redraw cadence when idle (clock/sats)

// ---------------------------------------------------------------------
//  WAYPOINTS / RENDERING  (v3 features)
// ---------------------------------------------------------------------
static constexpr char   WAYPT_PATH[]   = "/offgridmap/waypoints.csv";   // lat,lon,icon,time,note
static constexpr int    MAX_WAYPOINTS  = 300;
static constexpr bool   USE_SCROLL_BLIT = true;  // smooth panning via scroll+edge-fill; set false for old full-redraw
