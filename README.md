# Cardputer-Adv — Off-grid Offline Map Viewer

A PlatformIO / Arduino app for the **M5Stack Cardputer-Adv** (ESP32-S3FN8,
ST7789V2 240×135) that shows your live GPS position over **offline OpenStreetMap
tiles** read from a MicroSD card. GPS comes from the **Cap LoRa-1262** module
(AT6668/ATGM336H). Pan the map with the keyboard; press **`g`** to snap back to GPS.

## Features
- M5Unified / M5GFX display setup (landscape, 240×135)
- TinyGPS++ NMEA parsing on HardwareSerial1 (GPIO 13 RX / GPIO 15 TX @ 9600)
- Slippy-map tile engine (Web Mercator) reading `/offgridmap/tiles/{z}/{x}/{y}.png` from SD
- Red GPS marker + centre crosshair + status bar (lat, lon, sats, zoom, mode)
- Non-blocking `millis()` timers; map redraws only when the view changes
- Cardputer keyboard panning + live zoom (`+`/`-`)
- Optional on-device WiFi tile download (cache the map straight to SD)

## Build & flash
1. Install [PlatformIO](https://platformio.org/) (VS Code extension or CLI).
2. Open this folder. Libraries in `platformio.ini` download automatically.
3. Connect the Cardputer-Adv over USB-C and run **Upload** (`pio run -t upload`).
4. Serial monitor is 115200 baud.

> Board is set to `m5stack-stamps3` (the Cardputer's ESP32-S3 module) with an
> 8 MB flash partition. If PlatformIO can't resolve it, update the espressif32
> platform.

## SD card layout
Everything the app reads or writes lives under one root folder on the card,
**`/offgridmap`**, so the card root stays uncluttered:

```
SD:/offgridmap/tiles/{z}/{x}/{y}.png       OSM Standard basemap
SD:/offgridmap/topo/{z}/{x}/{y}.png        OpenTopoMap basemap
SD:/offgridmap/cyclosm/{z}/{x}/{y}.png     CyclOSM basemap
SD:/offgridmap/tracks/                     recorded GPX tracks
SD:/offgridmap/waypoints.csv               saved waypoints / notes
SD:/offgridmap/settings.txt                device settings
SD:/offgridmap/wifi.txt                    saved WiFi credentials
```

The firmware creates `/offgridmap` itself at boot if it's missing, and each
basemap/tracks subfolder is created on first use. If you have an older card
with `/tiles`, `/topo`, `/cyclosm`, `/tracks`, `/waypoints.csv`,
`/settings.txt` or `/wifi.txt` at the card root, move them inside
`/offgridmap` by hand — the firmware doesn't migrate them automatically.
The root folder name is `MAP_ROOT` in `include/config.h` if you'd rather use
something else.

## Getting the maps
Use the included downloader to pull OSM tiles into the exact folder layout:

```bash
# centre + radius (km)
python tools/download_tiles.py --center 51.5074 -0.1278 --radius 3 --zoom 14

# or a bounding box (south west north east)
python tools/download_tiles.py --bbox 51.47 -0.20 51.55 -0.05 --zoom 14
```

This creates `sdcard/offgridmap/tiles/{z}/{x}/{y}.png`. **Copy the whole
`sdcard/offgridmap/` folder to the root of your MicroSD card** (so it becomes
`SD:/offgridmap/...`). Set `DEFAULT_LAT` / `DEFAULT_LON` in
`include/config.h` to your area so a map shows before the first GPS fix.

> **Respect the OSM tile policy** (https://operations.osmfoundation.org/policies/tiles/).
> For large areas use a provider you may bulk-download from via `--url-template`,
> e.g. MapTiler/Thunderforest with your API key, or your own tile server. Other
> offline sources that produce the same `{z}/{x}/{y}.png` layout also work
> (e.g. Mobile Atlas Creator / MOBAC in "OSMdroid ZIP"/"raw PNG" mode).

## Hardware pin notes (verified from the M5 UserDemo)
| Signal | GPIO |
|--------|------|
| SD SCK / MISO / MOSI / CS | 40 / 39 / 14 / 12 |
| GPS (Cap LoRa) ESP-RX / ESP-TX @115200 | 15 / 13 |
| GPS (Grove port) ESP-RX / ESP-TX | 1 / 2 |
| Keyboard interrupt | 11 |
| LoRa SX1262 NSS / RST / BUSY / DIO1 | 5 / 3 / 6 / 4 |

There is **no SD/LoRa pin conflict** (LoRa has its own CS on GPIO 5, sharing the
SPI bus with the SD card). The firmware just holds LoRa NSS high so the idle
SX1262 stays de-selected. Crucially it does **not** touch GPIO 11 (keyboard
interrupt) or 13/15 (GPS) — driving those breaks the keyboard or GPS.

## Keyboard
| Key | Action |
|-----|--------|
| `;` `.` `,` `/` | pan up/down/left/right (slides smoothly) |
| `g` | recenter on GPS (follow mode) |
| `=` / `+` | zoom in |
| `-` | zoom out (zoom range 1-19) |
| `w` | WiFi scan / connect |
| `d` | download tiles for current view (WiFi) |
| `5` | bulk download: pick basemap/zoom/X-Y tile range (WiFi) |
| `t` | auto-cache tiles on/off |
| `r` | start / stop recording a GPX track |
| `p` | pause / resume the active recording |
| `4` | track manager: view / info / rename / delete (`x` = back to Main map) |
| `m` | switch basemap: OSM Standard -> OpenTopoMap -> CyclOSM |
| `n` | tag a waypoint at the crosshair (pick icon + note) |
| `1` | toggle waypoint icons on/off |
| `2` | list waypoints; select to view one solo on the map |
| `3` | distance calculator |
| `9` | screen off (battery saver) |
| `0` | debug HUD |
| `s` | settings: time zone, boot map, brightness, GPS module |
| `h` | on-screen help / key list |
| **ESC** | back / cancel in every menu |

**ESC** is the back key everywhere — the keyboard reports that keycap as a
backtick, which is why older builds showed `` ` `` on screen. It's the same
physical button; only the on-screen label changed.

Inside the waypoints list (`2`):

| Key | Action |
|-----|--------|
| `f` | find — type a string, the list filters live on the note text |
| `i` | cycle the icon-category filter (Food, Water, Danger, … , all) |
| `o` | cycle sort: saved order -> newest -> oldest -> nearest |
| `x` | clear search + icon filter, back to saved order |

Arrow directions use the Cardputer Fn layer; remap the `KEY_*` values in `include/config.h`.

## v2.0 features
- **Flicker-free rendering:** the whole frame (map + markers + status bar) is drawn to an
  off-screen buffer and pushed once, and the view **slides** to its target instead of jumping.
- **Status bar** shows local time (GPS time + your time-zone), altitude, travel direction,
  the crosshair's latitude/longitude, satellites, zoom and WiFi state.
- **Direction** is derived from GPS course-over-ground (there is no magnetometer on this
  board, so it shows a heading only while you're moving faster than a walking pace).
- **Time zone** is set on-device via the settings screen (`s`) and saved to `SD:/offgridmap/settings.txt`.
- **Track recording** (`r`) writes a standard **GPX** file to `SD:/offgridmap/tracks/` (named
  `trk_YYYYMMDD_HHMMSS.gpx` from GPS UTC time). Press `l` for the track manager:
  **view** on the map (thicker line, green start dot, red end square), **info**
  (start/end lat-lon, altitude, timestamps, distance, duration), **rename**, or **delete**.

## Downloading maps on the device (WiFi)
No credentials need to be hard-coded. Press **`w`** on the map to open the WiFi menu:
1. It scans and lists nearby networks (signal strength shown; `*` = secured).
2. Use `;` / `.` to move, **ENTER** to pick a network, `` ` `` to cancel.
3. Type the password on the Cardputer keyboard, **DEL** to erase, **ENTER** to connect.
4. On success the credentials are saved to `SD:/offgridmap/wifi.txt`, so it auto-reconnects
   on the next boot. The status bar shows `WiFi` when connected.

Once connected, back on the map:
- Press **`d`** to download every missing tile for the current on-screen view at
  the current zoom (progress bar shows the count). Repeat for other areas/zooms.
- Press **`t`** to auto-cache tiles as you pan/zoom.

You can still pre-set a network in `include/config.h` (`WIFI_SSID` / `WIFI_PASS`)
as a fallback, or seed a large area much faster from a PC with
`tools/download_tiles.py`. Respect the OSM tile policy either way.

### Bulk range download (key `5`)
For pre-loading a whole area at once, without panning across it tile by tile:
press **`5`** to open a six-step wizard --

1. **Basemap** -- left/right to pick OSM Standard / OpenTopoMap / CyclOSM.
2. **Zoom** -- left/right, clamped to that basemap's native max (over-zoom is a
   rendering trick, not a real tile, so it isn't offered here). This screen
   also shows the tile the crosshair is currently sitting in at that zoom,
   e.g. `view centre here = tile 32,21` -- use it to work out the range you want.
3. **X1 / X2** and **Y1 / Y2** -- type the tile-index box you want (west/east
   then north/south edges). Each field is pre-filled with a sensible default
   (X1/Y1 = the crosshair's tile, X2/Y2 continue from X1/Y1) and is validated
   against `0..(2^zoom - 1)` for that zoom **before** it's accepted -- type an
   out-of-range number and it tells you the valid span right there instead of
   failing later. Reversed ranges (X2 < X1) are swapped automatically.
4. **Confirm** -- shows the basemap, zoom, X/Y range and total tile count.
   Requests over `BULK_DL_MAX_TILES` (4000 by default, in `config.h`) are
   flagged in red and blocked -- split a big area into a few smaller requests
   instead. Tiles already on the card are skipped, never overwritten.

Press ENTER to start; a progress bar tracks it the same way `d` does, and
**esc** cancels mid-download (whatever finished downloading up to that point
stays on the card). The status bar's `Z<n> x,y` reading (see below) is handy
for scouting X/Y ranges before you even open the wizard.

## v3 features
- **Smoother navigation (scroll-blit):** the map layer is kept in an off-screen
  buffer and *scrolled*, so panning only decodes the newly-revealed edge tiles
  instead of the whole screen. Set `USE_SCROLL_BLIT = false` in `config.h` to
  revert to full-frame redraw if needed.
- **Waypoints:** press `n` to drop a tagged marker at the crosshair - choose an
  icon (food, gas, skull, X, info, water, bus, train, hospital, caution), type a
  note; lat/lon + UTC time are saved to `SD:/offgridmap/waypoints.csv`. Works while
  recording a track. `1` toggles icons; `2` lists them (select = show solo + jump).
- **Settings:** the settings screen (`s`) now also sets the **boot basemap**
  (which style loads on power-up), saved alongside the time zone.
- **Boot screen:** a Fallout / Pip-Boy style green terminal boot animation.

## v3.1 features
- **Searchable / filterable waypoints:** in the `2` list press `f` to type a
  search string (case-insensitive, matches the note text, filters as you type),
  `i` to filter by icon category, `o` to sort by **newest / oldest / nearest**
  (nearest measures from your live GPS fix, or the map centre if there's no fix
  yet — the distance is shown on each row), and `x` to clear everything. A status
  strip above the list shows `hits/total`, the sort mode and the active filters.
  Filtering is a view over the store, so `waypoints.csv` keeps its original order.
- **Second GPS option:** a generic NMEA receiver (e.g. NEO-6M) on the **Grove
  port**, selected on the settings screen with its own baud rate. See
  [GPS module options](#gps-module-options).
- **Key changes:** the track manager moved from `l` to **`4`**, and
  pause/resume-recording moved from `4` to **`p`**. The back key is now labelled
  **esc** everywhere instead of `` ` `` (same physical key).
- **Single SD root folder:** all app data moved under **`/offgridmap`** (tiles,
  topo, cyclosm, tracks, waypoints.csv, settings.txt, wifi.txt) instead of
  sitting loose at the card root. See [SD card layout](#sd-card-layout) —
  existing cards need their old files moved in by hand.

## v3.2 features
- **Tile X,Y in the status bar:** row 2's right side now shows the crosshair's
  tile index next to the zoom, e.g. `OSM Z6 1,2 S07 W`. It only appears when
  it fits without crowding the lat/lon on the left, so it fades out again at
  high zoom where tile numbers get long (into the thousands/hundred-thousands) —
  zoom and satellite count are still always shown either way.
- **Bulk range download (`5`):** a wizard to pre-download a whole rectangle of
  tiles — pick a basemap, a zoom level, and a west/east/north/south tile-index
  box — instead of panning across it one screen at a time. See
  [Bulk range download](#bulk-range-download-key-5).

## Basemaps (map styles) & terrain
Press **`m`** to cycle basemap styles on the device. Each style is stored in its
own folder on the SD card so they don't overwrite each other:

| Style | Folder | Best for | Native max zoom |
|-------|--------|----------|-----------------|
| OSM Standard | `/offgridmap/tiles` | general navigation, streets, paths | 19 |
| OpenTopoMap  | `/offgridmap/topo`  | **hiking** - contour lines + hillshading | 17 |
| CyclOSM      | `/offgridmap/cyclosm` | outdoor / cycling | 18 |

The active style is shown in the status bar (`OSM` / `TOPO` / `CYCL`). Zooming
past a style's native max still works via **over-zoom** (the app scales the
highest available tiles), so you can push OpenTopoMap beyond 17 - it just gets
softer, not blank.

Download tiles for a style with the PC script by pointing `--root` at its folder
and `--url-template` at the provider (topo caps at zoom 17):

```
py tools\download_tiles.py --center 51.5074 -0.1278 --radius 5 --zoom 14 15 16 17 ^
   --root topo --url-template "https://tile.opentopomap.org/{z}/{x}/{y}.png"
```

Then copy the resulting `topo` folder into `/offgridmap` on the SD card (so it
becomes `SD:/offgridmap/topo`). On-device `d`/`D` download into whichever
basemap is currently active. Respect each provider's
tile-usage policy and attribution (OpenTopoMap is CC-BY-SA); they run on
donated servers, so keep volumes modest or self-host for large areas.

## GPS module options
Two receivers are supported and you pick between them on the **settings screen
(`s` → "GPS module")**. The choice is saved to `SD:/offgridmap/settings.txt`
and applied immediately — no reboot.

| Setting | Receiver | ESP RX | ESP TX | Baud |
|---------|----------|--------|--------|------|
| `LoRa Cap` | AT6668/ATGM336H on the Cap LoRa-1262 (14P header) | GPIO 15 | GPIO 13 | 115200 (fixed) |
| `Grove UART` | any NMEA module on the Grove port, e.g. **GY-NEO6MV2 (u-blox NEO-6M)** | GPIO 1 | GPIO 2 | selectable, default 9600 |

Both share HardwareSerial 1, so only one can be active at a time — which is fine,
since only one is ever plugged in.

### Wiring a NEO-6M to the Grove port
The HY2.0-4P Grove connector carries **GND, 5V, G2, G1**:

| NEO-6M pin | Grove pin |
|------------|-----------|
| VCC | 5V — the GY-NEO6MV2 breakout has its own 3.3V regulator |
| GND | GND |
| TX  | **G1** (Cardputer RX — this is the wire that matters) |
| RX  | G2 (Cardputer TX — only needed to reconfigure the module) |

G1/G2 are **3.3V logic** on the ESP32-S3, which is not 5V tolerant. The NEO-6M's
TX line idles at 3.3V so it connects directly; never drive these pins from a
5V-logic device. If `rx` stays at 0, swap the TX/RX wires first, then try another
baud rate on the settings screen (`s` → "GPS baud": 4800 … 115200).

## GPS troubleshooting
Verified against the official M5 Cardputer-Adv UserDemo (`hal_config.h` +
`uart_helper.c`): the Cap LoRa-1262 GPS uses **ESP RX = GPIO 15, ESP TX = GPIO 13,
baud = 115200** (not 9600 — an ATGM336H default that trips a lot of people up).
The config matches these. If you still see nothing, swap `GPS_RX_PIN` / `GPS_TX_PIN`.

Diagnostics built in:
- The status bar shows `rx:<n>` until you get a fix. **If `rx` stays 0**, no NMEA
  bytes are arriving -> wrong RX pin, swapped wires, wrong baud, or no power to
  the module. **If `rx` climbs but there's still no fix**, wiring is fine — you
  just need satellites: go outside with a clear view of the sky (first cold fix
  can take 30 s to a few minutes).
- Set `GPS_DEBUG = true` in `config.h` (default on) to echo raw NMEA to the USB
  serial monitor at 115200 — you should see `$GNGGA`/`$GPRMC` lines streaming.

## Layout
```
platformio.ini            build config + library deps
include/config.h          all pins, zoom, timings, key map, default view
src/main.cpp              modular app (Gps / Slippy / MapView / Ui / Input)
tools/download_tiles.py   OSM tile downloader -> SD:/offgridmap/... layout
data/                     placeholder (tiles live on SD, not flash)
```

## Notes / tuning
- `MAP_ZOOM` is the boot zoom; `+`/`-` move between `MAP_ZOOM_MIN` and
  `MAP_ZOOM_MAX`. Download every level you want to browse (each level up is ~4x
  the tiles).
- If SD reads are flaky, lower `SD_SPI_HZ` to 10 MHz.
- Rendering streams PNGs straight from SD to the panel (no full-frame buffer),
  keeping RAM use low on the S3. 