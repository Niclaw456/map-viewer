#!/usr/bin/env python3
"""
download_tiles.py  -  Fetch OpenStreetMap raster tiles for offline use
on the Cardputer-Adv map viewer.

Downloads standard 256x256 PNG tiles into the exact folder layout the
firmware expects:

    <out>/<root>/{z}/{x}/{y}.png

By default that's  sdcard/offgridmap/tiles/{z}/{x}/{y}.png -- copy the whole
sdcard/offgridmap/  folder to the root of your MicroSD card so the device
sees  SD:/offgridmap/tiles/...  (all app data -- tiles, tracks, waypoints,
settings, wifi -- lives under that one /offgridmap folder on the card).

USAGE
-----
    # A bounding box (south west north east) at zoom 14:
    python download_tiles.py --bbox 51.47 -0.20 51.55 -0.05 --zoom 14

    # ...or a centre point + radius (km):
    python download_tiles.py --center 51.5074 -0.1278 --radius 3 --zoom 14

    # multiple zooms at once:
    python download_tiles.py --center 51.5074 -0.1278 --radius 3 --zoom 12 14

    # OpenTopoMap (topographic) into the "topo" basemap folder (max zoom 17):
    python download_tiles.py --center 51.5074 -0.1278 --radius 5 --zoom 14 15 16 17 \
        --root topo --url-template "https://tile.opentopomap.org/{z}/{x}/{y}.png"

OSM TILE USAGE POLICY  (read before bulk downloading!)
------------------------------------------------------
The default tile.openstreetmap.org server is a donated resource. Keep
downloads modest, set a real User-Agent, and do NOT hammer it. For large
areas use a provider you are allowed to bulk-download from (MapTiler,
Thunderforest, your own tile server) via --url-template.
Policy: https://operations.osmfoundation.org/policies/tiles/
"""

import argparse
import math
import os
import sys
import time
import urllib.request

DEFAULT_TEMPLATE = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
USER_AGENT = "cardputer-adv-mapviewer/1.0 (offline personal use)"


def deg2num(lat, lon, z):
    lat_r = math.radians(lat)
    n = 2 ** z
    x = int((lon + 180.0) / 360.0 * n)
    y = int((1.0 - math.asinh(math.tan(lat_r)) / math.pi) / 2.0 * n)
    return x, y


def km_to_bbox(lat, lon, radius_km):
    dlat = radius_km / 111.0
    dlon = radius_km / (111.0 * math.cos(math.radians(lat)) or 1e-6)
    return (lat - dlat, lon - dlon, lat + dlat, lon + dlon)


def fetch(url, dest, retries=3):
    req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    for attempt in range(1, retries + 1):
        try:
            with urllib.request.urlopen(req, timeout=20) as r:
                data = r.read()
            with open(dest, "wb") as f:
                f.write(data)
            return True
        except Exception as e:
            print(f"    ! {url} ({attempt}/{retries}): {e}", file=sys.stderr)
            time.sleep(1.5 * attempt)
    return False


def main():
    ap = argparse.ArgumentParser(description="Download OSM tiles for offline use.")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--bbox", nargs=4, type=float,
                   metavar=("S", "W", "N", "E"), help="south west north east")
    g.add_argument("--center", nargs=2, type=float,
                   metavar=("LAT", "LON"), help="centre point")
    ap.add_argument("--radius", type=float, default=2.0,
                    help="radius in km when using --center (default 2)")
    ap.add_argument("--zoom", nargs="+", type=int, required=True,
                    help="one or more zoom levels, e.g. --zoom 12 14")
    ap.add_argument("--out", default="sdcard/offgridmap",
                    help="output folder (default sdcard/offgridmap -- copy this whole "
                         "folder to the MicroSD card root as /offgridmap)")
    ap.add_argument("--root", default="tiles",
                    help="tile subfolder / basemap name on the SD card, e.g. tiles, topo, cyclosm (default tiles)")
    ap.add_argument("--url-template", default=DEFAULT_TEMPLATE,
                    help="tile URL template with {z}/{x}/{y}")
    ap.add_argument("--delay", type=float, default=0.2,
                    help="seconds between requests (be polite, default 0.2)")
    args = ap.parse_args()

    if args.bbox:
        s, w, n, e = args.bbox
    else:
        lat, lon = args.center
        s, w, n, e = km_to_bbox(lat, lon, args.radius)
    if s > n:
        s, n = n, s
    if w > e:
        w, e = e, w

    total_planned = 0
    plan = []
    for z in args.zoom:
        x0, y1 = deg2num(s, w, z)   # SW -> min x, max y
        x1, y0 = deg2num(n, e, z)   # NE -> max x, min y
        xa, xb = min(x0, x1), max(x0, x1)
        ya, yb = min(y0, y1), max(y0, y1)
        cnt = (xb - xa + 1) * (yb - ya + 1)
        plan.append((z, xa, xb, ya, yb, cnt))
        total_planned += cnt

    print(f"Area: S{s:.4f} W{w:.4f} N{n:.4f} E{e:.4f}")
    for z, xa, xb, ya, yb, cnt in plan:
        print(f"  zoom {z}: x[{xa}..{xb}] y[{ya}..{yb}]  = {cnt} tiles")
    print(f"TOTAL: {total_planned} tiles -> {args.out}/{args.root}/")
    if total_planned > 5000:
        print("WARNING: that is a lot of tiles. Consider a smaller area or a "
              "provider that allows bulk downloads.", file=sys.stderr)
    if input("Proceed? [y/N] ").strip().lower() != "y":
        print("Aborted.")
        return

    got, skipped, failed = 0, 0, 0
    for z, xa, xb, ya, yb, _ in plan:
        for x in range(xa, xb + 1):
            d = os.path.join(args.out, args.root, str(z), str(x))
            os.makedirs(d, exist_ok=True)
            for y in range(ya, yb + 1):
                dest = os.path.join(d, f"{y}.png")
                if os.path.exists(dest) and os.path.getsize(dest) > 0:
                    skipped += 1
                    continue
                url = args.url_template.format(z=z, x=x, y=y)
                if fetch(url, dest):
                    got += 1
                    print(f"  [{got}] z{z}/{x}/{y}")
                else:
                    failed += 1
                time.sleep(args.delay)

    print(f"\nDone. downloaded={got} skipped={skipped} failed={failed}")
    print(f"Copy the '{args.out}' folder to the ROOT of your MicroSD card "
          f"(so it becomes SD:/offgridmap/...) -- this run's tiles land in "
          f"/offgridmap/{args.root}/{args.zoom[0]}/...")


if __name__ == "__main__":
    main()
