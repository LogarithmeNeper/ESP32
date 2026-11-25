#!/usr/bin/env python3
"""
make_map.py - builds map_data.h (map image) for the ESP32 screen.

Setup (once):  python3 -m pip install -U pillow requests
Run (from the sketch folder):
    python3 make_map.py                       # colour OpenStreetMap
    MAP_STYLE=dark python3 make_map.py        # dark map (CARTO if CARTO_KEY is set,
                                              #           otherwise inverted OSM)
Reads HOME_LAT / HOME_LON from secrets.h and RADIUS_NM from the .ino.
Re-run after changing either.
"""
import glob, io, math, os, re, sys, time
import requests
from PIL import Image, ImageDraw, ImageEnhance, ImageOps

MAP_W, MAP_H = 150, 130     # size of the map box on the screen
RING_R = 54                 # pixels from home to the outer range ring
ZOOM_BIAS = 1               # +1 = more map detail (more tiles), 0 = coarser
BRIGHTNESS = 0.75           # colour map is dimmed so the overlays stand out
USER_AGENT = "esp32-flight-panel-mapmaker/1.0 (personal project)"

CARTO_URL = "https://a.basemaps.cartocdn.com/dark_nolabels/{z}/{x}/{y}.png"
OSM_URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
RESAMPLE = getattr(Image, "Resampling", Image).LANCZOS


def die(msg):
    print("ERROR:", msg)
    sys.exit(1)


def rgb565(r, g, b):
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)


def read_config(folder):
    try:
        secrets = open(os.path.join(folder, "secrets.h"), encoding="utf-8").read()
        lat = float(re.search(r"#define\s+HOME_LAT\s+(-?[\d.]+)", secrets).group(1))
        lon = float(re.search(r"#define\s+HOME_LON\s+(-?[\d.]+)", secrets).group(1))
    except (OSError, AttributeError):
        die("could not read HOME_LAT / HOME_LON from secrets.h in " + folder)
    inos = glob.glob(os.path.join(folder, "*.ino"))
    if not inos:
        die("no .ino file found in " + folder)
    m = re.search(r"const\s+int\s+RADIUS_NM\s*=\s*(\d+)", open(inos[0], encoding="utf-8").read())
    if not m:
        die("could not find 'const int RADIUS_NM = ...' in " + inos[0])
    return lat, lon, int(m.group(1))


def world_px(lat, lon, z):
    n = 256 * 2 ** z
    x = (lon + 180.0) / 360.0 * n
    s = math.sin(math.radians(lat))
    y = (0.5 - math.log((1 + s) / (1 - s)) / (4 * math.pi)) * n
    return x, y


def make_map(folder):
    lat, lon, radius_nm = read_config(folder)
    style = os.environ.get("MAP_STYLE", "osm").strip().lower()
    key = os.environ.get("CARTO_KEY", "").strip()

    if style == "dark" and key:
        url, params, invert, light = CARTO_URL, {"key": key}, False, False
        attribution = "(c) OSM (c) CARTO"
        print("Map: CARTO dark")
    elif style == "dark":
        url, params, invert, light = OSM_URL, {}, True, False
        attribution = "(c) OSM contributors"
        print("Map: OpenStreetMap, inverted to dark")
    else:
        url, params, invert, light = OSM_URL, {}, False, True
        attribution = "(c) OSM contributors"
        print("Map: OpenStreetMap, full colour")

    px_per_km = RING_R / (radius_nm * 1.852)        # output pixels per km
    out_m_per_px = 1000.0 / px_per_km

    def src_m_per_px(z):
        return 156543.03392 * math.cos(math.radians(lat)) / 2 ** z

    z = 0
    while z < 18 and src_m_per_px(z) > out_m_per_px / 1.6:
        z += 1
    z = max(0, min(18, z + ZOOM_BIAS))

    scale = out_m_per_px / src_m_per_px(z)          # source pixels per output pixel
    win_w, win_h = MAP_W * scale, MAP_H * scale
    cx, cy = world_px(lat, lon, z)
    x0, y0, x1, y1 = cx - win_w / 2, cy - win_h / 2, cx + win_w / 2, cy + win_h / 2

    tx0, tx1 = int(math.floor(x0 / 256)), int(math.floor((x1 - 1e-6) / 256))
    ty0, ty1 = int(math.floor(y0 / 256)), int(math.floor((y1 - 1e-6) / 256))
    n_tiles = (tx1 - tx0 + 1) * (ty1 - ty0 + 1)
    print(f"Zoom {z}, {n_tiles} tiles")
    if n_tiles > 36:
        die("too many tiles; lower ZOOM_BIAS")

    canvas = Image.new("RGB", ((tx1 - tx0 + 1) * 256, (ty1 - ty0 + 1) * 256))
    session = requests.Session()
    session.headers["User-Agent"] = USER_AGENT
    n = 2 ** z
    for ty in range(ty0, ty1 + 1):
        for tx in range(tx0, tx1 + 1):
            r = session.get(url.format(z=z, x=tx % n, y=ty), params=params, timeout=20)
            if r.status_code != 200:
                die(f"tile download failed (HTTP {r.status_code}): {r.url.split('?')[0]}")
            tile = Image.open(io.BytesIO(r.content)).convert("RGB")
            canvas.paste(tile, ((tx - tx0) * 256, (ty - ty0) * 256))
            time.sleep(0.3)                          # be polite to the tile server

    box = (x0 - tx0 * 256, y0 - ty0 * 256, x1 - tx0 * 256, y1 - ty0 * 256)
    img = canvas.resize((MAP_W, MAP_H), RESAMPLE, box=box)

    if light:                                        # colour map: dim it a little
        img = ImageEnhance.Color(img).enhance(0.9)
        img = ImageEnhance.Brightness(img).enhance(BRIGHTNESS)
    else:                                            # dark map: blue-tinted greyscale
        g = img.convert("L")
        if invert:
            g = ImageOps.invert(g)
        g = ImageOps.autocontrast(g, cutoff=1)
        img = ImageOps.colorize(g, black=(8, 12, 24), white=(90, 108, 150))

    vals = [rgb565(r, g, b) for r, g, b in img.getdata()]
    out = os.path.join(folder, "map_data.h")
    with open(out, "w", encoding="utf-8") as f:
        f.write("// Generated by make_map.py - do not edit, do not commit (shows your location)\n")
        f.write("#pragma once\n#include <stdint.h>\n#include <pgmspace.h>\n")
        f.write(f"#define MAP_IMG_W {MAP_W}\n#define MAP_IMG_H {MAP_H}\n")
        f.write(f"#define MAP_IMG_R {RING_R}\n#define MAP_IMG_RADIUS_NM {radius_nm}\n")
        f.write(f"#define MAP_IMG_LIGHT {1 if light else 0}\n")
        f.write(f'#define MAP_IMG_ATTRIBUTION "{attribution}"\n')
        f.write("const uint16_t MAP_IMG[] PROGMEM = {\n")
        for i in range(0, len(vals), 16):
            f.write("  " + ", ".join(f"0x{v:04X}" for v in vals[i:i + 16]) + ",\n")
        f.write("};\n")

    prev = img.resize((MAP_W * 3, MAP_H * 3), RESAMPLE)
    d = ImageDraw.Draw(prev)
    c = (MAP_W * 3 // 2, MAP_H * 3 // 2)
    r3 = RING_R * 3
    d.ellipse([c[0] - r3, c[1] - r3, c[0] + r3, c[1] + r3], outline=(255, 190, 40), width=2)
    d.ellipse([c[0] - 6, c[1] - 6, c[0] + 6, c[1] + 6], fill=(60, 200, 255))
    prev.save(os.path.join(folder, "map_preview.png"))
    print(f"Wrote map_data.h and map_preview.png ({radius_nm} nm = {RING_R} px)")


if __name__ == "__main__":
    make_map(sys.argv[1] if len(sys.argv) > 1 else ".")