#!/usr/bin/env python3
"""
make_gz_world.py - Build a georeferenced Gazebo Harmonic world around a lat/lon
for ArduPilot SITL drone navigation testing.

What it produces (in --out):
  worlds/<name>.sdf                    world with spherical_coordinates = your point
  models/<name>_ground/                textured ground (aerial imagery), optional terrain
  models/<name>_buildings/             OSM + Microsoft footprints extruded to 3D
  models/<name>_trees/                 OSM individual trees (natural=tree)
  site_info.json                       origin, elevation, counts, sources
  run.sh                               launch helper: local gz, or Docker (--docker-image) if absent

Data sources (fetched from YOUR machine):
  - Imagery:   Esri World Imagery tiles (check Esri terms of use) or --imagery-url template
  - Buildings: OpenStreetMap via Overpass API (ODbL, attribution required)
  - Buildings: Microsoft Global ML Building Footprints (ODbL) fill gaps in OSM;
               heights mostly unknown -> --default-building-height
  - Elevation: OpenTopoData public API (SRTM 30 m by default)

Deps: python3, requests, pillow, numpy    (pip install requests pillow numpy)

Example:
  python3 gazebo/make_gz_world.py --lat -22.380870396426957 --lon -46.93462825810613 \
      --radius 50 --terrain --name site01 --out gazebo/sites/site01
"""

import argparse
import io
import json
import math
import os
import sys
import time

import numpy as np

try:
    import requests
except ImportError:  # allow --help / offline tests without requests
    requests = None
try:
    from PIL import Image
except ImportError:
    Image = None

UA = "make_gz_world/1.0 (drone sim world generator)"

# --------------------------------------------------------------------------- #
# Geodesy: local ENU tangent plane (accurate to cm over a few hundred metres)
# --------------------------------------------------------------------------- #
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


class LocalFrame:
    def __init__(self, lat0, lon0):
        self.lat0, self.lon0 = lat0, lon0
        phi = math.radians(lat0)
        s = math.sin(phi)
        self.M = WGS84_A * (1 - WGS84_E2) / (1 - WGS84_E2 * s * s) ** 1.5  # meridional
        self.N = WGS84_A / math.sqrt(1 - WGS84_E2 * s * s)                  # prime vertical
        self.cos0 = math.cos(phi)

    def to_xy(self, lat, lon):
        """lat/lon -> (x east, y north) metres; Gazebo ENU world frame."""
        x = math.radians(lon - self.lon0) * self.N * self.cos0
        y = math.radians(lat - self.lat0) * self.M
        return x, y

    def to_latlon(self, x, y):
        lat = self.lat0 + math.degrees(y / self.M)
        lon = self.lon0 + math.degrees(x / (self.N * self.cos0))
        return lat, lon


# --------------------------------------------------------------------------- #
# HTTP helpers
# --------------------------------------------------------------------------- #
def http_get(url, params=None, timeout=30, retries=3):
    for i in range(retries):
        try:
            r = requests.get(url, params=params, timeout=timeout,
                             headers={"User-Agent": UA})
            if r.status_code == 429:
                time.sleep(2 + 2 * i)
                continue
            r.raise_for_status()
            return r
        except Exception as e:  # noqa: BLE001
            if i == retries - 1:
                raise
            print(f"  retry {url[:60]}... ({e})")
            time.sleep(1.5 * (i + 1))


def http_post(url, data, timeout=90, retries=3):
    for i in range(retries):
        try:
            r = requests.post(url, data=data, timeout=timeout,
                              headers={"User-Agent": UA})
            if r.status_code in (429, 504):
                time.sleep(5 * (i + 1))
                continue
            r.raise_for_status()
            return r
        except Exception as e:  # noqa: BLE001
            if i == retries - 1:
                raise
            print(f"  retry overpass ({e})")
            time.sleep(5 * (i + 1))


# --------------------------------------------------------------------------- #
# Imagery (slippy-map tiles, Web Mercator)
# --------------------------------------------------------------------------- #
ESRI_URL = ("https://server.arcgisonline.com/ArcGIS/rest/services/"
            "World_Imagery/MapServer/tile/{z}/{y}/{x}")


def latlon_to_global_px(lat, lon, z):
    n = 256 * (2 ** z)
    x = (lon + 180.0) / 360.0 * n
    lat_r = math.radians(lat)
    y = (1 - math.log(math.tan(lat_r) + 1 / math.cos(lat_r)) / math.pi) / 2 * n
    return x, y


def fetch_imagery(frame, half, zoom, url_tmpl, out_png, min_zoom=16):
    """Fetch tiles covering the square [-half, half]^2 and crop it exactly.
    Falls back to lower zoom if the server returns blank/placeholder tiles."""
    lat_n, lon_w = frame.to_latlon(-half, half)
    lat_s, lon_e = frame.to_latlon(half, -half)
    for z in range(zoom, min_zoom - 1, -1):
        x0, y0 = latlon_to_global_px(lat_n, lon_w, z)
        x1, y1 = latlon_to_global_px(lat_s, lon_e, z)
        tx0, ty0 = int(x0 // 256), int(y0 // 256)
        tx1, ty1 = int(x1 // 256), int(y1 // 256)
        ntiles = (tx1 - tx0 + 1) * (ty1 - ty0 + 1)
        print(f"  imagery zoom {z}: {ntiles} tiles")
        mosaic = Image.new("RGB", ((tx1 - tx0 + 1) * 256, (ty1 - ty0 + 1) * 256))
        ok = True
        for tx in range(tx0, tx1 + 1):
            for ty in range(ty0, ty1 + 1):
                try:
                    r = http_get(url_tmpl.format(z=z, x=tx, y=ty), timeout=20)
                    tile = Image.open(io.BytesIO(r.content)).convert("RGB")
                except Exception as e:  # noqa: BLE001
                    print(f"    tile {tx},{ty} failed: {e}")
                    ok = False
                    break
                mosaic.paste(tile, ((tx - tx0) * 256, (ty - ty0) * 256))
            if not ok:
                break
        if not ok:
            continue
        crop = mosaic.crop((round(x0 - tx0 * 256), round(y0 - ty0 * 256),
                            round(x1 - tx0 * 256), round(y1 - ty0 * 256)))
        arr = np.asarray(crop, dtype=np.float32)
        # Esri "Map data not yet available" placeholders are near-uniform grey.
        if arr.std() < 6.0:
            print(f"    zoom {z} looks like a placeholder (std={arr.std():.1f}), trying lower")
            continue
        native = crop.size[0]
        # resample to a power-of-two square (GPU-friendly), not below native size
        side = 1 << max(9, math.ceil(math.log2(native)))
        side = min(side, 4096)
        crop.resize((side, side), Image.LANCZOS).save(out_png)
        gsd = 2 * half / native
        print(f"  imagery saved: {side}px (native {native}px, ~{gsd:.2f} m/px, zoom {z})")
        return {"zoom": z, "native_px": native, "gsd_m": round(gsd, 3), "size_px": side}
    raise RuntimeError("could not fetch imagery at any zoom")


def placeholder_texture(out_png, size=512):
    """Checker-grid ground used with --imagery none (10 m squares)."""
    img = np.zeros((size, size, 3), np.uint8)
    img[:] = (96, 128, 72)
    step = size // 10
    for i in range(10):
        for j in range(10):
            if (i + j) % 2:
                img[i * step:(i + 1) * step, j * step:(j + 1) * step] = (86, 116, 64)
    img[size // 2 - 1:size // 2 + 1, :] = (200, 60, 60)   # x axis (east) red
    img[:, size // 2 - 1:size // 2 + 1] = (60, 200, 60)   # y axis (north) green
    Image.fromarray(img).save(out_png)


# --------------------------------------------------------------------------- #
# Elevation
# --------------------------------------------------------------------------- #
def fetch_elevation_grid(frame, half, n, dataset):
    """n x n grid of elevations (row 0 = south) from OpenTopoData."""
    xs = np.linspace(-half, half, n)
    pts = [(x, y) for y in xs for x in xs]
    locs = [frame.to_latlon(x, y) for x, y in pts]
    elev = []
    for i in range(0, len(locs), 100):
        chunk = locs[i:i + 100]
        q = "|".join(f"{la:.7f},{lo:.7f}" for la, lo in chunk)
        r = http_get(f"https://api.opentopodata.org/v1/{dataset}", params={"locations": q})
        res = r.json()["results"]
        elev += [e["elevation"] if e["elevation"] is not None else np.nan for e in res]
        time.sleep(1.1)  # public API: 1 request / second
    g = np.array(elev, dtype=float).reshape(n, n)
    if np.isnan(g).all():
        raise RuntimeError("elevation API returned no data")
    g[np.isnan(g)] = np.nanmean(g)
    return xs, g


def bilinear(xs, grid, x, y):
    """Sample grid (row = y index ascending, col = x) at (x, y)."""
    x = min(max(x, xs[0]), xs[-1])
    y = min(max(y, xs[0]), xs[-1])
    i = min(np.searchsorted(xs, x) - 1, len(xs) - 2)
    j = min(np.searchsorted(xs, y) - 1, len(xs) - 2)
    i, j = max(i, 0), max(j, 0)
    tx = (x - xs[i]) / (xs[i + 1] - xs[i])
    ty = (y - xs[j]) / (xs[j + 1] - xs[j])
    a = grid[j, i] * (1 - tx) + grid[j, i + 1] * tx
    b = grid[j + 1, i] * (1 - tx) + grid[j + 1, i + 1] * tx
    return a * (1 - ty) + b * ty


# --------------------------------------------------------------------------- #
# OSM via Overpass
# --------------------------------------------------------------------------- #
OVERPASS_URLS = ["https://overpass-api.de/api/interpreter",
                 "https://overpass.kumi.systems/api/interpreter"]


def fetch_osm(lat, lon, radius_m):
    q = f"""[out:json][timeout:60];
(
  way["building"](around:{radius_m},{lat},{lon});
  relation["building"](around:{radius_m},{lat},{lon});
  way["building:part"](around:{radius_m},{lat},{lon});
  node["natural"="tree"](around:{radius_m},{lat},{lon});
);
out geom;"""
    last = None
    for url in OVERPASS_URLS:
        try:
            return http_post(url, {"data": q}).json()
        except Exception as e:  # noqa: BLE001
            last = e
            print(f"  overpass {url} failed: {e}")
    raise RuntimeError(f"Overpass unavailable: {last}")


def parse_height(tags, default_h, level_h=3.0):
    for k in ("height", "building:height"):
        if k in tags:
            try:
                return float(tags[k].replace("m", "").replace(",", ".").strip())
            except ValueError:
                pass
    if "building:levels" in tags:
        try:
            return float(tags["building:levels"]) * level_h + 0.5
        except ValueError:
            pass
    return default_h


def osm_buildings(osm, frame, default_h):
    out = []
    for el in osm.get("elements", []):
        tags = el.get("tags", {})
        rings = []
        if el["type"] == "way" and "geometry" in el:
            rings.append(el["geometry"])
        elif el["type"] == "relation":
            for m in el.get("members", []):
                if m.get("role") == "outer" and "geometry" in m:
                    rings.append(m["geometry"])
        if not rings:
            continue
        h = parse_height(tags, default_h)
        min_h = 0.0
        if "min_height" in tags:
            try:
                min_h = float(tags["min_height"])
            except ValueError:
                pass
        for ring in rings:
            pts = [frame.to_xy(p["lat"], p["lon"]) for p in ring]
            if len(pts) > 1 and np.allclose(pts[0], pts[-1]):
                pts = pts[:-1]
            if len(pts) >= 3:
                out.append({"pts": pts, "h": h, "min_h": min_h,
                            "id": el["id"], "name": tags.get("name", "")})
    return out


# Microsoft Global ML Building Footprints (ODbL): footprints detected in satellite
# imagery, worldwide. Far better coverage than OSM in many places (e.g. Brazilian
# residential areas), but heights are mostly unknown (height = -1). Distributed
# as one gzipped GeoJSON-lines file per zoom-9 quadkey (~tens of MB), cached.
MS_LINKS_URL = "https://minedbuildings.z5.web.core.windows.net/global-buildings/dataset-links.csv"


def quadkey(lat, lon, z):
    s = math.sin(math.radians(lat))
    n = 2 ** z
    x = min(n - 1, int((lon + 180) / 360 * n))
    y = min(n - 1, int((0.5 - math.log((1 + s) / (1 - s)) / (4 * math.pi)) * n))
    return "".join(str(((x >> i) & 1) | (((y >> i) & 1) << 1)) for i in range(z - 1, -1, -1))


def cache_dir():
    d = os.path.join(os.environ.get("XDG_CACHE_HOME", os.path.expanduser("~/.cache")),
                     "make_gz_world")
    os.makedirs(d, exist_ok=True)
    return d


def fetch_ms_buildings(frame, half, default_h):
    """Microsoft footprints overlapping the mapped square, in the same dict
    shape as osm_buildings()."""
    import csv
    import gzip
    corners = [frame.to_latlon(sx * half, sy * half) for sx in (-1, 1) for sy in (-1, 1)]
    qks = {quadkey(la, lo, 9) for la, lo in corners}
    links = http_get(MS_LINKS_URL, timeout=60).text
    urls = [r["Url"] for r in csv.DictReader(io.StringIO(links)) if r["QuadKey"] in qks]
    if not urls:
        print("  no Microsoft footprint tiles cover this area")
    lats = [c[0] for c in corners]
    lons = [c[1] for c in corners]
    out = []
    for url in urls:
        path = os.path.join(cache_dir(), "msbuildings_" + url.split("quadkey=")[1].replace("/", "_"))
        if not os.path.exists(path):
            print(f"  downloading {url.rsplit('/', 3)[-3]} footprint tile (cached afterwards) ...")
            with requests.get(url, stream=True, timeout=120, headers={"User-Agent": UA}) as r:
                r.raise_for_status()
                with open(path + ".part", "wb") as fh:
                    for chunk in r.iter_content(1 << 20):
                        fh.write(chunk)
            os.replace(path + ".part", path)
        with gzip.open(path, "rt") as fh:
            for line in fh:
                f = json.loads(line)
                ring = f["geometry"]["coordinates"][0]
                rlon = [p[0] for p in ring]
                rlat = [p[1] for p in ring]
                if (max(rlon) < min(lons) or min(rlon) > max(lons) or
                        max(rlat) < min(lats) or min(rlat) > max(lats)):
                    continue
                pts = [frame.to_xy(la, lo) for lo, la in ring]
                if len(pts) > 1 and np.allclose(pts[0], pts[-1]):
                    pts = pts[:-1]
                h = f.get("properties", {}).get("height", -1) or -1
                if len(pts) >= 3:
                    out.append({"pts": pts, "h": h if h > 0 else default_h, "min_h": 0.0,
                                "id": f"ms{len(out)}", "name": "", "source": "microsoft",
                                "height_known": h > 0})
    return out


def point_in_poly(p, pts):
    x, y = p
    inside = False
    for (x1, y1), (x2, y2) in zip(pts, pts[1:] + pts[:1]):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


def merge_buildings(primary, extra):
    """Add `extra` footprints that don't overlap any `primary` one (centroid test
    both ways). OSM is primary: it is hand-mapped and may carry real heights."""
    def centroid(pts):
        return (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts))
    keep = [e for e in extra
            if not any(point_in_poly(centroid(e["pts"]), b["pts"]) or
                       point_in_poly(centroid(b["pts"]), e["pts"]) for b in primary)]
    return primary + keep


def osm_trees(osm, frame):
    t = []
    for el in osm.get("elements", []):
        if el["type"] == "node" and el.get("tags", {}).get("natural") == "tree":
            x, y = frame.to_xy(el["lat"], el["lon"])
            h = parse_height(el["tags"], 8.0)
            t.append((x, y, h))
    return t


# --------------------------------------------------------------------------- #
# Geometry helpers
# --------------------------------------------------------------------------- #
def signed_area(pts):
    a = 0.0
    for i in range(len(pts)):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % len(pts)]
        a += x1 * y2 - x2 * y1
    return a / 2


def _point_in_tri(p, a, b, c):
    def s(p1, p2, p3):
        return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1])
    d1, d2, d3 = s(p, a, b), s(p, b, c), s(p, c, a)
    neg = d1 < 0 or d2 < 0 or d3 < 0
    pos = d1 > 0 or d2 > 0 or d3 > 0
    return not (neg and pos)


def triangulate(pts):
    """Ear clipping for a simple CCW polygon. Returns index triples."""
    idx = list(range(len(pts)))
    tris = []
    guard = 0
    while len(idx) > 3 and guard < 10000:
        guard += 1
        found = False
        for k in range(len(idx)):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % len(idx)]
            a, b, c = pts[i0], pts[i1], pts[i2]
            cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
            if cross <= 1e-9:
                continue
            if any(_point_in_tri(pts[j], a, b, c) for j in idx if j not in (i0, i1, i2)):
                continue
            tris.append((i0, i1, i2))
            idx.pop(k)
            found = True
            break
        if not found:  # degenerate / self-intersecting: fan the rest
            for k in range(1, len(idx) - 1):
                tris.append((idx[0], idx[k], idx[k + 1]))
            return tris
    if len(idx) == 3:
        tris.append(tuple(idx))
    return tris


class ObjWriter:
    def __init__(self):
        self.v, self.vt, self.vn, self.f = [], [], [], []

    def add_vertex(self, p):
        self.v.append(p)
        return len(self.v)

    def add_normal(self, n):
        self.vn.append(n)
        return len(self.vn)

    def add_uv(self, uv):
        self.vt.append(uv)
        return len(self.vt)

    def face(self, vs, n=None, uvs=None):
        self.f.append((vs, n, uvs))

    def write(self, path, mtl=None, mtl_name=None):
        with open(path, "w") as fh:
            fh.write("# generated by make_gz_world.py\n")
            if mtl:
                fh.write(f"mtllib {mtl}\nusemtl {mtl_name}\n")
            for p in self.v:
                fh.write(f"v {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
            for t in self.vt:
                fh.write(f"vt {t[0]:.6f} {t[1]:.6f}\n")
            for n in self.vn:
                fh.write(f"vn {n[0]:.5f} {n[1]:.5f} {n[2]:.5f}\n")
            for vs, n, uvs in self.f:
                parts = []
                for k, vi in enumerate(vs):
                    nk = n[k] if isinstance(n, (list, tuple)) else n
                    if uvs and n:
                        parts.append(f"{vi}/{uvs[k]}/{nk}")
                    elif uvs:
                        parts.append(f"{vi}/{uvs[k]}")
                    elif n:
                        parts.append(f"{vi}//{nk}")
                    else:
                        parts.append(str(vi))
                fh.write("f " + " ".join(parts) + "\n")

    def write_stl(self, path):
        """Binary STL of the same triangles, for <collision> geometry.

        Gazebo Harmonic's default physics (DART with the ODE collision detector)
        aborts with `assertion "vertices" failed in buildData()` when an OBJ is
        used as a collision mesh, even a trivial one. STL loads fine, so OBJ is
        kept for visuals (it carries UVs) and STL is used for collisions.
        """
        import struct
        v = np.asarray(self.v, dtype=np.float64)
        with open(path, "wb") as fh:
            fh.write(b"make_gz_world collision mesh".ljust(80, b"\0"))
            fh.write(struct.pack("<I", len(self.f)))
            for vs, _, _ in self.f:
                a, b, c = (v[i - 1] for i in vs)
                n = np.cross(b - a, c - a)
                ln = np.linalg.norm(n)
                n = n / ln if ln > 0 else n
                fh.write(struct.pack("<12fH", *n, *a, *b, *c, 0))


def build_ground_mesh(path, half, xs=None, grid=None, z0=0.0, n=41):
    """Square ground mesh, UV (0,0)=SW corner, (1,1)=NE corner. Z relative to origin.
    Returns the lowest ground z (for placing the horizon skirt below it)."""
    w = ObjWriter()
    g = np.linspace(-half, half, n)
    zs = np.zeros((n, n))
    for j, y in enumerate(g):
        for i, x in enumerate(g):
            zs[j, i] = 0.0 if grid is None else bilinear(xs, grid, x, y) - z0
    # Per-vertex normals from the height field. They are required: Gazebo's
    # ogre2 renders a normal-less OBJ flat white, ignoring its texture.
    dzdy, dzdx = np.gradient(zs, g, g)
    ids = np.zeros((n, n), int)
    for j, y in enumerate(g):
        for i, x in enumerate(g):
            ids[j, i] = w.add_vertex((x, y, zs[j, i]))
            w.add_uv(((x + half) / (2 * half), (y + half) / (2 * half)))
            nv = np.array([-dzdx[j, i], -dzdy[j, i], 1.0])
            w.add_normal(tuple(nv / np.linalg.norm(nv)))
    for j in range(n - 1):
        for i in range(n - 1):
            a, b, c, d = ids[j, i], ids[j, i + 1], ids[j + 1, i + 1], ids[j + 1, i]
            w.face([a, b, c], n=[a, b, c], uvs=[a, b, c])
            w.face([a, c, d], n=[a, c, d], uvs=[a, c, d])
    w.write(path, mtl="ground.mtl", mtl_name="ground")
    w.write_stl(os.path.splitext(path)[0] + ".stl")
    with open(os.path.join(os.path.dirname(path), "ground.mtl"), "w") as fh:
        fh.write("newmtl ground\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\n"
                 "map_Kd ../materials/textures/ground.png\n")
    return float(zs.min())


def build_buildings_mesh(walls_path, roofs_path, buildings, ground_z):
    walls, roofs = ObjWriter(), ObjWriter()
    for b in buildings:
        pts = b["pts"]
        if signed_area(pts) < 0:
            pts = pts[::-1]
        if abs(signed_area(pts)) < 1.0:
            continue
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        base = min(ground_z(x, y) for x, y in pts + [(cx, cy)])
        z_bot = base - 0.5 + b["min_h"]              # sink slightly into terrain
        z_top = max(ground_z(cx, cy), base) + b["h"]
        # walls
        for k in range(len(pts)):
            (x1, y1), (x2, y2) = pts[k], pts[(k + 1) % len(pts)]
            dx, dy = x2 - x1, y2 - y1
            L = math.hypot(dx, dy)
            if L < 1e-6:
                continue
            ni = walls.add_normal((dy / L, -dx / L, 0.0))   # outward for CCW
            a = walls.add_vertex((x1, y1, z_bot))
            bb = walls.add_vertex((x2, y2, z_bot))
            c = walls.add_vertex((x2, y2, z_top))
            d = walls.add_vertex((x1, y1, z_top))
            walls.face([a, bb, c], n=ni)
            walls.face([a, c, d], n=ni)
        # roof (flat) + floor
        nu = roofs.add_normal((0, 0, 1))
        nd = roofs.add_normal((0, 0, -1))
        top = [roofs.add_vertex((x, y, z_top)) for x, y in pts]
        bot = [roofs.add_vertex((x, y, z_bot)) for x, y in pts]
        for t in triangulate(pts):
            roofs.face([top[t[0]], top[t[1]], top[t[2]]], n=nu)
            roofs.face([bot[t[0]], bot[t[2]], bot[t[1]]], n=nd)
    walls.write(walls_path)
    roofs.write(roofs_path)
    walls.write_stl(os.path.splitext(walls_path)[0] + ".stl")
    roofs.write_stl(os.path.splitext(roofs_path)[0] + ".stl")
    return len(walls.f), len(roofs.f)


# --------------------------------------------------------------------------- #
# SDF writers
# --------------------------------------------------------------------------- #
def model_config(path, name, desc):
    with open(path, "w") as fh:
        fh.write(f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>{desc}</description>
</model>
""")


def write_ground_model(mdir, name, half, flat, skirt_z=-0.05):
    collision = (f"""      <collision name="collision">
        <geometry><plane><normal>0 0 1</normal><size>{2*half:.1f} {2*half:.1f}</size></plane></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision>""" if flat else
                 f"""      <collision name="collision">
        <geometry><mesh><uri>model://{name}/meshes/ground.stl</uri></mesh></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision>""")
    with open(os.path.join(mdir, "model.sdf"), "w") as fh:
        fh.write(f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
{collision}
      <visual name="visual">
        <geometry><mesh><uri>model://{name}/meshes/ground.obj</uri></mesh></geometry>
        <material>
          <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>
          <pbr><metal>
            <albedo_map>model://{name}/materials/textures/ground.png</albedo_map>
            <roughness>1.0</roughness><metalness>0.0</metalness>
          </metal></pbr>
        </material>
      </visual>
      <!-- large skirt so the horizon isn't a void beyond the mapped square; kept
           below the lowest terrain point so it never hides the imagery -->
      <visual name="skirt">
        <pose>0 0 {skirt_z:.2f} 0 0 0</pose>
        <geometry><plane><normal>0 0 1</normal><size>2000 2000</size></plane></geometry>
        <material><ambient>0.33 0.38 0.27 1</ambient><diffuse>0.33 0.38 0.27 1</diffuse>
          <specular>0 0 0 1</specular></material>
      </visual>
    </link>
  </model>
</sdf>
""")
    model_config(os.path.join(mdir, "model.config"), name, "Georeferenced ground with aerial imagery")


def write_buildings_model(mdir, name):
    with open(os.path.join(mdir, "model.sdf"), "w") as fh:
        fh.write(f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="walls_col">
        <geometry><mesh><uri>model://{name}/meshes/walls.stl</uri></mesh></geometry>
      </collision>
      <collision name="roofs_col">
        <geometry><mesh><uri>model://{name}/meshes/roofs.stl</uri></mesh></geometry>
      </collision>
      <visual name="walls">
        <geometry><mesh><uri>model://{name}/meshes/walls.obj</uri></mesh></geometry>
        <material><ambient>0.78 0.75 0.70 1</ambient><diffuse>0.78 0.75 0.70 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular></material>
      </visual>
      <visual name="roofs">
        <geometry><mesh><uri>model://{name}/meshes/roofs.obj</uri></mesh></geometry>
        <material><ambient>0.62 0.32 0.24 1</ambient><diffuse>0.62 0.32 0.24 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular></material>
      </visual>
    </link>
  </model>
</sdf>
""")
    model_config(os.path.join(mdir, "model.config"), name, "OSM buildings (ODbL)")


def write_trees_model(mdir, name, trees, ground_z):
    links = []
    for k, (x, y, h) in enumerate(trees):
        z = ground_z(x, y)
        trunk_h = h * 0.45
        crown_r = max(1.2, h * 0.3)
        links.append(f"""    <link name="tree_{k}">
      <pose>{x:.2f} {y:.2f} {z:.2f} 0 0 0</pose>
      <collision name="trunk_c"><pose>0 0 {trunk_h/2:.2f} 0 0 0</pose>
        <geometry><cylinder><radius>0.2</radius><length>{trunk_h:.2f}</length></cylinder></geometry></collision>
      <collision name="crown_c"><pose>0 0 {trunk_h + crown_r*0.8:.2f} 0 0 0</pose>
        <geometry><sphere><radius>{crown_r:.2f}</radius></sphere></geometry></collision>
      <visual name="trunk"><pose>0 0 {trunk_h/2:.2f} 0 0 0</pose>
        <geometry><cylinder><radius>0.2</radius><length>{trunk_h:.2f}</length></cylinder></geometry>
        <material><ambient>0.35 0.24 0.15 1</ambient><diffuse>0.35 0.24 0.15 1</diffuse></material></visual>
      <visual name="crown"><pose>0 0 {trunk_h + crown_r*0.8:.2f} 0 0 0</pose>
        <geometry><sphere><radius>{crown_r:.2f}</radius></sphere></geometry>
        <material><ambient>0.18 0.42 0.16 1</ambient><diffuse>0.18 0.42 0.16 1</diffuse></material></visual>
    </link>""")
    with open(os.path.join(mdir, "model.sdf"), "w") as fh:
        fh.write(f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
{chr(10).join(links)}
  </model>
</sdf>
""")
    model_config(os.path.join(mdir, "model.config"), name, "OSM trees (ODbL)")


def write_world(path, name, lat, lon, elev, half, models, vehicle, spawn_z):
    includes = "\n".join(f"""    <include>
      <uri>model://{m}</uri>
      <name>{m}</name>
      <pose>0 0 0 0 0 0</pose>
    </include>""" for m in models)
    veh = ""
    if vehicle:
        veh = f"""
    <!-- ArduPilot vehicle (from ardupilot_gazebo). Faces north (yaw 90 deg in ENU). -->
    <include>
      <uri>model://{vehicle}</uri>
      <pose degrees="true">0 0 {spawn_z:.3f} 0 0 90</pose>
    </include>"""
    with open(path, "w") as fh:
        fh.write(f"""<?xml version="1.0" ?>
<!--
  Generated by make_gz_world.py
  Origin: {lat:.9f}, {lon:.9f}  elevation {elev:.1f} m (AMSL)
  Mapped area: {2*half:.0f} x {2*half:.0f} m centred on origin. x = East, y = North.
  Map data (c) OpenStreetMap contributors (ODbL). Imagery: see site_info.json.
-->
<sdf version="1.9">
  <world name="{name}">
    <physics name="1ms" type="ignore">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
    <plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>
    <plugin filename="gz-sim-altimeter-system" name="gz::sim::systems::Altimeter"/>

    <scene>
      <ambient>0.8 0.8 0.8 1</ambient>
      <background>0.62 0.78 0.95 1</background>
      <sky></sky>
      <shadows>true</shadows>
    </scene>

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{lat:.9f}</latitude_deg>
      <longitude_deg>{lon:.9f}</longitude_deg>
      <elevation>{elev:.2f}</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.3 -0.85</direction>
    </light>

{includes}{veh}
  </world>
</sdf>
""")


# Launch helper written to <out>/run.sh. Uses a local `gz` if installed, otherwise
# runs Gazebo inside a Docker image that ships Gazebo Harmonic + ardupilot_gazebo.
# Placeholders: @NAME@ world name, @IMAGE@ docker image, @HOME@ SITL home lat,lon,alt,hdg.
RUN_SH = r"""#!/usr/bin/env bash
# Launch the generated world.
#   ./run.sh            headless server (physics + sensors + ArduPilot plugin, no window)
#   ./run.sh --gui      also open the Gazebo GUI
# Uses a local 'gz' if installed (needs ardupilot_gazebo on GZ_SIM_SYSTEM_PLUGIN_PATH
# and GZ_SIM_RESOURCE_PATH); otherwise runs Gazebo in Docker:
#   GZ_IMAGE=<image> ./run.sh     (default image: @IMAGE@)
#   GZ_DOCKER=1 ./run.sh          force Docker even if 'gz' is installed
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD="worlds/@NAME@.sdf"
GUI=0
for arg in "$@"; do
  case "$arg" in
    --gui) GUI=1 ;;
    -h|--help) sed -n '2,8p' "$0"; exit 0 ;;
    *) echo "unknown option: $arg (try --help)" >&2; exit 2 ;;
  esac
done

echo "SITL home must match the world origin. In another terminal run:"
echo "  sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console -l @HOME@"

if command -v gz >/dev/null 2>&1 && [ "${GZ_DOCKER:-0}" != 1 ]; then
  export GZ_SIM_RESOURCE_PATH="$HERE/models:${GZ_SIM_RESOURCE_PATH:-}"
  if [ "$GUI" = 1 ]; then exec gz sim -v4 -r "$HERE/$WORLD"; fi
  exec gz sim -s -r --headless-rendering -v4 "$HERE/$WORLD"
fi

command -v docker >/dev/null 2>&1 || {
  echo "Neither 'gz' nor 'docker' found: install Gazebo Harmonic + ardupilot_gazebo, or Docker." >&2
  exit 1; }
IMAGE="${GZ_IMAGE:-@IMAGE@}"
echo "'gz' not installed locally -> running Gazebo in Docker image $IMAGE"

# Keep the image's resource path (ardupilot_gazebo models) and put this site's models first.
RES="$(docker image inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$IMAGE" 2>/dev/null \
       | sed -n 's/^GZ_SIM_RESOURCE_PATH=//p' || true)"
# --network host : ArduPilot plugin <-> SITL over UDP 9002/9003 on localhost, and
#                  gz-transport discovery (gz topic ...) works from the host.
# --entrypoint gz: bypass the image's entrypoint, which launches its own world.
ARGS=(--rm --init --network host --name "gz-@NAME@"
      -v "$HERE:/site:ro"
      -e "GZ_SIM_RESOURCE_PATH=/site/models:${RES:-/opt/ardupilot_gazebo/models}"
      --entrypoint gz)
if [ -t 0 ]; then ARGS+=(-it); fi
if [ -e /dev/dri ]; then ARGS+=(--device /dev/dri); fi

if [ "$GUI" = 1 ]; then
  [ -n "${DISPLAY:-}" ] || { echo "--gui needs an X11/XWayland DISPLAY" >&2; exit 1; }
  if command -v xhost >/dev/null 2>&1; then xhost +local: >/dev/null; fi
  ARGS+=(-e "DISPLAY=$DISPLAY" -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw)
  exec docker run "${ARGS[@]}" "$IMAGE" sim -v4 -r "/site/$WORLD"
fi
exec docker run "${ARGS[@]}" "$IMAGE" sim -s -r --headless-rendering -v4 "/site/$WORLD"
"""


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lat", type=float, required=True)
    ap.add_argument("--lon", type=float, required=True)
    ap.add_argument("--radius", type=float, default=50.0,
                    help="half-size of mapped square in metres (default 50 -> 100x100 m)")
    ap.add_argument("--name", default="geo_site")
    ap.add_argument("--out", default=None)
    ap.add_argument("--zoom", type=int, default=19, help="max imagery zoom (falls back if blank)")
    ap.add_argument("--imagery", default="esri",
                    help="'esri', 'none' (checker grid), or a tile URL template with {z}/{x}/{y}")
    ap.add_argument("--terrain", action="store_true",
                    help="use elevation grid for ground relief (SRTM is ~30 m: coarse)")
    ap.add_argument("--dem-dataset", default="srtm30m",
                    help="OpenTopoData dataset (srtm30m, aster30m, mapzen ...)")
    ap.add_argument("--elevation", type=float, default=None,
                    help="override origin elevation AMSL (m); skips elevation API if no --terrain")
    ap.add_argument("--default-building-height", type=float, default=6.0)
    ap.add_argument("--no-osm", action="store_true", help="skip OSM buildings/trees")
    ap.add_argument("--no-ms-buildings", dest="ms_buildings", action="store_false",
                    help="skip Microsoft ML building footprints (used to fill buildings "
                         "missing from OSM; ~30 MB tile download, cached in ~/.cache)")
    ap.add_argument("--vehicle", default="iris_dual_cam",
                    help="model to spawn at origin ('' for none). Default: this repo's "
                         "camera airframe (gazebo/models/iris_dual_cam); ardupilot_gazebo's "
                         "iris_with_gimbal has no /cam1 /cam2 and ignores GZ_FDM_IN_PORT.")
    ap.add_argument("--docker-image", default="ghcr.io/draganfly/gazebo-sitl:latest",
                    help="Gazebo Harmonic + ardupilot_gazebo image that run.sh falls back to "
                         "when 'gz' is not installed (override at run time with GZ_IMAGE=...)")
    a = ap.parse_args(argv)

    if requests is None or Image is None:
        sys.exit("missing deps: pip install requests pillow numpy")

    out = os.path.abspath(a.out or a.name)
    half = a.radius
    frame = LocalFrame(a.lat, a.lon)
    gname, bname, tname = f"{a.name}_ground", f"{a.name}_buildings", f"{a.name}_trees"
    gdir = os.path.join(out, "models", gname)
    for d in ("meshes", "materials/textures"):
        os.makedirs(os.path.join(gdir, d), exist_ok=True)
    os.makedirs(os.path.join(out, "worlds"), exist_ok=True)
    info = {"origin": {"lat": a.lat, "lon": a.lon}, "area_m": 2 * half}

    # ---- elevation
    xs = grid = None
    elev0 = a.elevation
    if a.terrain or elev0 is None:
        print("[1/4] elevation ...")
        try:
            n = 9 if a.terrain else 2
            xs, grid = fetch_elevation_grid(frame, half, n, a.dem_dataset)
            e_center = bilinear(xs, grid, 0, 0)
            if elev0 is None:
                elev0 = float(e_center)
            info["elevation"] = {"dataset": a.dem_dataset, "origin_amsl": round(elev0, 2),
                                 "relief_m": round(float(grid.max() - grid.min()), 2)}
            print(f"  origin elevation {elev0:.1f} m, relief over area "
                  f"{grid.max() - grid.min():.1f} m")
            if not a.terrain:
                xs = grid = None
        except Exception as e:  # noqa: BLE001
            print(f"  elevation failed ({e}); using 0 m. Pass --elevation to set it.")
            elev0 = elev0 or 0.0
            xs = grid = None
            info["elevation"] = {"origin_amsl": elev0, "note": "API failed"}
    else:
        info["elevation"] = {"origin_amsl": elev0, "source": "user"}

    def ground_z(x, y):
        return 0.0 if grid is None else bilinear(xs, grid, x, y) - elev0

    # ---- imagery
    print("[2/4] imagery ...")
    tex = os.path.join(gdir, "materials/textures/ground.png")
    if a.imagery == "none":
        placeholder_texture(tex)
        info["imagery"] = {"source": "checker placeholder"}
    else:
        url = ESRI_URL if a.imagery == "esri" else a.imagery
        try:
            meta = fetch_imagery(frame, half, a.zoom, url, tex)
            meta["source"] = "Esri World Imagery" if a.imagery == "esri" else url
            info["imagery"] = meta
        except Exception as e:  # noqa: BLE001
            print(f"  imagery failed ({e}); using checker placeholder")
            placeholder_texture(tex)
            info["imagery"] = {"source": "checker placeholder", "error": str(e)}

    zmin = build_ground_mesh(os.path.join(gdir, "meshes/ground.obj"), half, xs, grid, elev0)
    write_ground_model(gdir, gname, half, flat=grid is None, skirt_z=zmin - 0.05)
    models = [gname]

    # ---- OSM
    buildings, trees = [], []
    if not a.no_osm:
        print("[3/4] OpenStreetMap ...")
        try:
            osm = fetch_osm(a.lat, a.lon, half * 1.45 + 10)  # covers square corners
            buildings = [b for b in osm_buildings(osm, frame, a.default_building_height)
                         if any(abs(x) <= half and abs(y) <= half for x, y in b["pts"])]
            trees = [t for t in osm_trees(osm, frame) if abs(t[0]) <= half and abs(t[1]) <= half]
        except Exception as e:  # noqa: BLE001
            print(f"  OSM failed: {e}")
    print(f"  {len(buildings)} buildings, {len(trees)} trees")
    info["osm"] = {"buildings": len(buildings), "trees": len(trees),
                   "attribution": "(c) OpenStreetMap contributors, ODbL"}

    # ---- Microsoft ML footprints, filling what OSM lacks
    if a.ms_buildings:
        print("[3b/4] Microsoft building footprints ...")
        try:
            ms = [b for b in fetch_ms_buildings(frame, half, a.default_building_height)
                  if any(abs(x) <= half and abs(y) <= half for x, y in b["pts"])]
            n_osm = len(buildings)
            buildings = merge_buildings(buildings, ms)
            added = len(buildings) - n_osm
            known = sum(1 for b in buildings[n_osm:] if b["height_known"])
            print(f"  {len(ms)} footprints, {added} added (not already in OSM); "
                  f"{known} with a height, the rest {a.default_building_height} m "
                  f"(--default-building-height)")
            info["microsoft_buildings"] = {
                "footprints": len(ms), "added": added, "with_height": known,
                "attribution": "Microsoft Global ML Building Footprints, ODbL"}
        except Exception as e:  # noqa: BLE001
            print(f"  Microsoft footprints failed: {e}")

    if buildings:
        bdir = os.path.join(out, "models", bname)
        os.makedirs(os.path.join(bdir, "meshes"), exist_ok=True)
        build_buildings_mesh(os.path.join(bdir, "meshes/walls.obj"),
                             os.path.join(bdir, "meshes/roofs.obj"), buildings, ground_z)
        write_buildings_model(bdir, bname)
        models.append(bname)
        info["buildings"] = [{"id": b["id"], "name": b["name"], "height_m": b["h"],
                              "source": b.get("source", "osm")} for b in buildings]
    if trees:
        tdir = os.path.join(out, "models", tname)
        os.makedirs(tdir, exist_ok=True)
        write_trees_model(tdir, tname, trees, ground_z)
        models.append(tname)

    # ---- world
    print("[4/4] world ...")
    spawn_z = ground_z(0, 0) + 0.2
    world = os.path.join(out, "worlds", f"{a.name}.sdf")
    write_world(world, a.name, a.lat, a.lon, elev0, half, models, a.vehicle, spawn_z)
    # ArduPilot --home / SITL_HOME: "lat,lon,alt,heading". Heading 0 = north,
    # matching the vehicle spawn yaw in write_world().
    info["sitl_home"] = f"{a.lat:.8f},{a.lon:.8f},{elev0:.1f},0"
    with open(os.path.join(out, "site_info.json"), "w") as fh:
        json.dump(info, fh, indent=2)
    with open(os.path.join(out, "run.sh"), "w") as fh:
        script = RUN_SH
        for key, val in {"@NAME@": a.name, "@IMAGE@": a.docker_image,
                         "@HOME@": f"{a.lat:.8f},{a.lon:.8f},{elev0:.1f},0"}.items():
            script = script.replace(key, val)
        fh.write(script)
    os.chmod(os.path.join(out, "run.sh"), 0o755)
    print(f"\nDone -> {out}\n  ./run.sh [--gui]   (prints the sim_vehicle.py command to use)")
    return out


if __name__ == "__main__":
    main()
