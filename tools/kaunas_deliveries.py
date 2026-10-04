# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy", "tifffile", "imagecodecs"]
# ///
"""Build the world of scenarios/kaunas-deliveries.

Delivery drones based on the roofs of two Kaunas shopping centres, Mega and
Akropolis, dispatched by the simulation to orders from a pool of street
addresses. Writes two files:

- world/agents.json: the drones, each a courier of its hub (depot) with its
  own pad on the roof and cruise height, plus one spare per hub.
- world/fields.json: the terrain, from the Copernicus GLO-30 elevation model
  (1 arc-second, about 30 m), cropped to the area the drones fly over, and the
  address book: every real OpenStreetMap building with addr:street +
  addr:housenumber within 5 km of a hub (or ADDRESS_POOL of them, sampled
  with a fixed seed), each served by the nearer hub.

Downloads are cached under .cache/, so the same cache and seed give the same
pack.

    uv run tools/kaunas_deliveries.py                # fetch (or reuse cache), write the pack
    uv run tools/kaunas_deliveries.py --seed 7       # a different address pool

Orders, queues and missions are the simulation's (see the drones model's
dispatch module and the pack's params.toml): every address orders rarely and
at random, orders queue at their hub, and the next ready drone there flies
the oldest one. A delivery is: climb off the roof, fly to the address at a
fixed height above the terrain (MAVLink's terrain frame, so the leg follows
the ground), hover while MAV_CMD_DO_WINCH / WINCH_DELIVER lowers the parcel
and stows the line, fly back and land on the pad, where it is loaded and
charged before it can take the next order.

Roof heights are the mall's OSM building:levels × LEVEL_HEIGHT_M above the
ground around it. GLO-30 is a surface model (it sees some trees and buildings),
which makes terrain-relative heights a little conservative, not short.

Address data © OpenStreetMap contributors, ODbL 1.0.
Elevation: Copernicus DEM GLO-30, © DLR e.V. 2010-2014 and © Airbus Defence
and Space GmbH 2014-2018, provided under COPERNICUS by the European Union and
ESA; all rights reserved.
"""

import argparse
import json
import math
import random
import urllib.parse
import urllib.request
from pathlib import Path

import numpy as np
import tifffile

ROOT = Path(__file__).resolve().parent.parent
PACK = ROOT / "scenarios" / "kaunas-deliveries"
OVERPASS = "https://overpass-api.de/api/interpreter"
DEM_URL = "https://copernicus-dem-30m.s3.amazonaws.com/{0}/{0}.tif"
DEM_TILE = "Copernicus_DSM_COG_10_N{lat:02d}_00_E{lon:03d}_00_DEM"

# Mall footprints' centres and storeys, from OpenStreetMap (ways 454500088
# and 28161836).
HUBS = [
    {"key": "mega", "name": "PLC Mega", "callsign": "MEGA",
     "lat": 54.93968, "lon": 23.89266, "levels": 2},
    {"key": "akropolis", "name": "PLC Akropolis", "callsign": "AKRO",
     "lat": 54.89146, "lon": 23.91949, "levels": 4},
]
LEVEL_HEIGHT_M = 5.0  # retail storeys are tall
GROUND_SAMPLE_M = 250.0  # ground level is the low end of the DEM around the hub

RADIUS_M = 5000.0
MIN_RADIUS_M = 300.0  # skip addresses next door to a hub
TERRAIN_MARGIN_M = 1500.0  # terrain beyond the delivery radius
ADDRESS_POOL = None  # None: every addressed building in range
DRONES_PER_HUB = 20
# Pads (one per drone, plus the spare) in a ring round the hub's centre, at
# least PAD_SPACING_M apart: more than the simulation's 10 m pad radius, so
# each drone lands on its own pad.
PAD_SPACING_M = 15.0
PAD_RING_M = max(25.0, (DRONES_PER_HUB + 1) * PAD_SPACING_M / (2 * math.pi))
CRUISE_AGL_M = [60.0, 70.0, 80.0]  # above the terrain; alternate between neighbours


def download(url: str, path: Path, data: bytes | None = None) -> Path:
    if not path.exists():
        req = urllib.request.Request(
            url, data=data, headers={"User-Agent": "krabshell-scenario-builder/0.1"}
        )
        with urllib.request.urlopen(req, timeout=600) as resp:
            path.write_bytes(resp.read())
    return path


def addresses(hub: dict, cache: Path) -> list[dict]:
    query = (
        "[out:json][timeout:120];"
        f'nwr["addr:housenumber"]["addr:street"]["building"]'
        f"(around:{RADIUS_M:.0f},{hub['lat']},{hub['lon']});"
        "out center tags;"
    )
    path = download(
        OVERPASS, cache / f"{hub['key']}.json", urllib.parse.urlencode({"data": query}).encode()
    )
    out = []
    for el in json.loads(path.read_text())["elements"]:
        lat = el.get("lat") or el.get("center", {}).get("lat")
        lon = el.get("lon") or el.get("center", {}).get("lon")
        if lat is None or lon is None:
            continue
        tags = el["tags"]
        out.append(
            {
                "osm": f"{el['type']}/{el['id']}",
                "label": f"{tags['addr:street']} {tags['addr:housenumber']}",
                "lat": lat,
                "lon": lon,
            }
        )
    # Overpass order is not guaranteed; sort so sampling depends only on the seed.
    out.sort(key=lambda a: a["osm"])
    return out


class Dem:
    """GLO-30 samples over a lat/lon box, stitched from 1° tiles.

    The tiles are PixelIsPoint: sample (0, 0) of tile N54 E023 is exactly at
    55° N, 23° E, and longitude spacing is 1.5" at these latitudes.
    """

    def __init__(self, south, west, north, east, cache: Path):
        tiles = {}
        for lat in range(math.floor(south), math.floor(north) + 1):
            for lon in range(math.floor(west), math.floor(east) + 1):
                name = DEM_TILE.format(lat=lat, lon=lon)
                path = download(DEM_URL.format(name), cache / f"{name}.tif")
                with tifffile.TiffFile(path) as tif:
                    page = tif.pages[0]
                    tie = page.tags["ModelTiepointTag"].value
                    scale = page.tags["ModelPixelScaleTag"].value
                    if tif.geotiff_metadata.get("GTRasterTypeGeoKey") != 2:
                        raise SystemExit(f"{name}: expected PixelIsPoint")
                    tiles[(lat, lon)] = (page.asarray(), tie[3], tie[4], scale[0], scale[1])
        (_, lon0, lat_top, dlon, dlat) = next(iter(tiles.values()))
        self.dlat, self.dlon = dlat, dlon
        # Whole samples covering the box, rows from the south.
        r0 = math.floor((south - math.floor(south)) / dlat)
        r1 = math.ceil((north - math.floor(south)) / dlat)
        c0 = math.floor((west - math.floor(west)) / dlon)
        c1 = math.ceil((east - math.floor(west)) / dlon)
        self.lat0 = math.floor(south) + r0 * dlat
        self.lon0 = math.floor(west) + c0 * dlon
        rows, cols = r1 - r0 + 1, c1 - c0 + 1
        grid = np.empty((rows, cols), dtype=np.float64)
        for r in range(rows):
            lat = self.lat0 + r * dlat
            for c in range(cols):
                lon = self.lon0 + c * dlon
                tile, tlon0, tlat_top, tdlon, tdlat = tiles[(math.floor(lat + 1e-9), math.floor(lon + 1e-9))]
                i = round((tlat_top - lat) / tdlat)
                j = round((lon - tlon0) / tdlon)
                grid[r, c] = tile[min(i, tile.shape[0] - 1), min(j, tile.shape[1] - 1)]
        self.grid = grid

    def sample(self, lat, lon) -> float:
        """Bilinear, as the simulation does it."""
        y = (lat - self.lat0) / self.dlat
        x = (lon - self.lon0) / self.dlon
        r, c = int(y), int(x)
        fy, fx = y - r, x - c
        g = self.grid
        south = g[r, c] * (1 - fx) + g[r, c + 1] * fx
        north = g[r + 1, c] * (1 - fx) + g[r + 1, c + 1] * fx
        return south * (1 - fy) + north * fy

    def ground_near(self, lat, lon, radius_m) -> float:
        """Ground level around a building: the 10th percentile of the samples
        within radius_m, which are mostly car park around a mall."""
        values = []
        steps = int(radius_m // 15)
        for i in range(-steps, steps + 1):
            for j in range(-steps, steps + 1):
                if math.hypot(i, j) * 15 <= radius_m:
                    values.append(self.sample(*offset(lat, lon, i * 15, j * 15)))
        return float(np.percentile(values, 10))

    def fields(self) -> dict:
        return {
            "terrain": {
                "source": "Copernicus DEM GLO-30 (DSM), © DLR e.V. 2010-2014 and © Airbus "
                "Defence and Space GmbH 2014-2018, provided under COPERNICUS by the "
                "European Union and ESA; all rights reserved",
                "lat0_e7": e7(self.lat0),
                "lon0_e7": e7(self.lon0),
                "dlat_e7": e7(self.dlat),
                "dlon_e7": e7(self.dlon),
                "rows": self.grid.shape[0],
                "cols": self.grid.shape[1],
                "elev_dm": [int(round(v * 10)) for v in self.grid.ravel()],
            }
        }


def distance_m(lat1, lon1, lat2, lon2) -> float:
    r = 6_371_000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def offset(lat: float, lon: float, north_m: float, east_m: float) -> tuple[float, float]:
    return (
        lat + north_m / 111_320.0,
        lon + east_m / (111_320.0 * math.cos(math.radians(lat))),
    )


def e7(deg: float) -> int:
    return round(deg * 1e7)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--seed", type=int, default=20261003)
    p.add_argument("--cache", type=Path, default=ROOT / ".cache")
    args = p.parse_args()
    osm_cache, dem_cache = args.cache / "kaunas-osm", args.cache / "kaunas-dem"
    osm_cache.mkdir(parents=True, exist_ok=True)
    dem_cache.mkdir(parents=True, exist_ok=True)
    rng = random.Random(args.seed)

    reach = RADIUS_M + TERRAIN_MARGIN_M
    corners = [offset(h["lat"], h["lon"], n * reach, e * reach)
               for h in HUBS for n in (-1, 1) for e in (-1, 1)]
    dem = Dem(min(c[0] for c in corners), min(c[1] for c in corners),
              max(c[0] for c in corners), max(c[1] for c in corners), dem_cache)

    # The address book: every addressed building within RADIUS_M of a hub
    # (and not right next to one), served by the nearer hub, then sampled.
    pool = {}
    for hub in HUBS:
        for a in addresses(hub, osm_cache):
            pool[a["osm"]] = a
    book = []
    for osm in sorted(pool):
        a = pool[osm]
        dists = [distance_m(h["lat"], h["lon"], a["lat"], a["lon"]) for h in HUBS]
        nearest = min(range(len(HUBS)), key=lambda i: dists[i])
        if MIN_RADIUS_M <= dists[nearest] <= RADIUS_M and min(dists) >= MIN_RADIUS_M:
            book.append((a, nearest))
    print(f"{len(book)} addressed buildings within {RADIUS_M / 1000:.0f} km of a hub")
    if ADDRESS_POOL is not None and ADDRESS_POOL < len(book):
        book = sorted(rng.sample(book, ADDRESS_POOL), key=lambda b: b[0]["osm"])
    for h, hub in enumerate(HUBS):
        print(f"  {hub['name']} serves {sum(1 for _, d in book if d == h)} of {len(book)}")

    agents = []
    sysid = 1
    for h, hub in enumerate(HUBS):
        ground = dem.ground_near(hub["lat"], hub["lon"], GROUND_SAMPLE_M)
        roof = round(ground + hub["levels"] * LEVEL_HEIGHT_M, 1)
        print(f"{hub['name']}: ground {ground:.1f} m, roof {roof:.1f} m above sea level")
        agents.append(
            {"id": 100 + h, "kind": "poi", "lon": hub["lon"], "lat": hub["lat"],
             "state": {"name": hub["name"]}}
        )

        def pad_at(slot):
            angle = 2 * math.pi * slot / (DRONES_PER_HUB + 1)
            lat, lon = offset(hub["lat"], hub["lon"],
                              PAD_RING_M * math.cos(angle), PAD_RING_M * math.sin(angle))
            return round(lat, 7), round(lon, 7)

        for d in range(DRONES_PER_HUB):
            pad = pad_at(d)
            agents.append(
                {
                    "id": sysid,
                    "kind": "drone",
                    "lon": pad[1],
                    "lat": pad[0],
                    "state": {
                        "sysid": sysid,
                        "callsign": f"{hub['callsign']}-{d + 1}",
                        "elevation_m": roof,
                        "depot": hub["callsign"],
                        "cruise_agl_m": CRUISE_AGL_M[d % len(CRUISE_AGL_M)],
                    },
                }
            )
            sysid += 1

        # One spare per hub, on its roof pad for a ground station to take. No
        # depot, so dispatch leaves it alone.
        pad = pad_at(DRONES_PER_HUB)
        agents.append(
            {"id": sysid, "kind": "drone", "lon": pad[1], "lat": pad[0],
             "state": {"sysid": sysid, "callsign": f"{hub['callsign']}-SPARE",
                       "elevation_m": roof}}
        )
        sysid += 1

    agents.sort(key=lambda a: a["id"])
    world = PACK / "world"
    world.mkdir(parents=True, exist_ok=True)
    (world / "agents.json").write_text(json.dumps(agents, indent=2, ensure_ascii=False) + "\n")
    fields = dem.fields()
    # One row of samples per line keeps the file diffable without being huge.
    t = fields["terrain"]
    head = json.dumps({k: v for k, v in t.items() if k != "elev_dm"}, ensure_ascii=False)[1:-1]
    rows = [json.dumps(t["elev_dm"][r * t["cols"]:(r + 1) * t["cols"]], separators=(",", ":"))[1:-1]
            for r in range(t["rows"])]
    depots = json.dumps([h["callsign"] for h in HUBS])
    items = [json.dumps([e7(a["lat"]), e7(a["lon"]), d, a["label"]], ensure_ascii=False)
             for a, d in book]
    (world / "fields.json").write_text(
        '{"addresses": {"source": "OpenStreetMap contributors, ODbL 1.0", "depots": '
        + depots + ', "items": [\n' + ",\n".join(items) + "\n]},\n"
        + '"terrain": {' + head + ', "elev_dm": [\n' + ",\n".join(rows) + "\n]}}\n"
    )
    drones = sum(a["kind"] == "drone" for a in agents)
    print(f"wrote {world.relative_to(ROOT)}: {drones} drones, {len(book)} addresses, "
          f"terrain {t['rows']} × {t['cols']} samples "
          f"({t['dlat_e7'] / 1e7 * 3600:.1f}\" × {t['dlon_e7'] / 1e7 * 3600:.1f}\")")


if __name__ == "__main__":
    main()
