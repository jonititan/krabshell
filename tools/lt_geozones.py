# /// script
# requires-python = ">=3.11"
# dependencies = ["shapely>=2"]
# ///
"""Add Lithuania's published UAS geographical zones to scenarios/kaunas-deliveries.

Oro navigacija, Lithuania's air navigation service provider, publishes every
UAS geographical zone (Regulation (EU) 2019/947, Article 15) on its UTM
platform, as ED-269 properties on GeoJSON polygons. This takes the zones over
the area the pack's drones fly in and writes them to world/geofences.json as
keep-out fences, beside the fences drawn by hand (entries without a "zone"
key), which it leaves alone.

The simulation's fences are closed at every altitude and have no
authorisations, so a zone becomes a fence when:

- its restriction is PROHIBITED or REQ_AUTHORISATION (a delivery operator
  without the authorisation stays out); NO_RESTRICTION zones are notices;
- its vertical band overlaps the drones' cruise heights above the ground
  (cruise_agl_m in world/agents.json), so zones that start high above them,
  or stop below them, are left out;
- it, grown by geofence_margin_m (params.toml), keeps clear of every drone's
  pad, because a fence over a pad stops the pack loading. Zones that fail
  this are listed, not written.

Each fence is the zone clipped to the area the addresses span (plus CLIP_M)
and simplified to SIMPLIFY_M, which keeps routing cheap; the drones fly round
a polygon's convex hull anyway. The download is cached under .cache/; pass
--refresh to fetch the zones again.

    uv run tools/lt_geozones.py
    uv run tools/lt_geozones.py --refresh

Zone data: Oro navigacija (ANS Lithuania), https://utm.ans.lt/avm/.
"""

import argparse
import json
import math
import re
import tomllib
import urllib.request
from datetime import date
from pathlib import Path

from shapely.geometry import MultiPolygon, Point, Polygon, box, shape
from shapely.ops import transform

ROOT = Path(__file__).resolve().parent.parent
PACK = ROOT / "scenarios" / "kaunas-deliveries"
CACHE = ROOT / ".cache" / "lt-uas-zones.geojson"
ZONES_URL = "https://utm.ans.lt/avm/utm/uas.geojson"
# Clip zones this far beyond the addresses' bounding box, metres.
CLIP_M = 300.0
# Simplification tolerance, metres.
SIMPLIFY_M = 3.0
# Extra clearance round a pad beyond geofence_margin_m, metres.
PAD_SLACK_M = 5.0
KEEP = {"PROHIBITED": "prohibited", "REQ_AUTHORISATION": "authorisation"}
FT = 0.3048


def fetch(refresh: bool) -> dict:
    if refresh or not CACHE.exists():
        CACHE.parent.mkdir(parents=True, exist_ok=True)
        req = urllib.request.Request(ZONES_URL, headers={"User-Agent": "krabshell"})
        with urllib.request.urlopen(req, timeout=120) as r:
            CACHE.write_bytes(r.read())
    return json.loads(CACHE.read_text())


class Local:
    """Metres east and north of a reference point, and back."""

    def __init__(self, lon0: float, lat0: float):
        self.lon0, self.lat0 = lon0, lat0
        self.kx = 111_320.0 * math.cos(math.radians(lat0))
        self.ky = 110_574.0

    def to_m(self, g):
        return transform(lambda x, y, z=None: ((x - self.lon0) * self.kx, (y - self.lat0) * self.ky), g)

    def to_deg(self, g):
        return transform(lambda x, y, z=None: (x / self.kx + self.lon0, y / self.ky + self.lat0), g)


def band_m(v: dict) -> tuple[float, float]:
    k = FT if v.get("uomDimensions") == "FT" else 1.0
    return v["lowerLimit"] * k, v["upperLimit"] * k


def name_of(p: dict) -> str:
    """The zone's id and who grants flights in it, at most 80 characters."""
    msgs = p.get("extendedProperties", {}).get("localizedMessages", [])
    text = next((m["message"] for m in msgs if m["language"] == "en-GB"), p.get("message") or "")
    text = re.sub(r"<[^>]+>", " ", text)
    who = re.search(
        r"(?:permission|authori[sz]ation) from (?:the )?(.+?)(?: and |[.(,]|$)", text
    )
    contact = re.search(r"Contact:\s*(\S+@\S+)", text)
    what = KEEP[p["restriction"]]
    if who and "responsible author" not in who.group(1):
        tail = who.group(1).strip()
    elif contact:
        tail = contact.group(1).split("@")[1]
    else:
        tail = "" if p.get("reason") in (None, "", "Other") else p["reason"].lower()
    name = f"{p['identifier']}: {what}" + (f", {tail}" if tail else "")
    return name[:80]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--refresh", action="store_true", help="download the zones again")
    args = ap.parse_args()

    agents = json.loads((PACK / "world" / "agents.json").read_text())
    drones = [a for a in agents if a["kind"] == "drone"]
    cruise = [a["state"]["cruise_agl_m"] for a in drones if "cruise_agl_m" in a["state"]]
    lo, hi = min(cruise), max(cruise)
    params = tomllib.loads((PACK / "params.toml").read_text())
    margin = params.get("geofence_margin_m", 30.0) + PAD_SLACK_M
    items = json.loads((PACK / "world" / "fields.json").read_text())["addresses"]["items"]
    lats = [i[0] * 1e-7 for i in items]
    lons = [i[1] * 1e-7 for i in items]

    local = Local((min(lons) + max(lons)) / 2, (min(lats) + max(lats)) / 2)
    area = local.to_m(box(min(lons), min(lats), max(lons), max(lats))).buffer(CLIP_M, join_style="mitre")
    pads = [local.to_m(Point(a["lon"], a["lat"])) for a in drones]

    data = fetch(args.refresh)
    fences, skipped = [], []
    for ft in data["features"]:
        p = ft["properties"]
        g = local.to_m(shape(ft["geometry"]))
        if not g.intersects(area):
            continue
        low, high = band_m(p["geometry"])
        if p["restriction"] not in KEEP:
            skipped.append((p["identifier"], f"restriction {p['restriction']}"))
            continue
        if low > hi or high < lo:
            skipped.append((p["identifier"], f"{low:.0f}-{high:.0f} m, clear of cruise at {lo:.0f}-{hi:.0f} m"))
            continue
        clipped = g.intersection(area)
        parts = clipped.geoms if isinstance(clipped, MultiPolygon) else [clipped]
        for i, part in enumerate(parts):
            if not isinstance(part, Polygon) or part.area < 1.0:
                continue
            # Fences have no holes: drop them (the drones fly the hull anyway).
            part = Polygon(part.exterior).simplify(SIMPLIFY_M, preserve_topology=True)
            if any(part.convex_hull.buffer(margin).contains(q) for q in pads):
                skipped.append((p["identifier"], "covers a drone's pad"))
                continue
            ring = list(local.to_deg(part).exterior.coords)[:-1]
            # Counter-clockwise, as GeoJSON has it.
            if Polygon(ring).exterior.is_ccw is False:
                ring.reverse()
            name = name_of(p) if len(parts) == 1 else f"{name_of(p)[:76]} ({i + 1})"
            fences.append(
                {
                    "name": name,
                    "zone": p["identifier"],
                    "polygon": [[round(x, 6), round(y, 6)] for x, y in ring],
                }
            )

    path = PACK / "world" / "geofences.json"
    drawn = [f for f in json.loads(path.read_text()) if "zone" not in f]
    fences.sort(key=lambda f: f["name"])
    path.write_text(json.dumps(drawn + fences, ensure_ascii=False, indent=2) + "\n")

    print(f"{len(fences)} zone fences ({date.today()}), {len(drawn)} drawn by hand, written to {path.relative_to(ROOT)}")
    for f in fences:
        print(f"  + {f['name']} ({len(f['polygon'])} corners)")
    for zone, why in sorted(set(skipped)):
        print(f"  - {zone}: {why}")


if __name__ == "__main__":
    main()
