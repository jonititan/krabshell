#!/usr/bin/env python3
"""Draw the sim-conflict figures in docs/figures from the crate's own data.

    cargo run --release -p sim-conflict --example figures > target/conflict-figures.json
    python3 tools/conflict_figures.py target/conflict-figures.json docs/figures

Standard library only. Each figure is a self-contained SVG that follows the
reader's light or dark preference, on a transparent background so it sits on
whatever page shows it.
"""

import json
import math
import sys
from pathlib import Path

R = 6_371_008.8  # the crate's Earth radius, m

# Roles, light then dark. Categorical slots 1 and 2 and the blue ramp are the
# dataviz reference palette; chrome is its ink and hairlines.
TOKENS = {
    "ink": ("#0b0b0b", "#f0efec"),
    "ink2": ("#52514e", "#c3c2b7"),
    "muted": ("#898781", "#898781"),
    "grid": ("#e1e0d9", "#2c2c2a"),
    "axis": ("#c3c2b7", "#383835"),
    "surface": ("#ffffff", "#171b21"),
    "box": ("#f6f7f9", "#1f242b"),
    "s1": ("#2a78d6", "#3987e5"),
    "s2": ("#eb6834", "#d95926"),
    "bad": ("#d03b3b", "#e66767"),
    "cost": ("#c3c2b7", "#5a5955"),
    # Sequential blue, binned: q1 (least) .. q5 (most). Dark mode runs the
    # other way so "nothing" recedes into the dark surface.
    "q1": ("#cde2fb", "#184f95"),
    "q2": ("#9ec5f4", "#1c5cab"),
    "q3": ("#6da7ec", "#2a78d6"),
    "q4": ("#3987e5", "#6da7ec"),
    "q5": ("#1c5cab", "#b7d3f6"),
    # Ordinal: the cloud's steps, early to late.
    "o1": ("#86b6ef", "#184f95"),
    "o2": ("#3987e5", "#256abf"),
    "o3": ("#1c5cab", "#5598e7"),
    "o4": ("#0d366b", "#b7d3f6"),
}


def style():
    light = ";".join(f"--{k}:{v[0]}" for k, v in TOKENS.items())
    dark = ";".join(f"--{k}:{v[1]}" for k, v in TOKENS.items())
    return f"""<style>
svg{{{light};font-family:system-ui,-apple-system,"Segoe UI",sans-serif}}
@media (prefers-color-scheme:dark){{svg{{{dark}}}}}
text{{fill:var(--ink);font-size:13px}}
.t2{{fill:var(--ink2)}} .tm{{fill:var(--muted);font-size:12px;font-variant-numeric:tabular-nums}}
.h{{font-weight:600}} .small{{font-size:12px}}
.halo{{paint-order:stroke;stroke:var(--surface);stroke-width:4px;stroke-linejoin:round}}
.grid{{stroke:var(--grid);stroke-width:1}} .axis{{stroke:var(--axis);stroke-width:1}}
</style>"""


class Svg:
    def __init__(self, w, h, title, desc):
        self.w, self.h = w, h
        self.parts = [
            f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" '
            f'width="{w}" height="{h}" role="img" aria-labelledby="t d">',
            f"<title id=\"t\">{esc(title)}</title><desc id=\"d\">{esc(desc)}</desc>",
            style(),
        ]

    def add(self, s):
        self.parts.append(s)

    def text(self, x, y, s, cls="", anchor="start", extra=""):
        c = f' class="{cls}"' if cls else ""
        self.add(f'<text x="{x:.1f}" y="{y:.1f}" text-anchor="{anchor}"{c}{extra}>{esc(s)}</text>')

    def line(self, x1, y1, x2, y2, stroke, width=1, extra=""):
        self.add(
            f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" '
            f'stroke="var(--{stroke})" stroke-width="{width}"{extra}/>'
        )

    def save(self, path):
        self.add("</svg>")
        Path(path).write_text("\n".join(self.parts) + "\n")


def esc(s):
    return str(s).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def fmt(v):
    """Thousands with a thin space, as the docs write them."""
    v = round(v)
    return f"{v:,}".replace(",", " ") if abs(v) >= 10_000 else str(v)


def nice_ticks(lo, hi, n=5):
    span = hi - lo
    raw = span / n
    mag = 10 ** math.floor(math.log10(raw))
    step = min((m * mag for m in (1, 2, 2.5, 5, 10) if m * mag >= raw), default=10 * mag)
    first = math.ceil(lo / step) * step
    out, v = [], first
    while v <= hi + 1e-9:
        out.append(round(v, 10))
        v += step
    return out


CLIPS = 0


class Plane:
    """A plan view in metres east and north of a reference, equal scale."""

    def __init__(self, ref, x0, y0, w, h, bounds):
        self.lat0, self.lon0 = ref
        self.cos = math.cos(math.radians(self.lat0))
        (ex0, ny0), (ex1, ny1) = bounds
        k = min(w / (ex1 - ex0), h / (ny1 - ny0))
        self.k = k
        # Centre the extent in the box.
        self.ox = x0 + (w - (ex1 - ex0) * k) / 2 - ex0 * k
        self.oy = y0 + (h - (ny1 - ny0) * k) / 2 + ny1 * k
        self.box = (x0, y0, w, h)
        self.ext = bounds

    def en(self, lat, lon):
        return (
            math.radians(lon - self.lon0) * R * self.cos,
            math.radians(lat - self.lat0) * R,
        )

    def xy(self, lat, lon):
        e, n = self.en(lat, lon)
        return self.ox + e * self.k, self.oy - n * self.k

    def px(self, e, n):
        return self.ox + e * self.k, self.oy - n * self.k

    def path(self, ring):
        pts = [self.xy(lat, lon) for lon, lat in ring]
        return "M" + "L".join(f"{x:.1f},{y:.1f}" for x, y in pts) + "Z"

    def frame(self, svg, label_axes=True, step=None):
        """Hairline grid in metres, with ticks on the left and bottom."""
        (e0, n0), (e1, n1) = self.ext
        x0, y0, w, h = self.box
        # Numbered in drawing order, so a figure drawn again from the same
        # data is the same file.
        global CLIPS
        CLIPS += 1
        clip = f"clip{CLIPS}"
        svg.add(f'<clipPath id="{clip}"><rect x="{x0}" y="{y0}" width="{w}" height="{h}"/></clipPath>')
        lo_e, hi_e = (x0 - self.ox) / self.k, (x0 + w - self.ox) / self.k
        lo_n, hi_n = (self.oy - y0 - h) / self.k, (self.oy - y0) / self.k
        ticks_e = nice_ticks(lo_e, hi_e, 5) if step is None else frange(lo_e, hi_e, step)
        ticks_n = nice_ticks(lo_n, hi_n, 4) if step is None else frange(lo_n, hi_n, step)
        for e in ticks_e:
            x, _ = self.px(e, 0)
            svg.line(x, y0, x, y0 + h, "grid")
            if label_axes:
                svg.text(x, y0 + h + 15, fmt(e), "tm", "middle")
        for n in ticks_n:
            _, y = self.px(0, n)
            svg.line(x0, y, x0 + w, y, "grid")
            if label_axes:
                svg.text(x0 - 6, y + 4, fmt(n), "tm", "end")
        svg.add(f'<rect x="{x0}" y="{y0}" width="{w}" height="{h}" fill="none" stroke="var(--axis)"/>')
        return clip


def frange(lo, hi, step):
    v = math.ceil(lo / step) * step
    out = []
    while v <= hi + 1e-9:
        out.append(v)
        v += step
    return out


def legend_swatches(svg, x, y, items, size=12):
    """items: (fill var, label, kind) with kind 'box', 'ring', 'dot' or 'line'."""
    for fill, label, kind in items:
        if kind == "box":
            svg.add(f'<rect x="{x}" y="{y - size + 2}" width="{size}" height="{size}" rx="2" fill="var(--{fill})"/>')
        elif kind == "ring":
            svg.add(
                f'<rect x="{x + 1}" y="{y - size + 3}" width="{size - 2}" height="{size - 2}" '
                f'fill="none" stroke="var(--{fill})" stroke-width="2"/>'
            )
        elif kind == "dot":
            svg.add(f'<circle cx="{x + size / 2}" cy="{y - size / 2 + 2}" r="3" fill="var(--{fill})"/>')
        elif kind == "line":
            svg.line(x, y - size / 2 + 2, x + size + 6, y - size / 2 + 2, fill, 2)
            x += 6
        elif kind == "dash":
            svg.line(x, y - size / 2 + 2, x + size + 6, y - size / 2 + 2, fill, 2, ' stroke-dasharray="5 3"')
            x += 6
        svg.text(x + size + 6, y, label, "small t2")
        x += size + 14 + 7 * len(label)
    return x


# ------------------------------------------------------------- figures


def pipeline(d, out):
    w, h = 780, 200
    svg = Svg(
        w, h, "How sim-conflict assesses aircraft",
        "Aircraft states go through three stages: predict each aircraft as a cloud of "
        "particles counted into cells and layers; detect cells where two or more aircraft "
        "sum to the threshold; resolve by predicting each candidate manoeuvre again and "
        "advising the one that removes most risk, repeating until nothing is in conflict.",
    )
    cfg = d["config"]
    boxes = [
        ("Aircraft states", ["position, height,", "speed, heading,", "climb and turn"], None),
        ("1  Predict", [f"{cfg['particles']} particles,", f"{cfg['steps']} steps of {cfg['step_s']:.0f} s,",
                        "counted into cells"], "s1"),
        ("2  Detect", ["sum each cell;", "2+ aircraft and", f"sum ≥ {cfg['threshold']:.0f}: conflict"], "s1"),
        ("3  Resolve", ["try 7 manoeuvres,", "advise the best,", "then again"], "s1"),
        ("Advisories", ["and the pairs", "left unresolved"], None),
    ]
    bw, bh, gap, top = 136, 90, 24, 12
    x = (w - (5 * bw + 4 * gap)) / 2
    centres = []
    for i, (title, lines, accent) in enumerate(boxes):
        stroke = f"var(--{accent})" if accent else "var(--axis)"
        width = 1.5 if accent else 1
        svg.add(f'<rect x="{x}" y="{top}" width="{bw}" height="{bh}" rx="8" fill="var(--box)" '
                f'stroke="{stroke}" stroke-width="{width}"/>')
        svg.text(x + 12, top + 24, title, "h")
        for j, ln in enumerate(lines):
            svg.text(x + 12, top + 46 + j * 15, ln, "small t2")
        centres.append(x + bw / 2)
        if i < 4:
            ax = x + bw + 3
            svg.line(ax, top + bh / 2, ax + gap - 9, top + bh / 2, "ink2", 1.5)
            svg.add(f'<path d="M{ax + gap - 3},{top + bh / 2} l-7,-4 v8z" fill="var(--ink2)"/>')
        x += bw + gap
    # Resolve loops back to predict.
    xr, xp, yb = centres[3], centres[1], top + bh
    svg.add(f'<path d="M{xr},{yb + 2} C{xr},{yb + 62} {xp},{yb + 62} {xp},{yb + 8}" fill="none" '
            f'stroke="var(--s1)" stroke-width="1.5"/>')
    svg.add(f'<path d="M{xp},{yb + 2} l-4,8 h8z" fill="var(--s1)"/>')
    svg.text((xr + xp) / 2, yb + 62, "predict again, flying each candidate manoeuvre", "small t2", "middle")
    svg.save(out / "conflict-pipeline.svg")


def cloud(d, out):
    c = d["cloud"]
    w, h = 760, 420
    svg = Svg(
        w, h, "One drone's particle cloud over a minute",
        "A drone at 10 m/s turning gently right. 200 particles start round its reported "
        "position and spread as heading, speed and wind errors build: a few metres at 5 s, "
        "tens of metres by 60 s. The line is the particles' mean.",
    )
    ref = c["start"]
    plane0 = Plane(ref, 0, 0, 1, 1, ((0, 0), (1, 1)))
    pts = [plane0.en(*p) for s in c["steps"] for p in s["particles"]] + [(0, 0)]
    es, ns = [p[0] for p in pts], [p[1] for p in pts]
    pad = 40
    bounds = ((min(es) - pad, min(ns) - pad), (max(es) + pad, max(ns) + pad))
    plane = Plane(ref, 64, 20, w - 84, h - 92, bounds)
    clip = plane.frame(svg)
    svg.add(f'<g clip-path="url(#{clip})">')
    mean = [plane.xy(*ref)] + [plane.xy(*m) for m in c["mean"]]
    svg.add('<path d="M' + "L".join(f"{x:.1f},{y:.1f}" for x, y in mean) +
            '" fill="none" stroke="var(--ink2)" stroke-width="1.5"/>')
    for i, step in enumerate(c["steps"]):
        dots = "".join(
            f'<circle cx="{x:.1f}" cy="{y:.1f}" r="1.9"/>'
            for x, y in (plane.xy(*p) for p in step["particles"])
        )
        svg.add(f'<g fill="var(--o{i + 1})" fill-opacity="0.85">{dots}</g>')
    x, y = plane.xy(*ref)
    svg.add(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="5" fill="var(--surface)" stroke="var(--ink)" stroke-width="2"/>')
    svg.text(x + 9, y + 18, "now", "small")
    for i, step in enumerate(c["steps"]):
        lat = sum(p[0] for p in step["particles"]) / len(step["particles"])
        lon = sum(p[1] for p in step["particles"]) / len(step["particles"])
        px, py = plane.xy(lat, lon)
        svg.text(px, py - 14 - 2 * i, f"{step['t']:.0f} s", "small h halo", "middle")
    svg.add("</g>")
    x0, y0, bw, bh = plane.box
    svg.text(x0 + bw / 2, y0 + bh + 32, "metres east", "small t2", "middle")
    svg.text(16, y0 + bh / 2, "metres north", "small t2", "middle",
             f' transform="rotate(-90 16 {y0 + bh / 2})"')
    legend_swatches(svg, x0, h - 8, [(f"o{i + 1}", f"{s['t']:.0f} s ahead", "dot")
                                     for i, s in enumerate(c["steps"])] + [("ink2", "mean", "line")])
    svg.save(out / "conflict-cloud.svg")


def bins(p, edges):
    for i, e in enumerate(edges):
        if p <= e + 1e-9:
            return i + 1
    return len(edges)


def cells(d, out):
    panels = d["cells"]
    w, h = 760, 400
    svg = Svg(
        w, h, "The cloud counted into cells, at two resolutions",
        "The same drone's cloud 30 s ahead. Each particle covers its own A5 cell and one "
        "ring round it (its protected zone), so a cell's probability is the share of "
        "particles whose zone covers it. At resolution 15 (cells about 180 m) the core "
        "cells are certain; at 17 (about 44 m) the probability spreads over more, smaller cells.",
    )
    ref = panels[0]["mean"]
    plane0 = Plane(ref, 0, 0, 1, 1, ((0, 0), (1, 1)))
    pts = [plane0.en(lat, lon) for p in panels for c in p["cells"] for lon, lat in c["ring"]]
    es, ns = [p[0] for p in pts], [p[1] for p in pts]
    bounds = ((min(es) - 10, min(ns) - 10), (max(es) + 10, max(ns) + 10))
    edges = [0.2, 0.4, 0.6, 0.8, 1.0]
    pw = (w - 64 - 40 - 24) / 2
    for i, p in enumerate(panels):
        x0 = 64 + i * (pw + 40)
        plane = Plane(ref, x0, 44, pw, h - 120, bounds)
        clip = plane.frame(svg, label_axes=True)
        svg.add(f'<g clip-path="url(#{clip})">')
        for c in sorted(p["cells"], key=lambda c: c["p"]):
            q = bins(c["p"], edges)
            svg.add(f'<path d="{plane.path(c["ring"])}" fill="var(--q{q})" stroke="var(--surface)" stroke-width="1.5"/>')
        dots = "".join(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="1.4"/>'
                       for x, y in (plane.xy(*q) for q in p["particles"]))
        svg.add(f'<g fill="var(--ink)" fill-opacity="0.75">{dots}</g>')
        svg.add("</g>")
        svg.text(x0, 16, f"Resolution {p['resolution']}: cells about {p['edge_m']:.0f} m", "h")
        svg.text(x0, 34, f"{len(p['cells'])} cells covered, the most likely {max(c['p'] for c in p['cells']):.2f}",
                 "small t2")
        svg.text(x0 + pw / 2, h - 46, "metres east of the cloud's mean", "small t2", "middle")
    labels = ["up to 0.2", "0.2 to 0.4", "0.4 to 0.6", "0.6 to 0.8", "0.8 to 1"]
    x = legend_swatches(svg, 64, h - 12, [(f"q{i + 1}", lab, "box") for i, lab in enumerate(labels)])
    legend_swatches(svg, x + 6, h - 12, [("ink", "particles", "dot")])
    svg.text(16, 44 + (h - 120) / 2, "metres north", "small t2", "middle",
             f' transform="rotate(-90 16 {44 + (h - 120) / 2})"')
    svg.save(out / "conflict-cells.svg")


def layers(d, out):
    L = d["layers"]
    w, h = 760, 420
    svg = Svg(
        w, h, "Altitude layers",
        "Left: the airspace's standard layers, 1 000 ft deep below 28 500 ft and 2 000 ft "
        "from there up, placed so each flight level is inside its layer. Right: the drones' "
        "30 m layers; a drone at 60 m with its 15 m vertical buffer covers two layers.",
    )
    top, bot = 52, h - 24

    def column(x0, cw, lo, hi, bands, ylab, title, sub):
        k = (bot - top) / (hi - lo)
        Y = lambda v: bot - (v - lo) * k
        svg.text(x0, 18, title, "h")
        svg.text(x0, 36, sub, "small t2")
        for i, (a, b) in enumerate(bands):
            a, b = max(a, lo), min(b, hi)
            fill = "var(--box)" if i % 2 == 0 else "transparent"
            svg.add(f'<rect x="{x0}" y="{Y(b):.1f}" width="{cw}" height="{Y(a) - Y(b):.1f}" fill="{fill}"/>')
            svg.line(x0, Y(a), x0 + cw, Y(a), "axis")
        svg.line(x0, Y(hi), x0 + cw, Y(hi), "axis")
        for v in ylab:
            svg.text(x0 - 6, Y(v) + 4, fmt(v), "tm", "end")
        return Y

    # Standard layers, in feet.
    std = [tuple(b) for b in L["standard_ft"]]
    lo, hi = std[0][0], std[-1][1]
    x0, cw = 92, 300
    Y = column(x0, cw, lo, hi, std, [b[0] for b in std] + [hi], "Airspace layers",
               "feet, the airliners preset")
    for a, b in std:
        fl = round((a + 500) / 1000) * 10
        svg.line(x0 + 40, Y(fl * 100), x0 + 64, Y(fl * 100), "s1", 2)
        svg.text(x0 + 70, Y(fl * 100) + 4, f"FL{fl:03d}", "small")
        svg.text(x0 + cw - 8, Y((a + b) / 2) + 4, f"{(b - a):,.0f} ft".replace(",", " "), "tm", "end")
    yb = Y(28_500)
    svg.line(x0 - 4, yb, x0 + cw + 4, yb, "ink", 1.5)
    svg.text(x0 + 140, yb - 6, "bands meet at 28 500 ft", "small h halo")

    # Drone layers, in metres.
    dl = [tuple(b) for b in L["drones_m"]]
    x1, cw1 = 480, 270
    Y = column(x1, cw1, dl[0][0], dl[-1][1], dl, [b[0] for b in dl] + [dl[-1][1]],
               "Drone layers", "metres, the drones preset")
    for i, (a, b) in enumerate(dl):
        svg.text(x1 + 8, Y((a + b) / 2) + 4, f"layer {i}", "tm")
    buf = d["encounter"]["vertical_buffer_m"]
    alt = 60
    bx = x1 + 100
    svg.add(f'<rect x="{bx - 18}" y="{Y(alt + buf):.1f}" width="36" height="{Y(alt - buf) - Y(alt + buf):.1f}" '
            f'rx="3" fill="var(--s1)" fill-opacity="0.18" stroke="var(--s1)"/>')
    svg.add(f'<circle cx="{bx}" cy="{Y(alt):.1f}" r="5" fill="var(--s1)" stroke="var(--surface)" stroke-width="2"/>')
    svg.text(bx + 26, Y(alt) - 4, "drone at 60 m", "small")
    svg.text(bx + 26, Y(alt) + 12, f"± {buf:.0f} m buffer covers", "small t2")
    svg.text(bx + 26, Y(alt) + 27, "layers 1 and 2", "small t2")
    svg.save(out / "conflict-layers.svg")


def detection(d, out):
    e = d["encounter"]
    w, h = 760, 360
    svg = Svg(
        w, h, "Two drones head-on: summed probability by cell",
        "Two drones 1.2 km apart flying at each other at 60 m and 10 m/s. Each cell is "
        "shaded by the probabilities both drones put in it, summed. 20 s ahead their clouds "
        "are apart; 40 s ahead one cell holds 1.31 aircraft and is a conflict; 60 s ahead "
        "six cells are.",
    )
    ref = e["starts"][0]
    plane0 = Plane(ref, 0, 0, 1, 1, ((0, 0), (1, 1)))
    pts = [plane0.en(lat, lon) for s in e["heat"] for c in s["cells"] for lon, lat in c["ring"]]
    es, ns = [p[0] for p in pts], [p[1] for p in pts]
    bounds = ((min(es) - 20, min(ns) - 20), (max(es) + 20, max(ns) + 20))
    edges = [0.5, 0.999, 1.5, 2.0]
    qmap = {1: 1, 2: 2, 3: 4, 4: 5}
    pw = (w - 56 - 2 * 16 - 16) / 3
    for i, step in enumerate(e["heat"]):
        x0 = 56 + i * (pw + 16)
        plane = Plane(ref, x0, 48, pw, h - 118, bounds)
        clip = plane.frame(svg, label_axes=(i == 0), step=500)
        if i:
            for t in frange(*sorted(((plane.box[0] - plane.ox) / plane.k,
                                     (plane.box[0] + pw - plane.ox) / plane.k)), 500):
                x, _ = plane.px(t, 0)
                svg.text(x, plane.box[1] + plane.box[3] + 15, fmt(t), "tm", "middle")
        svg.add(f'<g clip-path="url(#{clip})">')
        for c in sorted(step["cells"], key=lambda c: c["summed"]):
            q = qmap[bins(c["summed"], edges)]
            svg.add(f'<path d="{plane.path(c["ring"])}" fill="var(--q{q})" stroke="var(--surface)" stroke-width="1.5"/>')
        for c in step["cells"]:
            if c["conflict"]:
                svg.add(f'<path d="{plane.path(c["ring"])}" fill="none" stroke="var(--bad)" stroke-width="2.5"/>')
        for j, (lat, lon) in enumerate(step["means"]):
            x, y = plane.xy(lat, lon)
            tri = f"M{x + 7},{y} l-12,-6 v12z" if j == 0 else f"M{x - 7},{y} l12,-6 v12z"
            svg.add(f'<path d="{tri}" fill="var(--ink)" stroke="var(--surface)" stroke-width="1.5"/>')
            svg.text(x, y - 12 if j == 0 else y + 22, f"UAV {j + 1}", "small h halo", "middle")
        svg.add("</g>")
        n = sum(c["conflict"] for c in step["cells"])
        top = max(c["summed"] for c in step["cells"])
        svg.text(x0, 16, f"{step['t']:.0f} s ahead", "h")
        svg.text(x0, 34, f"peak {top:.2f}" + (f", {n} cell{'s' if n > 1 else ''} in conflict" if n else ", no conflict"), "small t2")
    svg.text(w / 2, h - 38, "metres east of where UAV 1 is now", "small t2", "middle")
    x = legend_swatches(svg, 56, h - 10, [("q1", "summed under 0.5", "box"), ("q2", "0.5 to 1", "box"),
                                          ("q4", "1 to 1.5", "box"), ("q5", "1.5 to 2", "box")])
    legend_swatches(svg, x + 4, h - 10, [("bad", "in conflict (2 aircraft, sum ≥ 1)", "ring")])
    svg.save(out / "conflict-detection.svg")


def protection(d, out):
    P = d["protection"]
    w, h = 760, 460
    svg = Svg(
        w, h, "A manned aircraft's protected volume on the grid",
        f"An airliner at {P['alt_m']:.0f} m. Left, from above: the A5 cells (resolution "
        f"{P['resolution']}, about {P['edge_m']:.0f} m) whose centres are within "
        f"{P['lateral_m']:.0f} m of it make its volume. A drone counts only the cell it is in: "
        f"one 230 m away is inside, one 450 m away is outside, and the cell and ring it keeps "
        f"from other drones do not count against the airliner. Right, from the side: the 30 m "
        f"layers within {P['vertical_m']:.0f} m above and below, {P['covered_m'][0]:.0f} to "
        f"{P['covered_m'][1]:.0f} m.",
    )
    ref = P["at"]
    # Plan view.
    pw, ph = 430, h - 120
    x0, y0 = 56, 50
    plane0 = Plane(ref, 0, 0, 1, 1, ((0, 0), (1, 1)))
    pts = [plane0.en(lat, lon) for c in P["cells"] for lon, lat in c["ring"]]
    half = max(max(abs(e) for e, _ in pts), max(abs(n) for _, n in pts)) * 0.82
    plane = Plane(ref, x0, y0, pw, ph, ((-half, -half), (half, half)))
    clip = plane.frame(svg, label_axes=True, step=200)
    svg.add(f'<g clip-path="url(#{clip})">')
    for c in P["cells"]:
        fill = "var(--q2)" if c["in"] else "transparent"
        svg.add(f'<path d="{plane.path(c["ring"])}" fill="{fill}" stroke="var(--axis)" stroke-width="1"/>')
    # The nominal minimum, a circle round the airliner.
    cx, cy = plane.px(0, 0)
    svg.add(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{P["lateral_m"] * plane.k:.1f}" fill="none" '
            f'stroke="var(--s2)" stroke-width="2" stroke-dasharray="6 4"/>')
    inside, outside = P["drones"]
    for zc in outside["zone"]:
        svg.add(f'<path d="{plane.path(zc)}" fill="none" stroke="var(--s1)" stroke-width="1.5" stroke-dasharray="3 3"/>')
    for dr, col in ((inside, "bad"), (outside, "s1")):
        svg.add(f'<path d="{plane.path(dr["cell"])}" fill="none" stroke="var(--{col})" stroke-width="2.5"/>')
        x, y = plane.xy(*dr["at"])
        svg.add(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="5" fill="var(--{col})" stroke="var(--surface)" stroke-width="2"/>')
    svg.add(f'<path d="M{cx - 9},{cy} l18,-6 v12z" fill="var(--ink)" stroke="var(--surface)" stroke-width="1.5"/>')
    svg.add("</g>")
    xi, yi = plane.xy(*inside["at"])
    svg.text(xi + 10, yi + 18, f"drone {inside['distance_m']:.0f} m off: in", "small h halo")
    xo, yo = plane.xy(*outside["at"])
    svg.text(xo - 10, yo - 12, f"{outside['distance_m']:.0f} m off: out", "small h halo", "end")
    svg.text(cx + 12, cy - 10, "airliner", "small h halo")
    svg.text(x0, 18, "From above", "h")
    svg.text(x0, 36, f"{P['volume_cells']} cells with centres within {P['lateral_m']:.0f} m: a drone's cell counts "
             f"from {P['nearest_out_m']:.0f} to {P['farthest_in_m']:.0f} m out", "small t2")
    svg.text(x0 + pw / 2, y0 + ph + 34, "metres east of the airliner", "small t2", "middle")
    svg.text(16, y0 + ph / 2, "metres north", "small t2", "middle", f' transform="rotate(-90 16 {y0 + ph / 2})"')

    # Side view: the layers.
    sx, sw = 560, 170
    lo, hi = P["layers"][0]["from"], P["layers"][-1]["to"]
    k = ph / (hi - lo)
    Y = lambda v: y0 + ph - (v - lo) * k
    for L in P["layers"]:
        fill = "var(--q2)" if L["in"] else "transparent"
        svg.add(f'<rect x="{sx}" y="{Y(L["to"]):.1f}" width="{sw}" height="{(L["to"] - L["from"]) * k:.1f}" '
                f'fill="{fill}" stroke="var(--axis)"/>')
    for L in P["layers"][::2] + [P["layers"][-1]]:
        svg.text(sx - 6, Y(L["from"]) + 4, fmt(L["from"]), "tm", "end")
    for v in (P["alt_m"] - P["vertical_m"], P["alt_m"] + P["vertical_m"]):
        svg.line(sx - 2, Y(v), sx + sw + 2, Y(v), "s2", 2, ' stroke-dasharray="6 4"')
    ay = Y(P["alt_m"])
    svg.add(f'<path d="M{sx + 40 - 9},{ay} l18,-6 v12z" fill="var(--ink)" stroke="var(--surface)" stroke-width="1.5"/>')
    svg.text(sx + 56, ay + 4, f"airliner {P['alt_m']:.0f} m", "small h halo")
    for dr, col, dx in ((inside, "bad", 110), (outside, "s1", 140)):
        yy = Y(dr["alt_m"])
        svg.add(f'<circle cx="{sx + dx}" cy="{yy:.1f}" r="5" fill="var(--{col})" stroke="var(--surface)" stroke-width="2"/>')
        svg.text(sx + dx - 9, yy + 4, f"{dr['alt_m'] - P['alt_m']:+.0f} m", "small h halo", "end")
    svg.text(sx, 18, "From the side", "h")
    svg.text(sx, 36, f"layers {P['covered_m'][0]:.0f} to {P['covered_m'][1]:.0f} m", "small t2")
    svg.text(sx + sw / 2, y0 + ph + 34, "metres above sea level", "small t2", "middle")

    x = legend_swatches(svg, x0, h - 12, [("q2", "the airliner's volume", "box"),
                                          ("s2", f"{P['lateral_m']:.0f} m and ±{P['vertical_m']:.0f} m", "dash"),
                                          ("bad", "drone inside", "ring"),
                                          ("s1", "drone outside (dashed: its drone zone)", "ring")])
    svg.save(out / "conflict-protection.svg")


NAMES = {
    "climb": "Climb", "descend": "Descend", "level_off": "Level off", "turn_left": "Turn left",
    "turn_right": "Turn right", "speed_up": "Speed up", "slow_down": "Slow down",
}


def search(d, out):
    e = d["encounter"]
    w, h = 760, 330
    svg = Svg(
        w, h, "Round one of the advisory search",
        "Every manoeuvre of both drones, scored as risk left plus the manoeuvre's cost. "
        f"Carrying on scores {e['risk_before']:.1f}; an advisory must score under "
        f"{e['threshold']:.1f}. Climb and descend both leave no risk and score 1.1; climb is "
        "tried first and is advised to UAV 1. Speeding up makes things worse.",
    )
    groups = [[c for c in e["candidates"] if c["aircraft"] == a] for a in (1, 2)]
    xmax = max(c.get("score", 0) for c in e["candidates"]) * 1.05
    xmax = max(xmax, e["risk_before"] * 1.1)
    pw, top, row = 290, 58, 30
    for g, cands in enumerate(groups):
        x0 = 92 + g * (pw + 86)
        k = pw / xmax
        X = lambda v: x0 + v * k
        bot = top + row * len(cands)
        for t in nice_ticks(0, xmax, 4):
            svg.line(X(t), top - 4, X(t), bot, "grid")
            svg.text(X(t), bot + 15, fmt(t), "tm", "middle")
        colour = "s1" if g == 0 else "s2"
        svg.text(x0 - 84, 18, f"UAV {g + 1}", "h")
        for i, c in enumerate(cands):
            y = top + i * row
            svg.text(x0 - 8, y + row / 2 + 4, NAMES[c["manoeuvre"]["action"]], "small", "end")
            if not c["offered"]:
                svg.text(x0 + 4, y + row / 2 + 4, "not offered: already level", "small t2")
                continue
            bh = row - 12
            left, cost = c["risk_left"], c["cost"]
            if left > 0:
                svg.add(f'<rect x="{X(0)}" y="{y + 6}" width="{max(left * k - 1, 1):.1f}" height="{bh}" '
                        f'rx="2" fill="var(--{colour})"/>')
            cx = X(left) + (1 if left > 0 else 0)
            svg.add(f'<rect x="{cx:.1f}" y="{y + 6}" width="{max(cost * k - 1, 1.5):.1f}" height="{bh}" '
                    f'rx="2" fill="var(--cost)"/>')
            chosen = any(a["aircraft"] == c["aircraft"] and a["manoeuvre"] == c["manoeuvre"]
                         for a in e["advisories"])
            if chosen:
                svg.text(X(c["score"]) + 6, y + row / 2 + 4, f"{c['score']:.1f}  advised", "small h")
        svg.line(X(e["threshold"]), top - 6, X(e["threshold"]), bot, "bad", 1.5)
        svg.text(X(e["threshold"]), top - 26, f"advise only below {e['threshold']:.1f}", "small", "middle",
                 ' style="fill:var(--bad)"')
        svg.text(X(e["threshold"]), top - 11, f"(2 % under carrying on, {e['risk_before']:.1f})", "tm", "middle")
        svg.text(x0 + pw / 2, bot + 34, "score: risk left + cost", "small t2", "middle")
    legend_swatches(svg, 10, h - 12, [("s1", "risk left, UAV 1", "box"), ("s2", "risk left, UAV 2", "box"),
                                     ("cost", "manoeuvre's cost (share of carrying on)", "box")])
    svg.save(out / "conflict-search.svg")


def profiles(d, out):
    e = d["encounter"]
    w, h = 760, 360
    adv = e["advisories"][0]
    svg = Svg(
        w, h, "Height over the next minute, before and after the advisory",
        f"UAV 1 is advised to climb at {adv['targets']['vs_ms']:.1f} m/s. After its response delay "
        "it rises out of the shared layers, so by the first conflict at 40 s the two are layers "
        "apart. Bands are the 5th to 95th percentile of the particles' heights.",
    )
    x0, y0, pw, ph = 64, 24, 560, h - 84
    lo, hi = 0, 240
    X = lambda t: x0 + t / 60 * pw
    Y = lambda a: y0 + ph - (a - lo) / (hi - lo) * ph
    edges = [v for v in range(0, 241, 30)]
    for i, v in enumerate(edges):
        svg.line(x0, Y(v), x0 + pw, Y(v), "grid")
        svg.text(x0 - 6, Y(v) + 4, fmt(v), "tm", "end")
        if i < len(edges) - 1:
            svg.text(x0 + pw + 8, Y(v + 15) + 4, f"layer {i}", "tm")
    for t in range(0, 61, 10):
        svg.text(X(t), y0 + ph + 16, f"{t}", "tm", "middle")
    svg.add(f'<rect x="{x0}" y="{y0}" width="{pw}" height="{ph}" fill="none" stroke="var(--axis)"/>')
    first = adv["first_conflict_s"]
    svg.line(X(first), y0, X(first), y0 + ph, "bad", 1.5)
    svg.text(X(first) + 5, y0 + 14, f"first conflict, {first:.0f} s", "small", extra=' style="fill:var(--bad)"')

    def band(rows, colour, dashed=False, fill=True):
        if fill:
            up = " L".join(f"{X(r[0]):.1f},{Y(r[3]):.1f}" for r in rows)
            dn = " L".join(f"{X(r[0]):.1f},{Y(r[1]):.1f}" for r in reversed(rows))
            svg.add(f'<path d="M{up} L{dn}Z" fill="var(--{colour})" fill-opacity="0.18"/>')
        mid = " L".join(f"{X(r[0]):.1f},{Y(r[2]):.1f}" for r in rows)
        dash = ' stroke-dasharray="5 3"' if dashed else ""
        svg.add(f'<path d="M{mid}" fill="none" stroke="var(--{colour})" stroke-width="2"{dash}/>')

    p1, p2 = e["profiles"]
    band(p2["after"], "s2")
    band(p1["after"], "s1")
    end = p1["after"][-1]
    svg.text(X(2), Y(200), f"UAV 1, advised to climb at {adv['targets']['vs_ms']:.1f} m/s:", "small h halo")
    svg.text(X(2), Y(200) + 16, f"{end[2]:.0f} m by 60 s, after its response delay", "small t2 halo")
    svg.text(X(58), Y(60) - 10, "UAV 2 carries on at 60 m (as UAV 1 would have)", "small h halo", "end")
    svg.text(x0 + pw / 2, h - 30, "seconds ahead", "small t2", "middle")
    svg.text(18, y0 + ph / 2, "metres above sea level", "small t2", "middle",
             f' transform="rotate(-90 18 {y0 + ph / 2})"')
    x = legend_swatches(svg, x0, h - 8, [("s1", "UAV 1", "line"), ("s2", "UAV 2", "line")])
    svg.add(f'<rect x="{x}" y="{h - 18}" width="18" height="10" fill="var(--ink2)" fill-opacity="0.18"/>')
    svg.text(x + 24, h - 8, "5th to 95th percentile of the particles", "small t2")
    svg.text(x0 + pw, h - 8, f"risk {e['risk_before']:.1f} → {e['risk_after']:.1f}", "small h", "end")
    svg.save(out / "conflict-resolution.svg")


def energy(d, out):
    q, j = d["energy"]["quad"], d["energy"]["jet"]
    w, h = 760, 330
    svg = Svg(
        w, h, "What the energy models allow",
        "Left: the generic quadcopter's best climb falls from 3 m/s in the hover to half that "
        "at top speed, while its descent limit stays at 2.5 m/s. Right: the generic narrow-body "
        "jet's best climb at its speed schedule falls with altitude to nothing at its ceiling.",
    )

    def panel(x0, rows, xlab, title, xs=lambda v: v, marks=()):
        pw, top, ph = 300, 40, h - 110
        xmax = rows[-1][0]
        ymax = max(max(r[1], r[2]) for r in rows) * 1.1
        X = lambda v: x0 + v / xmax * pw
        Y = lambda v: top + ph - v / ymax * ph
        for t in nice_ticks(0, ymax, 4):
            svg.line(x0, Y(t), x0 + pw, Y(t), "grid")
            svg.text(x0 - 6, Y(t) + 4, f"{t:g}", "tm", "end")
        for t in nice_ticks(0, xmax, 4):
            svg.text(X(t), top + ph + 16, fmt(xs(t)), "tm", "middle")
        svg.line(x0, top + ph, x0 + pw, top + ph, "axis")
        for i, (v, lab) in enumerate(marks):
            svg.line(X(v), top, X(v), top + ph, "axis", 1)
            svg.text(X(v) - 4, top + 12 + 15 * i, lab, "tm", "end")
        for col, colour, label in ((1, "s1", "best climb"), (2, "s2", "most descent")):
            pts = " L".join(f"{X(r[0]):.1f},{Y(r[col]):.1f}" for r in rows)
            svg.add(f'<path d="M{pts}" fill="none" stroke="var(--{colour})" stroke-width="2"/>')
            last = rows[-1]
            svg.text(X(last[0]) - 2, Y(last[col]) + (-8 if col == 1 else 16), label, "small h", "end")
        svg.text(x0, 24, title, "h")
        svg.text(x0 + pw / 2, top + ph + 36, xlab, "small t2", "middle")
        svg.text(x0 - 40, top + ph / 2, "m/s", "small t2", "middle",
                 f' transform="rotate(-90 {x0 - 40} {top + ph / 2})"')

    panel(70, q["rows"], f"forward airspeed, m/s (at {q['alt_m']:.0f} m)", "Multirotor: generic quadcopter")
    panel(440, j["rows"], "altitude, ft (at its speed schedule)", "Fixed-wing: generic narrow-body",
          marks=((j["crossover_ft"], f"CAS to Mach, {fmt(round(j['crossover_ft'], -2))} ft"),
                 (j["tropopause_ft"], "tropopause")))
    legend_swatches(svg, 70, h - 10, [("s1", "best climb", "line"), ("s2", "most descent", "line")])
    svg.save(out / "conflict-energy.svg")


def main():
    data = json.loads(Path(sys.argv[1]).read_text())
    out = Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    figs = (pipeline, cloud, cells, layers, detection, protection, search, profiles, energy)
    for fig in figs:
        fig(data, out)
    print(f"wrote {len(figs)} figures to {out}")


if __name__ == "__main__":
    main()
