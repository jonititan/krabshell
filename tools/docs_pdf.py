#!/usr/bin/env python3
"""Export the guides as one PDF, with a contents index and bookmarks.

    uv run --with markdown --with playwright --with pypdf tools/docs_pdf.py [out.pdf]

The guides are the ones the dashboard's Docs tab serves, in its order and
groups (crates/sim-axum/src/docs.rs). Links between them become links inside
the PDF, links to source files go to GitHub, and the figures are embedded.

It needs Google Chrome (Playwright drives the installed one). It prints
twice: the first pass finds the page every heading lands on from the PDF's
bookmarks, the second prints those page numbers in the contents.
"""

import base64
import datetime
import html
import posixpath
import re
import subprocess
import sys
from pathlib import Path

import markdown
from playwright.sync_api import sync_playwright
from pypdf import PdfReader

ROOT = Path(__file__).resolve().parent.parent
REPO_BLOB = "https://github.com/jonititan/krabshell/blob/master/"

# (slug, group, path): the dashboard's order, read from its source so the two
# never drift apart.
GUIDE = re.compile(r'guide!\("([^"]+)",\s*"([^"]+)",\s*"([^"]+)"\)')


def guides():
    src = (ROOT / "crates/sim-axum/src/docs.rs").read_text()
    return GUIDE.findall(src)


def slugify(text):
    """GitHub's heading anchors, as the dashboard makes them."""
    text = re.sub(r"<[^>]+>", "", text)
    text = html.unescape(text).strip().lower()
    return "".join("-" if c == " " else c for c in text if c.isalnum() or c in " -_")


def render(slug, path, by_path):
    """One guide as HTML, its anchors prefixed with its slug so they are
    unique in the whole document, and its links pointed inside it."""
    md = markdown.Markdown(
        extensions=["tables", "fenced_code", "toc", "sane_lists"],
        extension_configs={"toc": {"slugify": lambda v, sep: slugify(v)}},
    )
    body = md.convert((ROOT / path).read_text())
    here = posixpath.dirname(path)

    def link(m):
        href = html.unescape(m.group(1))
        if href.startswith(("http://", "https://", "mailto:")):
            return m.group(0)
        if href.startswith("#"):
            return f'href="#{slug}--{href[1:]}"'
        target, _, frag = href.partition("#")
        full = posixpath.normpath(posixpath.join(here, target))
        if full in by_path:
            other = by_path[full]
            return f'href="#{other}--{frag}"' if frag else f'href="#{other}"'
        return f'href="{REPO_BLOB}{full}{"#" + frag if frag else ""}"'

    body = re.sub(r'href="([^"]*)"', link, body)
    body = re.sub(r'id="([^"]*)"', lambda m: f'id="{slug}--{m.group(1)}"', body)

    def image(m):
        src = posixpath.normpath(posixpath.join(here, html.unescape(m.group(1))))
        data = base64.b64encode((ROOT / src).read_bytes()).decode()
        return f'src="data:image/svg+xml;base64,{data}"'

    body = re.sub(r'src="([^"]*\.svg)"', image, body)
    # The guide's own title is the chapter heading.
    heads = [(int(lv), hid, re.sub(r"<[^>]+>", "", t))
             for lv, hid, t in re.findall(r'<h([12]) id="([^"]+)">(.*?)</h[12]>', body)]
    return body, heads


CSS = """
@page { size: A4; margin: 18mm 16mm 20mm 16mm; }
body { font: 10.5pt/1.45 system-ui, -apple-system, "Segoe UI", sans-serif; color: #111; }
h1 { font-size: 20pt; margin: 0 0 10pt; }
h2 { font-size: 14pt; margin: 18pt 0 6pt; border-bottom: 1px solid #ddd; padding-bottom: 2pt; }
h3 { font-size: 12pt; margin: 14pt 0 4pt; }
h4 { font-size: 10.5pt; margin: 12pt 0 4pt; }
h2, h3, h4 { break-after: avoid; }
p, li { orphans: 3; widows: 3; }
a { color: #1c5cab; text-decoration: none; }
code { font: 9pt ui-monospace, "SFMono-Regular", Menlo, monospace; background: #f3f3f1; padding: 0 2pt; border-radius: 2pt; }
pre { background: #f6f7f9; border: 1px solid #e1e0d9; border-radius: 4pt; padding: 6pt 8pt; overflow-wrap: anywhere; white-space: pre-wrap; break-inside: avoid; }
pre code { background: none; padding: 0; font-size: 8.5pt; }
table { border-collapse: collapse; width: 100%; margin: 6pt 0 10pt; font-size: 9pt; break-inside: auto; }
th, td { border: 1px solid #ddd; padding: 3pt 5pt; vertical-align: top; text-align: left; overflow-wrap: break-word; }
th { background: #f6f7f9; }
tr { break-inside: avoid; }
img { max-width: 100%; display: block; margin: 8pt auto; break-inside: avoid; }
blockquote { border-left: 3px solid #ddd; margin: 0; padding-left: 10pt; color: #444; }
.guide { break-before: page; }
.cover { height: 240mm; display: flex; flex-direction: column; justify-content: center; }
.cover h1 { font-size: 30pt; border: none; }
.cover p { color: #555; font-size: 12pt; margin: 2pt 0; }
.contents { break-before: page; }
.contents h1 { margin-bottom: 14pt; }
.contents .group { font-weight: 700; margin: 12pt 0 4pt; text-transform: uppercase; font-size: 9pt; letter-spacing: .06em; color: #555; }
.contents .entry { display: flex; align-items: baseline; gap: 4pt; margin: 1.5pt 0; }
.contents .entry.l1 { font-weight: 600; margin-top: 6pt; }
.contents .entry.l2 { padding-left: 14pt; font-size: 9.5pt; }
.contents .entry .dots { flex: 1; border-bottom: 1px dotted #aaa; transform: translateY(-3pt); }
.contents .entry .page { font-variant-numeric: tabular-nums; min-width: 18pt; text-align: right; }
"""


def document(chapters, pages):
    """The whole book. `pages` maps an anchor to its page, or is empty on
    the first pass (the numbers then hold their place)."""
    rev = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"],
                         capture_output=True, text=True).stdout.strip() or "unknown"
    today = datetime.date.today().isoformat()
    out = [f"<!doctype html><html><head><meta charset='utf-8'><title>krabshell documentation</title>"
           f"<style>{CSS}</style></head><body>"]
    out.append("<section class='cover'><h1>krabshell</h1><p>Documentation</p>"
               f"<p>{today} · commit {html.escape(rev)}</p></section>")
    out.append("<section class='contents'><h1 id='contents'>Contents</h1>")
    group = None
    for ch in chapters:
        if ch["group"] != group:
            group = ch["group"]
            out.append(f"<div class='group'>{html.escape(group)}</div>")
        for level, hid, title in ch["heads"]:
            anchor = ch["slug"] if level == 1 else f"{ch['slug']}--{hid}"
            page = pages.get(anchor, "000")
            out.append(f"<div class='entry l{level}'><a href='#{anchor}'>{title}</a>"
                       f"<span class='dots'></span><span class='page'>{page}</span></div>")
    out.append("</section>")
    for ch in chapters:
        out.append(f"<section class='guide' id='{ch['slug']}'>{ch['body']}</section>")
    out.append("</body></html>")
    return "".join(out)


def outline_pages(pdf_path):
    """Every bookmark in document order, as (title, page number)."""
    reader = PdfReader(pdf_path)
    flat = []

    def walk(items):
        for it in items:
            if isinstance(it, list):
                walk(it)
            else:
                flat.append((it.title.strip(), reader.get_destination_page_number(it) + 1))

    walk(reader.outline)
    return flat, len(reader.pages)


def print_pdf(page, html_doc, out):
    page.set_content(html_doc, wait_until="load")
    page.pdf(
        path=str(out),
        format="A4",
        print_background=True,
        outline=True,
        tagged=True,
        display_header_footer=True,
        header_template="<span></span>",
        footer_template=(
            "<div style='font:8pt system-ui,sans-serif;color:#888;width:100%;"
            "padding:0 16mm;display:flex;justify-content:space-between'>"
            "<span>krabshell documentation</span><span class='pageNumber'></span></div>"
        ),
        margin={"top": "18mm", "bottom": "20mm", "left": "16mm", "right": "16mm"},
    )


def main():
    out = Path(sys.argv[1] if len(sys.argv) > 1 else ROOT / "target/krabshell-docs.pdf")
    out.parent.mkdir(parents=True, exist_ok=True)
    listed = guides()
    by_path = {path: slug for slug, _, path in listed}
    chapters = []
    for slug, group, path in listed:
        body, heads = render(slug, path, by_path)
        # The chapter's title links to the chapter, the rest to their heading.
        chapters.append({"slug": slug, "group": group, "body": body, "heads": heads})

    with sync_playwright() as p:
        browser = p.chromium.launch(channel="chrome", headless=True)
        page = browser.new_page()
        page.emulate_media(media="print", color_scheme="light")
        # Pass 1: where does every heading land?
        print_pdf(page, document(chapters, {}), out)
        marks, _ = outline_pages(out)
        # Chrome drops the space where a long heading wraps, so titles are
        # compared without their spaces.
        squash = lambda t: re.sub(r"\s+", "", t)
        marks = [(squash(t), n) for t, n in marks]
        # The guides start after the cover and the contents, whose own
        # headings would otherwise answer for a guide of the same name.
        start = next((j + 1 for j, (t, _) in enumerate(marks) if t == "Contents"), 0)
        pages, i = {}, start
        for ch in chapters:
            for level, hid, title in ch["heads"]:
                anchor = ch["slug"] if level == 1 else f"{ch['slug']}--{hid}"
                plain = squash(html.unescape(title))
                # Bookmarks come in document order: this heading's is the
                # next one with its title. One not found leaves the place
                # where it is, so it cannot cost every entry after it.
                found = next((j for j in range(i, len(marks)) if marks[j][0] == plain), None)
                if found is not None:
                    pages[anchor] = marks[found][1]
                    i = found + 1
        # Pass 2: the same book with the page numbers in.
        print_pdf(page, document(chapters, pages), out)
        browser.close()

    _, count = outline_pages(out)
    missing = sum(len(ch["heads"]) for ch in chapters) - len(pages)
    print(f"wrote {out}: {count} pages, {len(chapters)} guides"
          + (f", {missing} contents entries without a page" if missing else ""))


if __name__ == "__main__":
    main()
