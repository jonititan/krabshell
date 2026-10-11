//! The guides served at `/docs`.
//!
//! They are the Markdown files under `docs/` (and the spec at the repository
//! root), compiled into the binary like the dashboard's assets, so the docs a
//! server shows are the docs for that build. Each is rendered once, on first
//! use. Links between them are written as relative `.md` paths, so they also
//! work on GitHub; here they become `/docs/{slug}`, and links to source files
//! go to the repository.

use std::collections::HashMap;
use std::sync::LazyLock;

use pulldown_cmark::{CowStr, Event, HeadingLevel, Options, Parser, Tag, TagEnd};

/// Where links to source files point.
const REPO_BLOB: &str = "https://github.com/jonititan/krabshell/blob/master/";

/// One guide: its URL slug, the sidebar group it sits in, its path from the
/// repository root (relative links in it resolve from there), and its text.
struct Source {
    slug: &'static str,
    group: &'static str,
    path: &'static str,
    markdown: &'static str,
}

macro_rules! guide {
    ($slug:literal, $group:literal, $path:literal) => {
        Source {
            slug: $slug,
            group: $group,
            path: $path,
            markdown: include_str!(concat!("../../../", $path)),
        }
    };
}

macro_rules! figure {
    ($name:literal) => {
        (
            $name,
            include_str!(concat!("../../../docs/figures/", $name)),
        )
    };
}

/// The guides, in sidebar order. The first is the `/docs` landing page.
const SOURCES: &[Source] = &[
    guide!("overview", "Start here", "docs/overview.md"),
    guide!("operating", "Start here", "docs/operating.md"),
    guide!("dashboard", "Start here", "docs/dashboard.md"),
    guide!("map", "Start here", "docs/map.md"),
    guide!("scenarios", "Simulation", "docs/scenarios.md"),
    guide!("drones", "Simulation", "docs/drones.md"),
    guide!("drone-behaviours", "Simulation", "docs/drone-behaviours.md"),
    guide!("geofences", "Simulation", "docs/geofences.md"),
    guide!("stations", "Simulation", "docs/stations.md"),
    guide!("weather", "Simulation", "docs/weather.md"),
    guide!("conflicts", "Simulation", "docs/conflicts.md"),
    guide!("connecting", "Integration", "docs/connecting.md"),
    guide!("protocols", "Integration", "docs/protocols.md"),
    guide!("api-guide", "Integration", "docs/api-guide.md"),
    guide!("architecture", "Internals", "docs/architecture.md"),
    guide!("development", "Internals", "docs/development.md"),
    guide!("baseline", "Internals", "docs/baseline.md"),
    guide!("spec", "Internals", "krabshell_spec.md"),
];

/// Images the guides show, served at `/docs/figures/{name}`. SVG only, so
/// they follow the reader's light or dark preference.
pub const FIGURES: &[(&str, &str)] = &[
    figure!("conflict-pipeline.svg"),
    figure!("conflict-cloud.svg"),
    figure!("conflict-cells.svg"),
    figure!("conflict-layers.svg"),
    figure!("conflict-detection.svg"),
    figure!("conflict-protection.svg"),
    figure!("conflict-search.svg"),
    figure!("conflict-resolution.svg"),
    figure!("conflict-energy.svg"),
];

pub fn figure(name: &str) -> Option<&'static str> {
    FIGURES
        .iter()
        .find(|(n, _)| *n == name)
        .map(|(_, svg)| *svg)
}

/// A heading on a page, for its contents list.
pub struct Heading {
    pub level: u8,
    pub id: String,
    pub text: String,
}

/// A guide, rendered.
pub struct Page {
    pub slug: &'static str,
    pub group: &'static str,
    pub title: String,
    pub html: String,
    /// The page's `##` and `###` headings.
    pub toc: Vec<Heading>,
}

/// The sidebar: groups in order, each with its pages' slugs and titles.
pub struct Group {
    pub name: &'static str,
    pub pages: Vec<(&'static str, String)>,
}

pub struct Docs {
    pub pages: Vec<Page>,
    pub groups: Vec<Group>,
}

impl Docs {
    pub fn page(&self, slug: &str) -> Option<&Page> {
        self.pages.iter().find(|p| p.slug == slug)
    }
}

pub static DOCS: LazyLock<Docs> = LazyLock::new(build);

fn build() -> Docs {
    // Every guide by its path, so a link to `docs/x.md` or `../README.md`
    // can be told apart from a link to source.
    let by_path: HashMap<&str, &str> = SOURCES.iter().map(|s| (s.path, s.slug)).collect();
    let pages: Vec<Page> = SOURCES.iter().map(|s| render(s, &by_path)).collect();
    let mut groups: Vec<Group> = Vec::new();
    for p in &pages {
        match groups.last_mut() {
            Some(g) if g.name == p.group => g.pages.push((p.slug, p.title.clone())),
            _ => groups.push(Group {
                name: p.group,
                pages: vec![(p.slug, p.title.clone())],
            }),
        }
    }
    Docs { pages, groups }
}

fn render(source: &Source, by_path: &HashMap<&str, &str>) -> Page {
    let options = Options::ENABLE_TABLES
        | Options::ENABLE_STRIKETHROUGH
        | Options::ENABLE_HEADING_ATTRIBUTES
        | Options::ENABLE_FOOTNOTES;
    let mut events: Vec<Event> = Parser::new_ext(source.markdown, options).collect();

    let mut title = String::new();
    let mut toc = Vec::new();
    let mut seen: HashMap<String, usize> = HashMap::new();
    let mut i = 0;
    while i < events.len() {
        match &events[i] {
            Event::Start(Tag::Heading { level, id, .. }) => {
                let level = *level;
                let given = id.as_ref().map(|s| s.to_string());
                let mut text = String::new();
                let mut j = i + 1;
                while !matches!(events.get(j), Some(Event::End(TagEnd::Heading(_))) | None) {
                    if let Event::Text(t) | Event::Code(t) = &events[j] {
                        text.push_str(t);
                    }
                    j += 1;
                }
                let id = given.unwrap_or_else(|| unique(slugify(&text), &mut seen));
                if let Event::Start(Tag::Heading { id: slot, .. }) = &mut events[i] {
                    *slot = Some(CowStr::from(id.clone()));
                }
                match level {
                    HeadingLevel::H1 if title.is_empty() => title = text,
                    HeadingLevel::H2 | HeadingLevel::H3 => toc.push(Heading {
                        level: level as u8,
                        id,
                        text,
                    }),
                    _ => {}
                }
                i = j;
            }
            Event::Start(Tag::Image { dest_url, .. }) => {
                let url = image(source.path, dest_url);
                if let Some(url) = url {
                    if let Event::Start(Tag::Image { dest_url, .. }) = &mut events[i] {
                        *dest_url = CowStr::from(url);
                    }
                }
                i += 1;
            }
            Event::Start(Tag::Link { dest_url, .. }) => {
                let url = rewrite(source.path, dest_url, by_path);
                if let Some(url) = url {
                    if let Event::Start(Tag::Link { dest_url, .. }) = &mut events[i] {
                        *dest_url = CowStr::from(url);
                    }
                }
                i += 1;
            }
            _ => i += 1,
        }
    }

    let mut html = String::with_capacity(source.markdown.len() * 3 / 2);
    pulldown_cmark::html::push_html(&mut html, events.into_iter());
    if title.is_empty() {
        title = source.slug.to_owned();
    }
    Page {
        slug: source.slug,
        group: source.group,
        title,
        html,
        toc,
    }
}

/// A heading's anchor, as GitHub makes it: lower case, spaces to hyphens,
/// and punctuation other than `-` and `_` dropped. Links written for GitHub
/// then land in the same place here.
pub fn slugify(text: &str) -> String {
    text.trim()
        .chars()
        .flat_map(char::to_lowercase)
        .filter_map(|c| match c {
            ' ' => Some('-'),
            c if c.is_alphanumeric() || c == '-' || c == '_' => Some(c),
            _ => None,
        })
        .collect()
}

fn unique(id: String, seen: &mut HashMap<String, usize>) -> String {
    let n = seen.entry(id.clone()).or_insert(0);
    let out = if *n == 0 { id } else { format!("{id}-{n}") };
    *n += 1;
    out
}

/// Where a relative link in the page at `from` should go: another guide's
/// `/docs/{slug}`, or the file in the repository. `None` leaves it alone
/// (absolute URLs, site paths, in-page anchors).
fn rewrite(from: &str, dest: &str, by_path: &HashMap<&str, &str>) -> Option<String> {
    if dest.is_empty()
        || dest.starts_with('#')
        || dest.starts_with('/')
        || dest.contains("://")
        || dest.starts_with("mailto:")
    {
        return None;
    }
    let (path, fragment) = match dest.split_once('#') {
        Some((p, f)) => (p, Some(f)),
        None => (dest, None),
    };
    let resolved = resolve(from, path);
    let hash = fragment.map(|f| format!("#{f}")).unwrap_or_default();
    Some(match by_path.get(resolved.as_str()) {
        Some(slug) => format!("/docs/{slug}{hash}"),
        None => format!("{REPO_BLOB}{resolved}{hash}"),
    })
}

/// Where a relative image in the page at `from` is served: one of
/// [`FIGURES`]. `None` leaves anything else alone.
fn image(from: &str, dest: &str) -> Option<String> {
    if dest.contains("://") || dest.starts_with('/') {
        return None;
    }
    let resolved = resolve(from, dest);
    let name = resolved.strip_prefix("docs/figures/")?;
    figure(name).map(|_| format!("/docs/figures/{name}"))
}

/// `path` relative to the directory of the file `from`, both from the
/// repository root, with `.` and `..` worked out.
fn resolve(from: &str, path: &str) -> String {
    let mut parts: Vec<&str> = from.split('/').collect();
    parts.pop();
    for part in path.split('/') {
        match part {
            "" | "." => {}
            ".." => {
                parts.pop();
            }
            p => parts.push(p),
        }
    }
    parts.join("/")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn anchors_match_github() {
        assert_eq!(slugify("Drones: MAVLink and CoT"), "drones-mavlink-and-cot");
        assert_eq!(
            slugify("WSL2 with NAT (the WSL default)"),
            "wsl2-with-nat-the-wsl-default"
        );
        assert_eq!(slugify("`/run/step`"), "runstep");
    }

    #[test]
    fn links_resolve_from_the_page() {
        let by_path: HashMap<&str, &str> = [("docs/a.md", "a"), ("krabshell_spec.md", "spec")]
            .into_iter()
            .collect();
        let go = |from, to| rewrite(from, to, &by_path);
        assert_eq!(go("docs/b.md", "a.md#x").as_deref(), Some("/docs/a#x"));
        assert_eq!(
            go("docs/b.md", "../krabshell_spec.md").as_deref(),
            Some("/docs/spec")
        );
        assert_eq!(
            go("docs/b.md", "../crates/sim-axum/src/lib.rs").as_deref(),
            Some("https://github.com/jonititan/krabshell/blob/master/crates/sim-axum/src/lib.rs")
        );
        assert_eq!(go("docs/b.md", "/docs/api"), None);
        assert_eq!(go("docs/b.md", "#here"), None);
    }

    /// Every guide renders with a title, and every link from one guide to
    /// another lands on a page and heading that exist.
    #[test]
    fn every_link_between_guides_lands() {
        let docs = &*DOCS;
        let mut broken = Vec::new();
        for page in &docs.pages {
            assert!(!page.title.is_empty(), "{} has no title", page.slug);
            for (at, _) in page.html.match_indices("href=\"") {
                let rest = &page.html[at + 6..];
                let href = &rest[..rest.find('"').unwrap()];
                let (target, anchor) = if let Some(local) = href.strip_prefix('#') {
                    (page.slug, Some(local))
                } else if let Some(path) = href.strip_prefix("/docs/") {
                    match path.split_once('#') {
                        Some((s, a)) => (s, Some(a)),
                        None => (path, None),
                    }
                } else {
                    continue;
                };
                if target == "api" || target.starts_with("figures/") {
                    continue;
                }
                let Some(dest) = docs.page(target) else {
                    broken.push(format!("{}: {href} (no such page)", page.slug));
                    continue;
                };
                if let Some(anchor) = anchor {
                    let found = dest.html.contains(&format!("id=\"{anchor}\""));
                    if !found {
                        broken.push(format!("{}: {href} (no such heading)", page.slug));
                    }
                }
            }
        }
        assert!(broken.is_empty(), "broken links:\n{}", broken.join("\n"));
    }

    /// Every image a guide shows is one of the figures served, and every
    /// figure is shown somewhere.
    #[test]
    fn every_image_is_served_and_every_figure_shown() {
        let docs = &*DOCS;
        let mut shown = std::collections::BTreeSet::new();
        for page in &docs.pages {
            for (at, _) in page.html.match_indices("<img src=\"") {
                let rest = &page.html[at + 10..];
                let src = &rest[..rest.find('"').unwrap()];
                let name = src.strip_prefix("/docs/figures/");
                assert!(
                    name.is_some_and(|n| figure(n).is_some()),
                    "{}: image {src} is not served",
                    page.slug
                );
                shown.insert(name.unwrap().to_owned());
            }
        }
        for (name, svg) in FIGURES {
            assert!(shown.contains(*name), "{name} is shown on no page");
            assert!(svg.starts_with("<svg"), "{name} is not an SVG");
        }
    }
}
