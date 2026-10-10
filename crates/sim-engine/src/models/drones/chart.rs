//! A standalone SVG bar chart, for the run log's histograms.

/// Bars a chart draws in.
pub const BAR: &str = "#2b6cb0";
/// A bar that stands apart from the rest, such as crashes among landings.
pub const ALERT: &str = "#c53030";

/// One bar, left to right.
pub struct Bar {
    pub count: usize,
    pub fill: &'static str,
    /// Shown on hover.
    pub tooltip: String,
}

/// What a chart says and shows.
pub struct Chart<'a> {
    pub title: &'a str,
    /// Grey lines under the title.
    pub notes: Vec<String>,
    pub bars: Vec<Bar>,
    /// Labels under the x axis, at positions counted in bar widths from the
    /// left edge.
    pub ticks: Vec<(f64, String)>,
    pub x_label: &'a str,
    pub y_label: &'a str,
}

impl Chart<'_> {
    pub fn svg(&self) -> String {
        let (w, h) = (760.0, 420.0);
        let (left, right, top, bottom) = (64.0, 24.0, 92.0, 56.0);
        let plot_w = w - left - right;
        let plot_h = h - top - bottom;
        let peak = self.bars.iter().map(|b| b.count).max().unwrap_or(0).max(1);
        let step = nice_step(peak as f64 / 5.0);
        let y_max = (peak as f64 / step).ceil() * step;
        let mut svg = format!(
            r##"<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" width="{w}" height="{h}" font-family="system-ui, sans-serif" font-size="12">
<rect width="100%" height="100%" fill="#ffffff"/>
<text x="{left}" y="28" font-size="16" font-weight="600" fill="#16181d">{}</text>
"##,
            xml(self.title)
        );
        for (i, note) in self.notes.iter().enumerate() {
            svg.push_str(&format!(
                r##"<text x="{left}" y="{}" fill="#666e7a">{}</text>
"##,
                50 + 18 * i,
                xml(note)
            ));
        }
        // Gridlines and y labels.
        let mut y = 0.0;
        while y <= y_max + 1e-9 {
            let py = top + plot_h - y / y_max * plot_h;
            svg.push_str(&format!(
                r##"<line x1="{left}" x2="{}" y1="{py:.1}" y2="{py:.1}" stroke="#dfe3e9"/>
<text x="{}" y="{:.1}" text-anchor="end" fill="#666e7a">{}</text>
"##,
                left + plot_w,
                left - 8.0,
                py + 4.0,
                y as u64
            ));
            y += step;
        }
        if !self.bars.is_empty() {
            let bar = plot_w / self.bars.len() as f64;
            for (i, b) in self.bars.iter().enumerate() {
                let bh = b.count as f64 / y_max * plot_h;
                let x = left + i as f64 * bar;
                svg.push_str(&format!(
                    r##"<rect x="{:.1}" y="{:.1}" width="{:.1}" height="{bh:.1}" fill="{}"><title>{}</title></rect>
"##,
                    x + 1.0,
                    top + plot_h - bh,
                    (bar - 2.0).max(1.0),
                    b.fill,
                    xml(&b.tooltip)
                ));
            }
            for (at, label) in &self.ticks {
                svg.push_str(&format!(
                    r##"<text x="{:.1}" y="{:.1}" text-anchor="middle" fill="#666e7a">{}</text>
"##,
                    left + at * bar,
                    top + plot_h + 18.0,
                    xml(label)
                ));
            }
        }
        svg.push_str(&format!(
            r##"<line x1="{left}" x2="{}" y1="{y0:.1}" y2="{y0:.1}" stroke="#16181d"/>
<text x="{:.1}" y="{:.1}" text-anchor="middle" fill="#16181d">{}</text>
<text transform="translate(18 {:.1}) rotate(-90)" text-anchor="middle" fill="#16181d">{}</text>
</svg>
"##,
            left + plot_w,
            left + plot_w / 2.0,
            h - 16.0,
            xml(self.x_label),
            top + plot_h / 2.0,
            xml(self.y_label),
            y0 = top + plot_h
        ));
        svg
    }
}

fn nice_step(raw: f64) -> f64 {
    let raw = raw.max(1.0);
    let mag = 10f64.powf(raw.log10().floor());
    [1.0, 2.0, 5.0, 10.0]
        .iter()
        .map(|m| m * mag)
        .find(|s| *s >= raw)
        .unwrap_or(10.0 * mag)
}

pub fn csv_field(s: &str) -> String {
    if s.contains([',', '"', '\n']) {
        format!("\"{}\"", s.replace('"', "\"\""))
    } else {
        s.to_owned()
    }
}

fn xml(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
}
