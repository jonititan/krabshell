//! The OpenAPI description of the REST surface, and the page that renders it.
//!
//! The document is written by hand in `openapi.json` beside this crate's
//! manifest and compiled into the binary. Hand-written because the bodies are
//! built from types across four crates and several are the model's own JSON;
//! deriving it would put a schema dependency on all of them. The route tests
//! keep it honest: every route must be described, every description must be
//! routed, and live responses must match their schemas.
//!
//! The docs page is rendered on the server from the same document, so it
//! works offline and without JavaScript like the rest of the dashboard. Any
//! OpenAPI tool (Swagger UI, Redoc, a client generator) can read
//! `/api/v1/openapi.json` instead.

use std::sync::LazyLock;

use axum::http::header;
use axum::response::{IntoResponse, Response};
use serde_json::{Map, Value};

use crate::API_PREFIX;

/// The document, as served.
pub const SPEC: &str = include_str!("../openapi.json");

/// The document, parsed once.
pub static DOC: LazyLock<Value> =
    LazyLock::new(|| serde_json::from_str(SPEC).expect("openapi.json is valid JSON"));

/// `GET /api/v1/openapi.json`.
pub async fn spec() -> Response {
    (
        [
            (header::CONTENT_TYPE, "application/json"),
            (header::CACHE_CONTROL, "public, max-age=60"),
        ],
        SPEC,
    )
        .into_response()
}

/// The HTTP methods an OpenAPI path item may hold, in the order the page
/// lists them.
pub const METHODS: [&str; 5] = ["get", "post", "put", "patch", "delete"];

/// Follow a local `$ref` (`#/components/...`) to what it names.
pub fn resolve<'a>(doc: &'a Value, schema: &'a Value) -> &'a Value {
    match schema.get("$ref").and_then(Value::as_str) {
        Some(reference) => reference
            .strip_prefix('#')
            .and_then(|pointer| doc.pointer(pointer))
            .unwrap_or(&Value::Null),
        None => schema,
    }
}

// ------------------------------------------------------------ view models

pub struct DocsView {
    pub title: String,
    pub version: String,
    pub intro: String,
    pub tags: Vec<TagView>,
    pub schemas: Vec<SchemaView>,
}

pub struct TagView {
    pub name: String,
    pub description: String,
    pub ops: Vec<OpView>,
}

pub struct OpView {
    pub anchor: String,
    pub method: String,
    pub path: String,
    pub summary: String,
    pub description: String,
    /// No token needed.
    pub public: bool,
    pub params: Vec<FieldView>,
    pub body: Option<BodyView>,
    pub responses: Vec<ResponseView>,
}

pub struct BodyView {
    pub content_type: String,
    pub required: bool,
    pub ty: String,
    pub fields: Vec<FieldView>,
}

pub struct ResponseView {
    pub status: String,
    pub description: String,
    pub content_type: String,
    pub ty: String,
    pub headers: Vec<FieldView>,
}

pub struct SchemaView {
    pub name: String,
    pub description: String,
    /// The type, for a schema that is not an object with fields.
    pub ty: String,
    pub fields: Vec<FieldView>,
    pub variants: Vec<VariantView>,
}

pub struct VariantView {
    pub title: String,
    pub description: String,
    pub ty: String,
    pub fields: Vec<FieldView>,
}

/// A parameter, header or property. `ty` and `description` are HTML.
pub struct FieldView {
    pub name: String,
    /// Where a parameter goes (`path`, `query`); empty for a property.
    pub location: String,
    pub required: bool,
    pub ty: String,
    pub description: String,
}

/// The whole page's content. Built once: the document cannot change while the
/// binary runs.
pub static VIEW: LazyLock<DocsView> = LazyLock::new(|| build(&DOC));

fn build(doc: &Value) -> DocsView {
    fn text<'a>(v: &'a Value, key: &str) -> &'a str {
        v.get(key).and_then(Value::as_str).unwrap_or_default()
    }
    let info = &doc["info"];

    let mut tags: Vec<TagView> = doc["tags"]
        .as_array()
        .into_iter()
        .flatten()
        .map(|tag| TagView {
            name: text(tag, "name").to_owned(),
            description: prose(text(tag, "description")),
            ops: Vec::new(),
        })
        .collect();

    for (path, item) in doc["paths"].as_object().into_iter().flatten() {
        for method in METHODS {
            let Some(op) = item.get(method) else { continue };
            let view = operation(doc, path, method, op);
            let tag = op["tags"][0].as_str().unwrap_or("other");
            match tags.iter_mut().find(|t| t.name == tag) {
                Some(t) => t.ops.push(view),
                None => tags.push(TagView {
                    name: tag.to_owned(),
                    description: String::new(),
                    ops: vec![view],
                }),
            }
        }
    }
    tags.retain(|t| !t.ops.is_empty());

    let schemas = doc["components"]["schemas"]
        .as_object()
        .into_iter()
        .flatten()
        .map(|(name, schema)| schema_view(doc, name, schema))
        .collect();

    DocsView {
        title: text(info, "title").to_owned(),
        version: text(info, "version").to_owned(),
        intro: prose(text(info, "description")),
        tags,
        schemas,
    }
}

fn operation(doc: &Value, path: &str, method: &str, op: &Value) -> OpView {
    let text = |key: &str| op.get(key).and_then(Value::as_str).unwrap_or_default();

    let params = op["parameters"]
        .as_array()
        .into_iter()
        .flatten()
        .map(|p| {
            let p = resolve(doc, p);
            FieldView {
                name: p["name"].as_str().unwrap_or_default().to_owned(),
                location: p["in"].as_str().unwrap_or_default().to_owned(),
                required: p["required"].as_bool().unwrap_or(false),
                ty: type_html(&p["schema"]),
                description: prose(p["description"].as_str().unwrap_or_default()),
            }
        })
        .collect();

    let body = op.get("requestBody").map(|body| {
        let body = resolve(doc, body);
        let (content_type, media) = first_content(body);
        let schema = &media["schema"];
        BodyView {
            content_type,
            required: body["required"].as_bool().unwrap_or(false),
            ty: type_html(schema),
            // Spell out an inline object; a named one links to its schema.
            fields: if schema.get("$ref").is_none() {
                fields(doc, schema)
            } else {
                Vec::new()
            },
        }
    });

    let responses = op["responses"]
        .as_object()
        .into_iter()
        .flatten()
        .map(|(status, response)| {
            let response = resolve(doc, response);
            let (content_type, media) = first_content(response);
            ResponseView {
                status: status.clone(),
                description: prose(response["description"].as_str().unwrap_or_default()),
                ty: media.get("schema").map(type_html).unwrap_or_default(),
                content_type,
                headers: response["headers"]
                    .as_object()
                    .into_iter()
                    .flatten()
                    .map(|(name, h)| {
                        let h = resolve(doc, h);
                        FieldView {
                            name: name.clone(),
                            location: "header".into(),
                            required: false,
                            ty: type_html(&h["schema"]),
                            description: prose(h["description"].as_str().unwrap_or_default()),
                        }
                    })
                    .collect(),
            }
        })
        .collect();

    OpView {
        anchor: text("operationId").to_owned(),
        method: method.to_owned(),
        path: format!("{API_PREFIX}{path}"),
        summary: text("summary").to_owned(),
        description: prose(text("description")),
        public: op["security"].as_array().is_some_and(Vec::is_empty),
        params,
        body,
        responses,
    }
}

/// The first media type of a request body or response, and its object.
fn first_content(v: &Value) -> (String, &Value) {
    v["content"]
        .as_object()
        .and_then(|c| c.iter().next())
        .map_or((String::new(), &Value::Null), |(t, m)| (t.clone(), m))
}

fn schema_view(doc: &Value, name: &str, schema: &Value) -> SchemaView {
    let description = prose(schema["description"].as_str().unwrap_or_default());
    let variants: Vec<VariantView> = schema["oneOf"]
        .as_array()
        .into_iter()
        .flatten()
        .map(|v| {
            let title = v["title"]
                .as_str()
                .or_else(|| v["allOf"][0]["title"].as_str())
                .unwrap_or_default();
            let fields = fields(doc, v);
            VariantView {
                title: title.to_owned(),
                description: prose(v["description"].as_str().unwrap_or_default()),
                ty: if fields.is_empty() {
                    type_html(v)
                } else {
                    String::new()
                },
                fields,
            }
        })
        .collect();
    let fields = if variants.is_empty() {
        fields(doc, schema)
    } else {
        Vec::new()
    };
    let ty = if fields.is_empty() && variants.is_empty() {
        type_html(schema)
    } else {
        String::new()
    };
    SchemaView {
        name: name.to_owned(),
        description,
        ty,
        fields,
        variants,
    }
}

/// An object's properties, with an `allOf`'s parts merged in order.
fn fields(doc: &Value, schema: &Value) -> Vec<FieldView> {
    let mut out = Vec::new();
    collect_fields(doc, schema, &mut out);
    out
}

fn collect_fields(doc: &Value, schema: &Value, out: &mut Vec<FieldView>) {
    let schema = resolve(doc, schema);
    for part in schema["allOf"].as_array().into_iter().flatten() {
        collect_fields(doc, part, out);
    }
    let required: Vec<&str> = schema["required"]
        .as_array()
        .into_iter()
        .flatten()
        .filter_map(Value::as_str)
        .collect();
    for (name, property) in schema["properties"].as_object().into_iter().flatten() {
        if out.iter().any(|f| &f.name == name) {
            continue;
        }
        // A property's own description wins over the one on what it refers to.
        let description = property["description"]
            .as_str()
            .or_else(|| resolve(doc, property)["description"].as_str())
            .unwrap_or_default();
        out.push(FieldView {
            name: name.clone(),
            location: String::new(),
            required: required.contains(&name.as_str()),
            ty: type_html(property),
            description: prose(description),
        });
    }
}

/// A short, linked description of a schema's type, as HTML.
pub fn type_html(schema: &Value) -> String {
    if let Some(reference) = schema.get("$ref").and_then(Value::as_str) {
        let name = reference.rsplit('/').next().unwrap_or(reference);
        return format!("<a href=\"#schema-{0}\">{0}</a>", esc(name));
    }
    if let Some(value) = schema.get("const") {
        return format!("<code>{}</code>", esc(&value.to_string()));
    }
    if let Some(values) = schema.get("enum").and_then(Value::as_array) {
        return values
            .iter()
            .map(|v| format!("<code>{}</code>", esc(&v.to_string())))
            .collect::<Vec<_>>()
            .join(" | ");
    }
    for (key, joiner) in [("oneOf", " | "), ("anyOf", " | "), ("allOf", " &amp; ")] {
        if let Some(parts) = schema.get(key).and_then(Value::as_array) {
            return parts.iter().map(type_html).collect::<Vec<_>>().join(joiner);
        }
    }
    let types: Vec<&str> = match &schema["type"] {
        Value::String(t) => vec![t.as_str()],
        Value::Array(ts) => ts.iter().filter_map(Value::as_str).collect(),
        _ => return "any".into(),
    };
    types
        .into_iter()
        .map(|t| match t {
            "array" => {
                if let Some(items) = schema["prefixItems"].as_array() {
                    let items: Vec<String> = items.iter().map(type_html).collect();
                    format!("[{}]", items.join(", "))
                } else if schema.get("items").is_some() {
                    format!("array of {}", type_html(&schema["items"]))
                } else {
                    "array".into()
                }
            }
            "object" => match schema.get("additionalProperties") {
                Some(inner @ Value::Object(_)) => {
                    format!("map of string to {}", type_html(inner))
                }
                _ => "object".into(),
            },
            "string" => match schema["format"].as_str() {
                Some(format) => format!("string ({})", esc(format)),
                None => "string".into(),
            },
            other => esc(other),
        })
        .collect::<Vec<_>>()
        .join(" | ")
}

/// Escape text for HTML.
pub fn esc(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for c in text.chars() {
        match c {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '"' => out.push_str("&quot;"),
            '\'' => out.push_str("&#39;"),
            c => out.push(c),
        }
    }
    out
}

/// The little Markdown the document's descriptions use, as HTML: paragraphs
/// split on blank lines, and `code` spans. Everything else is escaped.
pub fn prose(text: &str) -> String {
    text.split("\n\n")
        .map(str::trim)
        .filter(|p| !p.is_empty())
        .map(|paragraph| {
            let mut html = String::from("<p>");
            for (i, piece) in paragraph.split('`').enumerate() {
                if i % 2 == 1 {
                    html.push_str("<code>");
                    html.push_str(&esc(piece));
                    html.push_str("</code>");
                } else {
                    html.push_str(&esc(piece));
                }
            }
            html.push_str("</p>");
            html
        })
        .collect()
}

/// Every `(method, path)` the document describes, paths without the prefix.
pub fn operations(doc: &Value) -> Vec<(&'static str, String)> {
    let empty = Map::new();
    let paths = doc["paths"].as_object().unwrap_or(&empty);
    let mut out = Vec::new();
    for (path, item) in paths {
        for method in METHODS {
            if item.get(method).is_some() {
                out.push((method, path.clone()));
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn prose_escapes_and_marks_code() {
        assert_eq!(
            prose("Use `a<b>` & go.\n\nNext."),
            "<p>Use <code>a&lt;b&gt;</code> &amp; go.</p><p>Next.</p>"
        );
    }

    #[test]
    fn types_link_to_schemas_and_spell_out_unions() {
        let schema = serde_json::json!({
            "type": ["array", "null"],
            "items": { "$ref": "#/components/schemas/GeoCell" }
        });
        assert_eq!(
            type_html(&schema),
            "array of <a href=\"#schema-GeoCell\">GeoCell</a> | null"
        );
    }

    #[test]
    fn every_reference_resolves() {
        fn walk(doc: &Value, v: &Value, at: &str) {
            match v {
                Value::Object(map) => {
                    if let Some(Value::String(r)) = map.get("$ref") {
                        assert!(!resolve(doc, v).is_null(), "{at}: $ref {r} names nothing");
                    }
                    for (k, child) in map {
                        walk(doc, child, &format!("{at}/{k}"));
                    }
                }
                Value::Array(items) => {
                    for (i, child) in items.iter().enumerate() {
                        walk(doc, child, &format!("{at}/{i}"));
                    }
                }
                _ => {}
            }
        }
        walk(&DOC, &DOC, "#");
    }

    #[test]
    fn every_operation_has_a_unique_id_a_known_tag_and_responses() {
        let doc = &*DOC;
        let tags: Vec<&str> = doc["tags"]
            .as_array()
            .unwrap()
            .iter()
            .filter_map(|t| t["name"].as_str())
            .collect();
        let mut ids = std::collections::BTreeSet::new();
        for (method, path) in operations(doc) {
            let op = &doc["paths"][&path][method];
            let id = op["operationId"].as_str().unwrap_or_else(|| {
                panic!("{method} {path} has no operationId");
            });
            assert!(ids.insert(id), "operationId {id} is used twice");
            let tag = op["tags"][0].as_str().unwrap_or_default();
            assert!(
                tags.contains(&tag),
                "{method} {path}: tag {tag:?} is not declared"
            );
            assert!(
                op["responses"].as_object().is_some_and(|r| !r.is_empty()),
                "{method} {path} has no responses"
            );
            // Path parameters must be declared, and only those in the path.
            let declared: Vec<String> = op["parameters"]
                .as_array()
                .into_iter()
                .flatten()
                .map(|p| resolve(doc, p))
                .filter(|p| p["in"] == "path")
                .filter_map(|p| p["name"].as_str().map(str::to_owned))
                .collect();
            let in_path: Vec<String> = path
                .split('/')
                .filter_map(|s| s.strip_prefix('{')?.strip_suffix('}'))
                .map(str::to_owned)
                .collect();
            assert_eq!(declared, in_path, "{method} {path}: path parameters");
        }
    }

    #[test]
    fn the_page_view_lists_every_operation_and_schema() {
        let view = &*VIEW;
        let ops: usize = view.tags.iter().map(|t| t.ops.len()).sum();
        assert_eq!(ops, operations(&DOC).len());
        assert_eq!(
            view.schemas.len(),
            DOC["components"]["schemas"].as_object().unwrap().len()
        );
    }
}
