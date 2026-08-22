# Website Style Guide

Canonical copy of this file lives here, in the `vishwamaggarwal.com` repo
(`STYLE_GUIDE.md` at repo root). Every project repo that has a `website/`
folder keeps its own copy at `website/STYLE_GUIDE.md` — same content,
copied by hand whenever this one changes (there's no automated sync).

**Who this is for**: anyone — human or Claude — writing or editing
`website/article.md`, `website/data.md`, `website/tool.md`, or
`website/app.html` in a project repo. Read this *before* writing new
website content in a project repo, so it looks and behaves consistently
with the rest of the site without needing to cross-check this repo.

If you only remember one thing: **you write semantic content (markdown,
plain HTML, inline SVG); the surrounding page chrome — nav, tag row,
status badge, buttons, back-links — comes from frontmatter and the
website repo's own page templates.** Don't hand-author chrome inside your
markdown body.

---

## 1. The `website/` folder contract

Every project repo the site pulls from keeps everything it feeds to
vishwamaggarwal.com under one `website/` folder instead of loose files at
repo root:

```
website/
  article.md     # long-form write-up (optional)
  data.md         # companion "full dataset" page (optional, needs article.md)
  tool.md         # Tools-section landing page content (optional — only if
                  #   this repo ships a browser tool)
  app.html        # the tool's actual self-contained web app (optional,
                  #   pairs with tool.md — generic filename, not
                  #   tool-specific, so every tool's app is predictably
                  #   website/app.html)
  images/
    <filename>    # images referenced by article.md/data.md/tool.md
  STYLE_GUIDE.md  # this file
```

Only include the files this repo actually needs — a repo with no browser
tool has no `tool.md`/`app.html`; a repo with no companion dataset has no
`data.md`.

**To register a new source**: add one entry to the relevant array
(`articleSources`, `toolPageSources`, `toolAppSources`) in the website
repo's `src/content.config.ts`. See that file's own comments for the
exact shape. This step happens in the website repo, not here — if you're
working in a project repo alone, tell the user this step is still needed
there.

---

## 2. Content types & frontmatter

Every file below is standard markdown with YAML frontmatter, parsed and
schema-validated by the website's Content Layer loader. Extra frontmatter
fields are ignored, not an error — but a missing *required* field fails
that source's fetch (logged as a build warning, non-fatal to the rest of
the site).

### `article.md`

```yaml
---
title: "Your Motor Doesn't Know How To Get There"
description: "One-sentence summary — used in <meta description>, og:description, and article list previews."
pubDate: 2026-08-22
tags: ["Robotics", "Embedded", "Motion Control"]
draft: true
---
```

- `title`, `description`, `pubDate` are required. `tags` defaults to `[]`.
- Rendered at `/articles/<id>/`, inside the site's narrow **prose** column
  (see §3) — this is continuous long-form reading content.

### `data.md` (optional companion to `article.md`)

```yaml
---
title: "Your Motor Doesn't Know How To Get There — Full Dataset"
description: "Every plot and the full dataset behind the article."
draft: true
---
```

- Rendered at `/articles/<id>/data/`. Link to it from the bottom of
  `article.md` by hand, wherever it reads naturally — it's not linked
  from anywhere else, deliberately: a page for the reader who wants more
  than the article chose to show, not a second front door into the
  content.
- Also prose-column width, but `.chart-figure` is allowed to break out
  wider on screens ≥56rem here specifically (dense multi-subplot grids
  need the room). `article.md`'s charts intentionally stay at the
  narrower text-column width — the two pages are allowed to disagree on
  this.

### `tool.md`

```yaml
---
title: "Servo Calibrator"
description: "One-sentence summary of what the tool does."
tags: ["Web Serial", "Robotics", "Arduino"]
status: active   # active | shipped | archived
repo: "https://github.com/owner/repo"
draft: true
---
```

- `title`, `description`, `repo` are required. `tags` defaults to `[]`,
  `status` defaults to `active`.
- Rendered at `/tools/<id>/` in the site's **wide** column (see §3) —
  this is a landing page, not continuous prose.
- The page template supplies the back-link, the tag row + status badge
  (from `tags`/`status`), and a fixed button row (**Launch** →
  `/tools/<id>/app/`, **View on GitHub** → `repo`) automatically. Your
  markdown body is just the content between those: what the tool does,
  wiring/setup, safety notes, how to get it running. Don't add your own
  "Launch"/"View on GitHub" buttons or repeat the tag/status badges
  inside the body.

### `app.html`

The tool's actual web app — not templated by the site at all, served
byte-for-byte as a static file at `/tools/<id>/app/`. It's a separate,
self-contained page (its own `<html>`, its own styling); it does **not**
need to match this style guide's look. Keep it self-contained (no
external script/asset dependencies) so it works when served as a bare
static file.

### ⚠️ The draft default is "published," not "draft"

**`draft` defaults to `false` (published) if you omit it.** There is no
safety net — a file with no `draft` field goes live the moment its
source repo is registered and the site rebuilds. Always write
`draft: true` explicitly while a piece is still being drafted, and only
remove it (or flip it to `false`) once you have explicit go-ahead to
publish.

---

## 3. Layout: two intentional widths, not a bug

The site uses two content-column widths, both deliberate:

| Token | Width | Used for |
|---|---|---|
| `.wrap` | 60rem | Home, project/tool listing pages, **tool landing pages** (`tool.md`) — anything with a "page" structure: headings, short sections, cards, button rows |
| `.prose-frame` | 43rem (nested inside `.wrap`) | Continuous long-form reading content — `article.md`, `data.md` |

**Rule of thumb**: if what you're writing reads like a landing page
(short sections, a diagram, a button row at the end) → wide, don't wrap
it. If it reads like an essay (paragraph after paragraph, the reader is
expected to read start to finish) → narrow, prose-optimized for line
length.

This is decided by which page template renders the content, not by
anything you write in the markdown file itself — `tool.md` always renders
wide, `article.md`/`data.md` always render narrow. Nothing to configure
per-file.

---

## 4. Visual language

### Theme

Single dark "blueprint" theme — deliberately no light/dark toggle, no
`prefers-color-scheme` split. Don't add one; a blueprint is definitionally
dark blue with pale linework.

### Color tokens

Reference these by CSS custom property name in any hand-written HTML/SVG
— never hardcode a hex value in content. The site may re-theme these
tokens later; content that references them by name stays correct, content
with hardcoded hex values won't.

| Token | Meaning |
|---|---|
| `--bg`, `--bg-deep`, `--bg-raised`, `--bg-raised-alt` | Page background, darker recess, raised card surface, a further-raised variant |
| `--text`, `--text-dim`, `--text-faint` | Primary / secondary / tertiary text |
| `--border`, `--border-strong` | Hairline / emphasized borders |
| `--accent`, `--accent-ink`, `--accent-soft` | Brand accent (orange) / text color for on-accent surfaces / a soft accent wash |
| `--danger` | Error/warning state |
| `--series-1`, `--series-2` | The two-way categorical palette for any chart comparing exactly two things (e.g. "naive" vs "calibrated") |
| `--model-*` (`-linear2`, `-table10` … `-table50`) | A five-way gradient for charts comparing many variants at once — deliberately close together in hue, not a rainbow, when the variants are all close in value (a rainbow would visually overstate differences that aren't really there) |
| `--font-body`, `--font-display`, `--font-mono` | Body copy / heading (condensed display) / monospace (labels, code, timestamps, tags) |

### Typography

- Headings (`h1`–`h4`) use `--font-display`, bold, tight line-height.
  Page-level headings (article/tool page `h1`, section `h2`s) render
  uppercase automatically via the surrounding template — you don't add
  `text-transform` yourself.
- Body copy uses `--font-body`. Secondary/muted text (captions, card
  descriptions) uses `--text-dim` or `--text-faint`, not full-opacity
  `--text`.
- `--font-mono` is for anything label-like: tags, status badges,
  timestamps, inline `code`, axis labels/legends inside charts.
- Don't hand-set `font-size`/`font-family` in your markdown body — write
  semantic HTML (`h2`, `h3`, `p`, `ul`, `table`, `img`) and the page's
  existing styles handle the look. The one place you do set styles by
  hand is inside a hand-built chart SVG (see §5).

### Components the templates already provide (don't hand-author these)

- **Tag row + status badge** (`.tag-row`, `.tag`, `.status-badge`) — driven
  by `tags`/`status` frontmatter on tool and project pages.
- **Button row** (`.button-row`, `.btn`, `.btn.primary`) — the tool page's
  Launch/GitHub buttons are supplied automatically from `repo` frontmatter
  and the tool's slug.
- **Card grid** (`.project-grid`, `.project-card`, `.project-links`) — the
  Tools/Projects listing pages build these from each entry's frontmatter.

You will never write these class names by hand in `article.md`/`tool.md`
— they belong to the page template, not the content.

---

## 5. Charts & diagrams (inline SVG)

Data charts and technical diagrams are hand-authored inline SVG, wrapped
in a `.chart-figure` card:

```html
<div class="chart-figure">
<p class="chart-title">Short label for what this chart shows</p>
<svg viewBox="0 0 680 260" role="img" aria-label="A full plain-language description of what the chart shows and its key takeaway — this is the only description a screen reader gets.">
  <!-- gridlines: stroke="var(--border)" -->
  <!-- axis labels: font-family="var(--font-mono)" fill="var(--text-faint)" -->
  <!-- data: stroke/fill="var(--series-1)" or "var(--series-2)" (two-way) or "var(--model-*)" (many-way) -->
</svg>
<div class="chart-legend">
  <span><span class="swatch" style="background: var(--series-1);"></span>Label for series 1</span>
  <span><span class="swatch" style="background: var(--series-2);"></span>Label for series 2</span>
</div>
<p class="chart-caption">One or two sentences of caption/context below the chart.</p>
</div>
```

Rules:

- **Always reference colors via `var(--token)`, never a literal hex.**
  `.chart-figure` locally redefines every one of these token *names* to a
  light palette (white background, dark ink, MATLAB-style axis colors) —
  charts render light-on-white for data-reading clarity even though the
  rest of the page is dark. An SVG that hardcodes a hex color instead of
  `var(--text)`/`var(--series-1)`/etc. won't pick up that remap and will
  look wrong (often invisible — e.g. pale text on the chart's white
  background).
- `viewBox` only, no explicit `width`/`height` attributes on the `<svg>` —
  it needs to scale to its container.
- `role="img"` and a full descriptive `aria-label` on every `<svg>` are
  required — screen readers cannot parse SVG internals, so the
  `aria-label` is the chart's entire accessible content. Describe the
  actual finding/shape, not just "a chart of X".
- `.chart-legend`/`.swatch` for a manual color-key when the chart itself
  doesn't label series inline.
- `.chart-title` above the SVG, `.chart-caption` below it — both plain
  `<p>` tags, not part of the SVG.
- Many small similar charts (e.g. one subplot per unit/case)? Use
  `.subplot-grid` > `.subplot` (each holding its own small `<svg>` +
  `.subplot-caption`) inside one `.chart-figure`, instead of one full
  `.chart-figure` per item — reads as one figure with several panels
  instead of a wall of near-identical cards.

A **plain content image** (a photo, a wiring diagram photo/rendering) is
not a chart — just use normal markdown `![alt text](path)`. It's already
styled (bordered, centered, shadowed) with no wrapper needed. Reserve
`.chart-figure` for actual data visualizations/technical diagrams.

---

## 6. Images

- Store under `website/images/` in this repo.
- Reference in markdown as `/images/<this-repo's-content-id>/<filename>`
  — the `id` is whatever this repo's source entry is registered under in
  the website's `content.config.ts` (ask if you don't know it, or check
  that repo's `articleSources`/`toolPageSources` entry). The website's
  build fetches the file from `website/images/<filename>` here and writes
  it to that URL — the path in your markdown is the *destination* URL,
  not a path within this repo.
- No automatic image optimization exists yet — keep source images
  reasonably sized before committing.

---

## 7. The Astro markdown gotcha: no blank line inside a raw HTML block

The site's markdown renderer drops back into Markdown mode the instant it
sees a blank line inside a raw HTML block (a `<div>`, `<svg>`, `<figure>`,
etc.) and renders everything after that point as an escaped code block
instead of real markup — silently, with no build error. Keep every raw
HTML/SVG block (charts, custom wrappers) as one continuous block of
non-blank lines from its opening tag to its closing tag. This is the most
common way hand-written charts break; if a chart renders as visible
`<div>`/`<svg>` source text instead of an actual chart, check for a blank
line inside it first.

A plain GFM markdown table doesn't have this problem and needs no special
handling — wide tables already scroll within their own container
automatically.

---

## 8. Before publishing: quick checklist

- [ ] `draft: true` explicitly set while still drafting (see §2's warning
      — the schema default is *published*)
- [ ] Every hand-written SVG uses `var(--token)` colors, not hex
- [ ] Every `<svg>` has `role="img"` and a real descriptive `aria-label`
- [ ] No blank line inside any raw HTML/SVG block
- [ ] Images live in `website/images/`, referenced by their website URL
      (`/images/<id>/<filename>`), not a repo-relative path
- [ ] `tool.md` doesn't re-implement the tag row, status badge, or
      Launch/GitHub buttons — those come from frontmatter automatically
- [ ] This repo's `website/STYLE_GUIDE.md` matches the canonical copy in
      the `vishwamaggarwal.com` repo (diff them if unsure)
