//! "Download this area": every tile of the visible map, from the zoom on screen down to the layer's last zoom,
//! into the tile folder (`tilestore.rs`), so the map can be drawn later without a network.

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use eframe::egui;
use walkers::TileId;

use super::tile_at;
use super::tilestore::{Fetched, Fetcher, Layer};

/// A download of more tiles than this is refused.
pub const MAX_TILES: u64 = 20_000;
/// Tiles fetched at once.
const THREADS: usize = 6;
/// Tries per tile after a failure.
const RETRIES: u32 = 2;
/// This many tiles failing in a row (the network is gone) stop the download.
const GIVE_UP_AFTER: u32 = 25;

/// The visible part of the map.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Area {
    pub north: f64,
    pub south: f64,
    pub west: f64,
    pub east: f64,
    /// The map's zoom.
    pub zoom: f64,
}

/// The tiles of one zoom: columns `x0..=x1`, rows `y0..=y1`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Span {
    pub zoom: u8,
    pub x0: u32,
    pub x1: u32,
    pub y0: u32,
    pub y1: u32,
}

impl Span {
    pub fn tiles(&self) -> u64 {
        u64::from(self.x1 - self.x0 + 1) * u64::from(self.y1 - self.y0 + 1)
    }
}

/// What a download would fetch.
#[derive(Debug, Clone, PartialEq)]
pub struct Plan {
    pub layers: Vec<(Layer, Vec<Span>)>,
}

impl Plan {
    pub fn tiles(&self) -> u64 {
        self.layers
            .iter()
            .flat_map(|(_, spans)| spans)
            .map(Span::tiles)
            .sum()
    }

    /// The most the download can weigh, from the layers' average tile.
    pub fn bytes(&self) -> u64 {
        self.layers
            .iter()
            .map(|(layer, spans)| spans.iter().map(Span::tiles).sum::<u64>() * layer.tile_bytes())
            .sum()
    }

    pub fn too_big(&self) -> bool {
        self.tiles() > MAX_TILES
    }

    /// Every tile, the zoomed-out ones first so that a download cut short still covers the area coarsely.
    fn list(&self) -> Vec<(Layer, TileId)> {
        let mut out = Vec::new();
        for (layer, spans) in &self.layers {
            let mut spans = spans.clone();
            spans.sort_by_key(|span| span.zoom);
            for span in spans {
                for y in span.y0..=span.y1 {
                    for x in span.x0..=span.x1 {
                        out.push((
                            *layer,
                            TileId {
                                x,
                                y,
                                zoom: span.zoom,
                            },
                        ));
                    }
                }
            }
        }
        out
    }
}

/// The tiles of `layers` that cover `area`, from the zoom of the tiles on screen down to the layer's last zoom
/// (just the last one when the map is zoomed closer than that).
pub fn plan(area: &Area, layers: &[Layer]) -> Plan {
    let north_west = walkers::lat_lon(area.north, area.west);
    let south_east = walkers::lat_lon(area.south, area.east);
    Plan {
        layers: layers
            .iter()
            .map(|&layer| {
                let spans = (layer.tile_zoom(area.zoom)..=layer.max_zoom())
                    .map(|zoom| {
                        let (x0, y0) = tile_at(north_west, zoom);
                        let (x1, y1) = tile_at(south_east, zoom);
                        Span {
                            zoom,
                            x0: x0.min(x1),
                            x1: x0.max(x1),
                            y0: y0.min(y1),
                            y1: y0.max(y1),
                        }
                    })
                    .collect();
                (layer, spans)
            })
            .collect(),
    }
}

/// 12345 as "12,345".
pub fn grouped(n: u64) -> String {
    let digits = n.to_string();
    let mut out = String::new();
    for (i, c) in digits.chars().enumerate() {
        if i > 0 && (digits.len() - i) % 3 == 0 {
            out.push(',');
        }
        out.push(c);
    }
    out
}

pub fn size_text(bytes: u64) -> String {
    if bytes >= 1_000_000_000 {
        format!("{:.1} GB", bytes as f64 / 1e9)
    } else if bytes >= 1_000_000 {
        format!("{:.0} MB", bytes as f64 / 1e6)
    } else {
        format!("{:.0} kB", (bytes as f64 / 1e3).max(1.0))
    }
}

/// The line shown before a download starts.
pub fn estimate(plan: &Plan) -> String {
    let tiles = plan.tiles();
    if plan.too_big() {
        return format!(
            "{} tiles is more than the {} allowed: zoom in to a smaller area",
            grouped(tiles),
            grouped(MAX_TILES)
        );
    }
    let (first, last) = plan
        .layers
        .iter()
        .flat_map(|(_, spans)| spans)
        .fold((u8::MAX, 0), |(lo, hi), span| {
            (lo.min(span.zoom), hi.max(span.zoom))
        });
    format!(
        "{} tiles, zoom {first} to {last}, about {} at most",
        grouped(tiles),
        size_text(plan.bytes())
    )
}

/// How far a download has come.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Progress {
    pub total: u64,
    /// Tiles dealt with: fetched, already on disk, absent or failed.
    pub done: u64,
    pub fetched: u64,
    pub absent: u64,
    pub failed: u64,
    pub bytes: u64,
    pub finished: bool,
    pub cancelled: bool,
    /// Why it stopped by itself.
    pub gave_up: Option<String>,
}

pub fn progress_line(p: &Progress) -> String {
    let counts = format!(
        "{} fetched, {} not available, {} failed",
        grouped(p.fetched),
        grouped(p.absent),
        grouped(p.failed)
    );
    if !p.finished {
        return format!(
            "Downloading: {} of {} tiles ({}, {})",
            grouped(p.done),
            grouped(p.total),
            size_text(p.bytes),
            counts
        );
    }
    if let Some(why) = &p.gave_up {
        return format!(
            "Stopped, {why}: {} of {} tiles done ({counts})",
            grouped(p.done),
            grouped(p.total)
        );
    }
    if p.cancelled {
        return format!(
            "Cancelled after {} of {} tiles ({counts})",
            grouped(p.done),
            grouped(p.total)
        );
    }
    format!(
        "Downloaded {} tiles, {} new ({counts})",
        grouped(p.total),
        size_text(p.bytes)
    )
}

struct Shared {
    total: u64,
    next: AtomicU64,
    done: AtomicU64,
    fetched: AtomicU64,
    absent: AtomicU64,
    failed: AtomicU64,
    bytes: AtomicU64,
    in_a_row: AtomicU32,
    active: AtomicU32,
    cancel: AtomicBool,
    gave_up: Mutex<Option<String>>,
}

/// A running or finished download.
pub struct Job {
    shared: Arc<Shared>,
}

impl Job {
    /// Starts fetching `plan` on worker threads. The tiles already on disk are left as they are.
    pub fn start(plan: &Plan, fetcher: &Fetcher, ctx: &egui::Context) -> Self {
        let list = Arc::new(plan.list());
        let shared = Arc::new(Shared {
            total: list.len() as u64,
            next: AtomicU64::new(0),
            done: AtomicU64::new(0),
            fetched: AtomicU64::new(0),
            absent: AtomicU64::new(0),
            failed: AtomicU64::new(0),
            bytes: AtomicU64::new(0),
            in_a_row: AtomicU32::new(0),
            active: AtomicU32::new(THREADS as u32),
            cancel: AtomicBool::new(false),
            gave_up: Mutex::new(None),
        });
        for _ in 0..THREADS {
            let (list, shared, fetcher, ctx) =
                (list.clone(), shared.clone(), fetcher.clone(), ctx.clone());
            std::thread::spawn(move || {
                run(&list, &shared, &fetcher);
                if shared.active.fetch_sub(1, Ordering::AcqRel) == 1 {
                    ctx.request_repaint();
                }
            });
        }
        Self { shared }
    }

    pub fn cancel(&self) {
        self.shared.cancel.store(true, Ordering::Relaxed);
    }

    pub fn progress(&self) -> Progress {
        let s = &self.shared;
        Progress {
            total: s.total,
            done: s.done.load(Ordering::Relaxed),
            fetched: s.fetched.load(Ordering::Relaxed),
            absent: s.absent.load(Ordering::Relaxed),
            failed: s.failed.load(Ordering::Relaxed),
            bytes: s.bytes.load(Ordering::Relaxed),
            finished: s.active.load(Ordering::Acquire) == 0,
            cancelled: s.cancel.load(Ordering::Relaxed),
            gave_up: s.gave_up.lock().ok().and_then(|why| why.clone()),
        }
    }
}

fn run(list: &[(Layer, TileId)], s: &Shared, fetcher: &Fetcher) {
    loop {
        if s.cancel.load(Ordering::Relaxed) {
            return;
        }
        let Ok(i) = usize::try_from(s.next.fetch_add(1, Ordering::Relaxed)) else {
            return;
        };
        let Some(&(layer, tile)) = list.get(i) else {
            return;
        };
        if fetcher.is_stored(layer, tile) {
            s.in_a_row.store(0, Ordering::Relaxed);
        } else {
            let mut answer = fetcher.fetch(layer, tile);
            for _ in 0..RETRIES {
                if !matches!(answer, Fetched::Failed(_)) || s.cancel.load(Ordering::Relaxed) {
                    break;
                }
                std::thread::sleep(Duration::from_millis(500));
                answer = fetcher.fetch(layer, tile);
            }
            match answer {
                Fetched::Bytes(bytes) => {
                    s.fetched.fetch_add(1, Ordering::Relaxed);
                    s.bytes.fetch_add(bytes.len() as u64, Ordering::Relaxed);
                    s.in_a_row.store(0, Ordering::Relaxed);
                }
                Fetched::Absent | Fetched::Offline => {
                    s.absent.fetch_add(1, Ordering::Relaxed);
                    s.in_a_row.store(0, Ordering::Relaxed);
                }
                Fetched::Failed(why) => {
                    s.failed.fetch_add(1, Ordering::Relaxed);
                    if s.in_a_row.fetch_add(1, Ordering::Relaxed) + 1 >= GIVE_UP_AFTER {
                        if let Ok(mut slot) = s.gave_up.lock() {
                            slot.get_or_insert(format!("the tile server does not answer ({why})"));
                        }
                        s.cancel.store(true, Ordering::Relaxed);
                    }
                }
            }
        }
        s.done.fetch_add(1, Ordering::Relaxed);
    }
}

/// The download control under the map's top row.
#[derive(Default)]
pub struct Panel {
    pub open: bool,
    /// Both layers, not only the one shown.
    both: bool,
    job: Option<Job>,
    /// The result of the last download, kept until the next.
    last: Option<String>,
}

impl Panel {
    pub fn ui(
        &mut self,
        ui: &mut egui::Ui,
        shown: Layer,
        area: Option<&Area>,
        fetcher: Option<&Fetcher>,
        ctx: &egui::Context,
    ) {
        if !self.open {
            return;
        }
        let running = self.job.as_ref().map(Job::progress);
        let busy = running.as_ref().is_some_and(|p| !p.finished);
        if let Some(p) = &running {
            if p.finished {
                self.last = Some(progress_line(p));
                self.job = None;
            } else {
                ctx.request_repaint_after(Duration::from_millis(250));
            }
        }
        ui.horizontal_wrapped(|ui| {
            ui.add_enabled(!busy, egui::Checkbox::new(&mut self.both, "Both map types"));
            let layers: Vec<Layer> = if self.both {
                Layer::ALL.to_vec()
            } else {
                vec![shown]
            };
            let plan = area.map(|area| plan(area, &layers));
            match (&running, &plan) {
                (Some(p), _) if busy => {
                    ui.label(progress_line(p));
                    if ui.button("Cancel").clicked()
                        && let Some(job) = &self.job
                    {
                        job.cancel();
                    }
                }
                (_, Some(plan)) => {
                    ui.label(estimate(plan));
                    let offline = fetcher.is_none_or(Fetcher::is_offline);
                    let start = ui.add_enabled(
                        !plan.too_big() && !offline,
                        egui::Button::new("Start download"),
                    );
                    if offline {
                        ui.label("Turn Offline off to download.");
                    }
                    if start.clicked()
                        && let Some(fetcher) = fetcher
                    {
                        self.last = None;
                        self.job = Some(Job::start(plan, fetcher, ctx));
                    }
                }
                _ => {
                    ui.label("No map area yet.");
                }
            }
            if !busy && ui.button("Close").clicked() {
                self.open = false;
            }
        });
        if let Some(last) = &self.last
            && !busy
        {
            ui.label(last);
        }
    }
}

#[cfg(test)]
#[path = "offline_tests.rs"]
mod tests;
