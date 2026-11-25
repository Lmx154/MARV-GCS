//! The map's tiles, kept on disk for good. A tile is read from its file when there is one, whatever its age;
//! only a missing tile is fetched, and only while "Offline" is off. Nothing here expires a file: the folder of
//! a layer (`<cache>/streets`, `<cache>/imagery`) is the offline map, and deleting it starts it over. Both the
//! tiles the map shows and the area download (`offline.rs`) go through `Fetcher`, so they fill the same files.

use std::collections::{HashMap, VecDeque};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{Receiver, Sender, channel};
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

use eframe::egui::{self, Rect, pos2};
use walkers::sources::{Attribution as SourceAttribution, TileSource as TileSourceTrait};
use walkers::{Style, Tile, TileId, TilePiece, Tiles};

use super::{ATTRIBUTION, OpenFreeMap};

/// What is drawn under the waypoints.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Layer {
    /// OpenFreeMap vector tiles, drawn by the station.
    Streets,
    /// USGS aerial imagery.
    Imagery,
}

/// Credit drawn on the map while the imagery is shown.
pub const IMAGERY_CREDIT: &str = "Imagery: USGS The National Map";

/// USGS imagery (The National Map, public domain; <https://basemap.nationalmap.gov>): 256 px JPEG tiles with
/// imagery to zoom 16 (a 17 is a 404 everywhere tried); the row comes before the column in the URL.
pub struct UsgsImagery;

/// The last zoom with imagery; closer, the zoom 16 tile is drawn larger.
pub const IMAGERY_ZOOM: u8 = 16;

impl TileSourceTrait for UsgsImagery {
    fn tile_url(&self, tile_id: TileId) -> String {
        format!(
            "https://basemap.nationalmap.gov/arcgis/rest/services/USGSImageryOnly/MapServer/tile/{}/{}/{}",
            tile_id.zoom, tile_id.y, tile_id.x
        )
    }

    fn attribution(&self) -> SourceAttribution {
        SourceAttribution {
            text: IMAGERY_CREDIT,
            url: "https://www.usgs.gov/programs/national-geospatial-program/national-map",
            logo_light: None,
            logo_dark: None,
        }
    }

    fn max_zoom(&self) -> u8 {
        IMAGERY_ZOOM
    }
}

impl Layer {
    pub const ALL: [Layer; 2] = [Layer::Streets, Layer::Imagery];

    pub fn name(self) -> &'static str {
        match self {
            Layer::Streets => "Streets",
            Layer::Imagery => "Imagery",
        }
    }

    /// The folder of the layer's tiles under the cache folder.
    pub fn folder(self) -> &'static str {
        match self {
            Layer::Streets => "streets",
            Layer::Imagery => "imagery",
        }
    }

    pub fn url(self, tile: TileId) -> String {
        match self {
            Layer::Streets => OpenFreeMap.tile_url(tile),
            Layer::Imagery => UsgsImagery.tile_url(tile),
        }
    }

    pub fn tile_size(self) -> u32 {
        match self {
            Layer::Streets => OpenFreeMap.tile_size(),
            Layer::Imagery => UsgsImagery.tile_size(),
        }
    }

    /// The last zoom with tiles.
    pub fn max_zoom(self) -> u8 {
        match self {
            Layer::Streets => OpenFreeMap.max_zoom(),
            Layer::Imagery => UsgsImagery.max_zoom(),
        }
    }

    /// The credit drawn on the map.
    pub fn credit(self) -> &'static str {
        match self {
            Layer::Streets => ATTRIBUTION,
            Layer::Imagery => IMAGERY_CREDIT,
        }
    }

    /// The zoom of the tiles drawn at the map zoom `zoom` (a 512 px tile stands for the zoom one closer, as
    /// walkers chooses), and no deeper than the layer has.
    pub fn tile_zoom(self, zoom: f64) -> u8 {
        let offset = f64::from(self.tile_size() / 256).log2().round();
        (zoom.round() - offset).clamp(0.0, f64::from(self.max_zoom())) as u8
    }

    /// What a tile of the layer weighs on disk, on average, for the size estimate.
    pub fn tile_bytes(self) -> u64 {
        match self {
            Layer::Streets => 45_000,
            Layer::Imagery => 26_000,
        }
    }
}

/// The URL of a tile.
pub type UrlOf = Arc<dyn Fn(Layer, TileId) -> String + Send + Sync>;

/// How one tile came out.
#[derive(Debug, PartialEq)]
pub enum Fetched {
    /// The tile's bytes.
    Bytes(Vec<u8>),
    /// The server has no such tile (kept as an empty file, so it is not asked again).
    Absent,
    /// Not on disk and the station is offline.
    Offline,
    /// The server or the network failed.
    Failed(String),
}

/// Reads tiles from disk and fetches the missing ones.
#[derive(Clone)]
pub struct Fetcher {
    root: Option<PathBuf>,
    pub offline: Arc<AtomicBool>,
    client: reqwest::blocking::Client,
    url: UrlOf,
}

static PART: AtomicU64 = AtomicU64::new(0);

impl Fetcher {
    /// `root`: the cache folder (`None`: nothing is kept).
    pub fn new(root: Option<PathBuf>, offline: Arc<AtomicBool>, user_agent: &str) -> Option<Self> {
        let client = reqwest::blocking::Client::builder()
            .user_agent(user_agent)
            .connect_timeout(Duration::from_secs(8))
            .timeout(Duration::from_secs(30))
            .build()
            .ok()?;
        Some(Self::with(root, offline, client, Arc::new(Layer::url)))
    }

    pub fn with(
        root: Option<PathBuf>,
        offline: Arc<AtomicBool>,
        client: reqwest::blocking::Client,
        url: UrlOf,
    ) -> Self {
        Self {
            root,
            offline,
            client,
            url,
        }
    }

    pub fn is_offline(&self) -> bool {
        self.offline.load(Ordering::Relaxed)
    }

    fn path(&self, layer: Layer, tile: TileId) -> Option<PathBuf> {
        Some(
            self.root
                .as_ref()?
                .join(layer.folder())
                .join(tile.zoom.to_string())
                .join(tile.x.to_string())
                .join(tile.y.to_string()),
        )
    }

    /// Whether the tile has a file (an empty one included).
    pub fn is_stored(&self, layer: Layer, tile: TileId) -> bool {
        self.path(layer, tile).is_some_and(|path| path.is_file())
    }

    /// The tile from its file, else from the server unless offline. Whatever the age of the file.
    pub fn get(&self, layer: Layer, tile: TileId) -> Fetched {
        if let Some(path) = self.path(layer, tile)
            && let Ok(bytes) = std::fs::read(path)
        {
            return if bytes.is_empty() {
                Fetched::Absent
            } else {
                Fetched::Bytes(bytes)
            };
        }
        if self.is_offline() {
            return Fetched::Offline;
        }
        self.fetch(layer, tile)
    }

    /// Asks the server for the tile and keeps the answer.
    pub fn fetch(&self, layer: Layer, tile: TileId) -> Fetched {
        let url = (self.url)(layer, tile);
        let response = match self.client.get(&url).send() {
            Ok(response) => response,
            Err(e) => return Fetched::Failed(e.without_url().to_string()),
        };
        let status = response.status();
        if status.as_u16() == 404 || status.as_u16() == 204 {
            self.keep(layer, tile, &[]);
            return Fetched::Absent;
        }
        if !status.is_success() {
            return Fetched::Failed(format!("HTTP {status}"));
        }
        match response.bytes() {
            Ok(bytes) if !bytes.is_empty() => {
                self.keep(layer, tile, &bytes);
                Fetched::Bytes(bytes.to_vec())
            }
            Ok(_) => Fetched::Failed("empty answer".to_string()),
            Err(e) => Fetched::Failed(e.without_url().to_string()),
        }
    }

    /// Writes the tile whole or not at all.
    fn keep(&self, layer: Layer, tile: TileId, bytes: &[u8]) {
        let Some(path) = self.path(layer, tile) else {
            return;
        };
        let Some(folder) = path.parent() else {
            return;
        };
        let part = folder.join(format!(
            "{}.part{}",
            tile.y,
            PART.fetch_add(1, Ordering::Relaxed)
        ));
        let written = std::fs::create_dir_all(folder)
            .and_then(|()| std::fs::write(&part, bytes))
            .and_then(|()| std::fs::rename(&part, &path));
        if written.is_err() {
            let _ = std::fs::remove_file(&part);
        }
    }

    /// Removes a file that holds no tile.
    fn discard(&self, layer: Layer, tile: TileId) {
        if let Some(path) = self.path(layer, tile) {
            let _ = std::fs::remove_file(path);
        }
    }
}

/// The tiles asked for and not yet taken, newest first.
struct Queue {
    items: Mutex<VecDeque<TileId>>,
    wake: Condvar,
    closed: AtomicBool,
}

/// The most tiles waiting for a worker; the oldest asked for are dropped (they have probably left the screen).
const QUEUE_MAX: usize = 96;
/// Threads that load the tiles of the map.
const WORKERS: usize = 4;
/// Tiles kept decoded; the least recently used go first.
const KEEP: usize = 600;
/// After a failure a tile is asked for again once this long has passed.
const RETRY_AFTER: Duration = Duration::from_secs(30);

enum Outcome {
    Loaded(Tile),
    /// The server has no such tile.
    Absent,
    /// Not on disk and offline: asked for again when "Offline" changes.
    Unavailable,
    /// Could not be fetched or decoded.
    Failed,
}

enum Entry {
    Pending,
    Ready(Tile),
    Missing(Retry),
}

enum Retry {
    Never,
    OnToggle,
    At(Instant),
}

/// The tiles of one layer for the map: [`Tiles`] over the files, drawn larger past the layer's last zoom.
pub struct DiskTiles {
    layer: Layer,
    fetcher: Fetcher,
    queue: Arc<Queue>,
    results: Receiver<(TileId, Outcome)>,
    slots: HashMap<TileId, (Entry, u64)>,
    clock: u64,
    was_offline: bool,
}

impl DiskTiles {
    pub fn new(layer: Layer, fetcher: Fetcher, style: Style, ctx: egui::Context) -> Self {
        let queue = Arc::new(Queue {
            items: Mutex::new(VecDeque::new()),
            wake: Condvar::new(),
            closed: AtomicBool::new(false),
        });
        let (sender, results) = channel();
        let style = Arc::new(style);
        for _ in 0..WORKERS {
            let (queue, fetcher, sender, style, ctx) = (
                queue.clone(),
                fetcher.clone(),
                sender.clone(),
                style.clone(),
                ctx.clone(),
            );
            std::thread::spawn(move || work(layer, &queue, &fetcher, &sender, &style, &ctx));
        }
        Self {
            layer,
            was_offline: fetcher.is_offline(),
            fetcher,
            queue,
            results,
            slots: HashMap::new(),
            clock: 0,
        }
    }

    fn take_results(&mut self) {
        while let Ok((tile, outcome)) = self.results.try_recv() {
            let entry = match outcome {
                Outcome::Loaded(tile) => Entry::Ready(tile),
                Outcome::Absent => Entry::Missing(Retry::Never),
                Outcome::Unavailable => Entry::Missing(Retry::OnToggle),
                Outcome::Failed => Entry::Missing(Retry::At(Instant::now() + RETRY_AFTER)),
            };
            self.slots.insert(tile, (entry, self.clock));
        }
    }

    fn request(&mut self, tile: TileId) {
        let dropped = {
            let Ok(mut items) = self.queue.items.lock() else {
                return;
            };
            items.push_front(tile);
            let dropped: Vec<TileId> = if items.len() > QUEUE_MAX {
                items.drain(QUEUE_MAX..).collect()
            } else {
                Vec::new()
            };
            self.queue.wake.notify_one();
            dropped
        };
        for old in dropped {
            self.slots.remove(&old);
        }
        self.slots.insert(tile, (Entry::Pending, self.clock));
    }

    fn evict(&mut self) {
        if self.slots.len() <= KEEP {
            return;
        }
        let mut stamps: Vec<u64> = self.slots.values().map(|(_, used)| *used).collect();
        stamps.sort_unstable();
        let oldest_kept = stamps[stamps.len() - KEEP * 2 / 3];
        self.slots.retain(|_, (_, used)| *used >= oldest_kept);
    }

    /// The tile if it is ready, else the part of the nearest zoomed-out one that covers the same ground.
    fn best_available(&mut self, tile: TileId) -> Option<TilePiece> {
        let mut zoom = tile.zoom;
        loop {
            let (from, uv) = piece_of(tile, zoom);
            if let Some((Entry::Ready(found), used)) = self.slots.get_mut(&from) {
                *used = self.clock;
                return Some(TilePiece::new(found.clone(), uv));
            }
            zoom = zoom.checked_sub(1)?;
        }
    }
}

impl Drop for DiskTiles {
    fn drop(&mut self) {
        self.queue.closed.store(true, Ordering::Relaxed);
        self.queue.wake.notify_all();
    }
}

/// The tile `tile` lies in at `zoom` (no deeper than `tile`), and the part of it that `tile` covers.
fn piece_of(tile: TileId, zoom: u8) -> (TileId, Rect) {
    let steps = u32::from(tile.zoom - zoom);
    let scale = 2u32.pow(steps);
    let (x, y) = (tile.x / scale, tile.y / scale);
    let (dx, dy) = ((tile.x % scale) as f32, (tile.y % scale) as f32);
    let size = (scale as f32).recip();
    (
        TileId { x, y, zoom },
        Rect::from_min_max(
            pos2(dx * size, dy * size),
            pos2(dx * size + size, dy * size + size),
        ),
    )
}

impl Tiles for DiskTiles {
    fn at(&mut self, tile: TileId) -> Option<TilePiece> {
        self.clock += 1;
        self.take_results();
        let offline = self.fetcher.is_offline();
        if offline != self.was_offline {
            self.was_offline = offline;
            self.slots
                .retain(|_, (entry, _)| !matches!(entry, Entry::Missing(Retry::OnToggle)));
        }
        if tile.zoom > 30 || tile.x >= 1 << tile.zoom || tile.y >= 1 << tile.zoom {
            return None;
        }
        let wanted = piece_of(tile, tile.zoom.min(self.layer.max_zoom())).0;
        let ask = match self.slots.get_mut(&wanted) {
            None => true,
            Some((entry, used)) => {
                *used = self.clock;
                matches!(entry, Entry::Missing(Retry::At(at)) if Instant::now() >= *at)
            }
        };
        if ask {
            self.request(wanted);
            self.evict();
        }
        self.best_available(tile)
    }

    fn attribution(&self) -> SourceAttribution {
        match self.layer {
            Layer::Streets => OpenFreeMap.attribution(),
            Layer::Imagery => UsgsImagery.attribution(),
        }
    }

    fn tile_size(&self) -> u32 {
        self.layer.tile_size()
    }
}

/// A worker: takes the newest request, loads the tile from disk or the server and decodes it.
fn work(
    layer: Layer,
    queue: &Queue,
    fetcher: &Fetcher,
    results: &Sender<(TileId, Outcome)>,
    style: &Style,
    ctx: &egui::Context,
) {
    loop {
        let tile = {
            let Ok(mut items) = queue.items.lock() else {
                return;
            };
            loop {
                if queue.closed.load(Ordering::Relaxed) {
                    return;
                }
                if let Some(tile) = items.pop_front() {
                    break tile;
                }
                items = match queue.wake.wait(items) {
                    Ok(items) => items,
                    Err(_) => return,
                };
            }
        };
        let outcome = match fetcher.get(layer, tile) {
            Fetched::Bytes(bytes) => {
                match Tile::new(&bytes, style, tile.zoom, layer.tile_size(), ctx) {
                    Ok(decoded) => Outcome::Loaded(decoded),
                    Err(_) => {
                        fetcher.discard(layer, tile);
                        Outcome::Failed
                    }
                }
            }
            Fetched::Absent => Outcome::Absent,
            Fetched::Offline => Outcome::Unavailable,
            Fetched::Failed(_) => Outcome::Failed,
        };
        if results.send((tile, outcome)).is_err() {
            return;
        }
        ctx.request_repaint();
    }
}

#[cfg(test)]
#[path = "tilestore_tests.rs"]
mod tests;
