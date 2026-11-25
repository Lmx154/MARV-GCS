use std::io::{Read, Write};
use std::net::TcpListener;
use std::sync::atomic::AtomicUsize;

use super::*;

fn tile(zoom: u8, x: u32, y: u32) -> TileId {
    TileId { x, y, zoom }
}

/// A scratch folder of this test.
fn folder(tag: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("marv-tilestore-{tag}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    dir
}

/// A server on this computer: `/ok/..` answers 200 with `body`, `/gone/..` 404, anything else 500. It counts the
/// requests.
fn serve(body: Vec<u8>) -> (String, Arc<AtomicUsize>) {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let base = format!("http://{}", listener.local_addr().unwrap());
    let hits = Arc::new(AtomicUsize::new(0));
    let counted = hits.clone();
    std::thread::spawn(move || {
        for stream in listener.incoming() {
            let Ok(mut stream) = stream else { continue };
            let mut request = Vec::new();
            let mut chunk = [0u8; 512];
            while !request.windows(4).any(|w| w == b"\r\n\r\n") {
                match stream.read(&mut chunk) {
                    Ok(0) | Err(_) => break,
                    Ok(n) => request.extend_from_slice(&chunk[..n]),
                }
            }
            counted.fetch_add(1, Ordering::SeqCst);
            let line = String::from_utf8_lossy(&request);
            let path = line.split_whitespace().nth(1).unwrap_or("").to_string();
            let (status, payload) = if path.starts_with("/ok/") {
                ("200 OK", body.clone())
            } else if path.starts_with("/gone/") {
                ("404 Not Found", Vec::new())
            } else {
                ("500 Internal Server Error", Vec::new())
            };
            let head = format!(
                "HTTP/1.1 {status}\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
                payload.len()
            );
            let _ = stream.write_all(head.as_bytes());
            let _ = stream.write_all(&payload);
        }
    });
    (base, hits)
}

fn fetcher(root: Option<PathBuf>, base: &str, offline: bool, path: &'static str) -> Fetcher {
    let base = base.to_string();
    Fetcher::with(
        root,
        Arc::new(AtomicBool::new(offline)),
        reqwest::blocking::Client::builder()
            .no_proxy()
            .build()
            .unwrap(),
        Arc::new(move |_, id| format!("{base}/{path}/{}/{}/{}", id.zoom, id.x, id.y)),
    )
}

fn png() -> Vec<u8> {
    let mut out = Vec::new();
    let mut encoder = png::Encoder::new(&mut out, 2, 2);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    encoder
        .write_header()
        .unwrap()
        .write_image_data(&[255u8; 16])
        .unwrap();
    out
}

#[test]
fn the_imagery_url_has_the_row_before_the_column_and_stops_at_zoom_16() {
    assert_eq!(
        Layer::Imagery.url(tile(16, 14883, 27824)),
        "https://basemap.nationalmap.gov/arcgis/rest/services/USGSImageryOnly/MapServer/tile/16/27824/14883"
    );
    assert_eq!(Layer::Imagery.max_zoom(), 16);
    assert_eq!(Layer::Imagery.tile_size(), 256);
    assert_eq!(Layer::Streets.max_zoom(), 14);
    assert_eq!(Layer::Imagery.credit(), "Imagery: USGS The National Map");
    assert_eq!(
        UsgsImagery.attribution().text,
        "Imagery: USGS The National Map"
    );
}

#[test]
fn the_zoom_of_the_tiles_drawn_follows_the_tile_size_and_stops_at_the_last_zoom() {
    assert_eq!(Layer::Imagery.tile_zoom(12.4), 12);
    assert_eq!(Layer::Imagery.tile_zoom(26.0), 16);
    assert_eq!(Layer::Streets.tile_zoom(12.4), 11);
    assert_eq!(Layer::Streets.tile_zoom(26.0), 14);
    assert_eq!(Layer::Streets.tile_zoom(0.0), 0);
}

#[test]
fn a_tile_closer_than_the_last_zoom_is_a_part_of_the_last_one() {
    let (from, uv) = piece_of(tile(18, 4 * 100 + 3, 4 * 200 + 1), 16);
    assert_eq!(from, tile(16, 100, 200));
    assert_eq!(uv, Rect::from_min_max(pos2(0.75, 0.25), pos2(1.0, 0.5)));
    let (same, whole) = piece_of(tile(5, 3, 4), 5);
    assert_eq!(same, tile(5, 3, 4));
    assert_eq!(whole, Rect::from_min_max(pos2(0.0, 0.0), pos2(1.0, 1.0)));
}

#[test]
fn a_fetched_tile_is_kept_and_then_read_from_disk_without_the_network_even_when_old() {
    let (base, hits) = serve(b"tile bytes".to_vec());
    let dir = folder("keep");
    let online = fetcher(Some(dir.clone()), &base, false, "ok");
    assert_eq!(
        online.get(Layer::Imagery, tile(3, 1, 2)),
        Fetched::Bytes(b"tile bytes".to_vec())
    );
    assert_eq!(hits.load(Ordering::SeqCst), 1);
    let file = dir.join("imagery/3/1/2");
    assert_eq!(std::fs::read(&file).unwrap(), b"tile bytes");
    // A file from long ago is as good as a new one.
    let old = std::time::SystemTime::UNIX_EPOCH + Duration::from_secs(1_000_000);
    std::fs::File::options()
        .write(true)
        .open(&file)
        .unwrap()
        .set_modified(old)
        .unwrap();
    for offline in [false, true] {
        let again = fetcher(Some(dir.clone()), &base, offline, "ok");
        assert_eq!(
            again.get(Layer::Imagery, tile(3, 1, 2)),
            Fetched::Bytes(b"tile bytes".to_vec())
        );
    }
    assert_eq!(hits.load(Ordering::SeqCst), 1, "no second request");
    // The other layer's folder is separate.
    assert_eq!(
        fetcher(Some(dir.clone()), &base, true, "ok").get(Layer::Streets, tile(3, 1, 2)),
        Fetched::Offline
    );
    let _ = std::fs::remove_dir_all(dir);
}

#[test]
fn offline_a_missing_tile_is_not_asked_for() {
    let (base, hits) = serve(Vec::new());
    let dir = folder("offline");
    let f = fetcher(Some(dir.clone()), &base, true, "ok");
    assert_eq!(f.get(Layer::Imagery, tile(2, 0, 0)), Fetched::Offline);
    assert_eq!(hits.load(Ordering::SeqCst), 0);
    assert!(!dir.exists());
}

#[test]
fn a_tile_the_server_does_not_have_is_remembered_and_a_server_error_is_not() {
    let (base, hits) = serve(b"x".to_vec());
    let dir = folder("absent");
    let gone = fetcher(Some(dir.clone()), &base, false, "gone");
    assert_eq!(gone.get(Layer::Imagery, tile(4, 1, 1)), Fetched::Absent);
    assert_eq!(gone.get(Layer::Imagery, tile(4, 1, 1)), Fetched::Absent);
    assert_eq!(hits.load(Ordering::SeqCst), 1);
    let broken = fetcher(Some(dir.clone()), &base, false, "broken");
    assert!(matches!(
        broken.get(Layer::Imagery, tile(4, 2, 2)),
        Fetched::Failed(_)
    ));
    assert!(!broken.is_stored(Layer::Imagery, tile(4, 2, 2)));
    let _ = std::fs::remove_dir_all(dir);
}

#[test]
fn an_unreachable_server_is_a_failure_and_nothing_is_kept() {
    let dir = folder("down");
    let port = TcpListener::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .port();
    let f = fetcher(
        Some(dir.clone()),
        &format!("http://127.0.0.1:{port}"),
        false,
        "ok",
    );
    assert!(matches!(
        f.get(Layer::Imagery, tile(1, 0, 0)),
        Fetched::Failed(_)
    ));
    assert!(!dir.exists());
}

fn wait_for(tiles: &mut DiskTiles, id: TileId) -> Option<TilePiece> {
    for _ in 0..200 {
        if let Some(piece) = tiles.at(id) {
            return Some(piece);
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    None
}

#[test]
fn the_map_draws_a_stored_tile_offline_and_over_zoomed_from_the_last_zoom() {
    let dir = folder("draw");
    let path = dir.join("imagery/16/100/200");
    std::fs::create_dir_all(path.parent().unwrap()).unwrap();
    std::fs::write(&path, png()).unwrap();
    let f = fetcher(Some(dir.clone()), "http://127.0.0.1:9", true, "ok");
    let mut tiles = DiskTiles::new(
        Layer::Imagery,
        f,
        Style::default(),
        egui::Context::default(),
    );
    let piece = wait_for(&mut tiles, tile(16, 100, 200)).expect("the stored tile");
    assert_eq!(piece.uv, Rect::from_min_max(pos2(0.0, 0.0), pos2(1.0, 1.0)));
    let closer = wait_for(&mut tiles, tile(20, 16 * 100 + 5, 16 * 200 + 15)).unwrap();
    assert_eq!(
        closer.uv,
        Rect::from_min_max(pos2(5.0 / 16.0, 15.0 / 16.0), pos2(6.0 / 16.0, 1.0))
    );
    // A tile off the grid is nothing.
    assert!(tiles.at(tile(3, 8, 0)).is_none());
    drop(tiles);
    let _ = std::fs::remove_dir_all(dir);
}

#[test]
fn a_tile_that_is_not_an_image_is_discarded_so_it_is_fetched_again() {
    let dir = folder("bad");
    let path = dir.join("imagery/2/1/1");
    std::fs::create_dir_all(path.parent().unwrap()).unwrap();
    std::fs::write(&path, b"not an image").unwrap();
    let f = fetcher(Some(dir.clone()), "http://127.0.0.1:9", true, "ok");
    let mut tiles = DiskTiles::new(
        Layer::Imagery,
        f,
        Style::default(),
        egui::Context::default(),
    );
    assert!(wait_for(&mut tiles, tile(2, 1, 1)).is_none());
    assert!(!path.exists());
    drop(tiles);
    let _ = std::fs::remove_dir_all(dir);
}

/// Needs a network: `cargo test -p marv-gcs -- --ignored live`.
#[test]
#[ignore]
fn live_the_real_servers_answer_over_https_and_the_tiles_decode() {
    let dir = folder("live");
    let f = Fetcher::new(
        Some(dir.clone()),
        Arc::new(AtomicBool::new(false)),
        crate::views::map::USER_AGENT,
    )
    .unwrap();
    let Fetched::Bytes(jpeg) = f.get(Layer::Imagery, tile(16, 14883, 27824)) else {
        panic!("no imagery at zoom 16");
    };
    assert_eq!(&jpeg[..2], [0xFF, 0xD8]);
    assert_eq!(
        f.get(Layer::Imagery, tile(17, 29767, 55649)),
        Fetched::Absent
    );
    let mut streets = DiskTiles::new(
        Layer::Streets,
        f.clone(),
        crate::views::map::dark_style(),
        egui::Context::default(),
    );
    let street = tile(14, 3720, 6956);
    assert!(wait_for(&mut streets, street).is_some(), "street tile");
    let mut imagery = DiskTiles::new(
        Layer::Imagery,
        f,
        Style::default(),
        egui::Context::default(),
    );
    assert!(wait_for(&mut imagery, tile(16, 14883, 27824)).is_some());
    let _ = std::fs::remove_dir_all(dir);
}
