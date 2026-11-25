use std::io::{Read, Write};
use std::net::TcpListener;
use std::path::PathBuf;

use super::*;

/// The area around 26.19 N, 98.24 W the size of a phone map: about 0.01 degrees.
fn small(zoom: f64) -> Area {
    Area {
        north: 26.195,
        south: 26.185,
        west: -98.245,
        east: -98.235,
        zoom,
    }
}

#[test]
fn the_plan_runs_from_the_zoom_on_screen_to_the_last_zoom_of_the_layer() {
    let plan = plan(&small(14.0), &[Layer::Imagery]);
    let zooms: Vec<u8> = plan.layers[0].1.iter().map(|s| s.zoom).collect();
    assert_eq!(zooms, [14, 15, 16]);
    // The patch of a hundredth of a degree is under half a tile wide at zoom 14, so at most 4 tiles, and more at 16.
    assert!(plan.layers[0].1[0].tiles() <= 4);
    assert!(plan.layers[0].1[2].tiles() > plan.layers[0].1[0].tiles());
    // Streets: 512 px tiles, so the zoom on screen is one more than the tile zoom, up to 14.
    let streets = super::plan(&small(14.0), &[Layer::Streets]);
    let zooms: Vec<u8> = streets.layers[0].1.iter().map(|s| s.zoom).collect();
    assert_eq!(zooms, [13, 14]);
}

#[test]
fn zoomed_closer_than_the_last_zoom_it_is_that_zoom_alone() {
    let plan = plan(&small(24.0), &[Layer::Imagery, Layer::Streets]);
    assert_eq!(plan.layers[0].1.len(), 1);
    assert_eq!(plan.layers[0].1[0].zoom, 16);
    assert_eq!(plan.layers[1].1.len(), 1);
    assert_eq!(plan.layers[1].1[0].zoom, 14);
}

#[test]
fn the_tiles_cover_the_corners_of_the_area() {
    let area = small(16.0);
    let plan = plan(&area, &[Layer::Imagery]);
    let span = plan.layers[0].1[0];
    let (x0, y0) = tile_at(walkers::lat_lon(area.north, area.west), 16);
    let (x1, y1) = tile_at(walkers::lat_lon(area.south, area.east), 16);
    assert_eq!((span.x0, span.y0, span.x1, span.y1), (x0, y0, x1, y1));
    assert_eq!(plan.list().len() as u64, plan.tiles());
}

#[test]
fn a_wide_area_over_the_cap_is_refused_with_a_message_to_zoom_in() {
    let wide = Area {
        north: 27.0,
        south: 25.0,
        west: -99.0,
        east: -97.0,
        zoom: 8.0,
    };
    let big = plan(&wide, &[Layer::Imagery]);
    assert!(big.too_big());
    assert!(big.tiles() > MAX_TILES);
    let text = estimate(&big);
    assert!(text.contains("more than the 20,000 allowed"), "{text}");
    assert!(text.contains("zoom in"), "{text}");
    let ok = plan(&small(15.0), &[Layer::Imagery]);
    assert!(!ok.too_big());
    let text = estimate(&ok);
    assert!(text.contains("tiles, zoom 15 to 16, about"), "{text}");
}

#[test]
fn both_layers_add_up_and_the_size_uses_each_layers_average_tile() {
    let one = plan(&small(15.0), &[Layer::Imagery]);
    let both = plan(&small(15.0), &[Layer::Imagery, Layer::Streets]);
    assert!(both.tiles() > one.tiles());
    assert_eq!(one.bytes(), one.tiles() * 26_000);
}

#[test]
fn numbers_and_sizes_read_as_people_write_them() {
    assert_eq!(grouped(0), "0");
    assert_eq!(grouped(999), "999");
    assert_eq!(grouped(1_000), "1,000");
    assert_eq!(grouped(20_000), "20,000");
    assert_eq!(grouped(1_234_567), "1,234,567");
    assert_eq!(size_text(800), "1 kB");
    assert_eq!(size_text(31_000_000), "31 MB");
    assert_eq!(size_text(2_500_000_000), "2.5 GB");
}

fn progress(done: u64, finished: bool) -> Progress {
    Progress {
        total: 5_000,
        done,
        fetched: 1_100,
        absent: 3,
        failed: 0,
        bytes: 28_600_000,
        finished,
        cancelled: false,
        gave_up: None,
    }
}

#[test]
fn the_progress_line_says_how_far_and_how_it_ended() {
    assert_eq!(
        progress_line(&progress(1_200, false)),
        "Downloading: 1,200 of 5,000 tiles (29 MB, 1,100 fetched, 3 not available, 0 failed)"
    );
    assert_eq!(
        progress_line(&progress(5_000, true)),
        "Downloaded 5,000 tiles, 29 MB new (1,100 fetched, 3 not available, 0 failed)"
    );
    let cancelled = Progress {
        cancelled: true,
        ..progress(1_200, true)
    };
    assert!(progress_line(&cancelled).starts_with("Cancelled after 1,200 of 5,000 tiles"));
    let gave_up = Progress {
        cancelled: true,
        gave_up: Some("the tile server does not answer".into()),
        ..progress(40, true)
    };
    assert!(progress_line(&gave_up).starts_with("Stopped, the tile server does not answer: 40 of"));
}

fn serve(tiles_answer: &'static str) -> String {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let base = format!("http://{}", listener.local_addr().unwrap());
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
            let body = b"0123456789";
            let head = format!(
                "HTTP/1.1 {tiles_answer}\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
                if tiles_answer.starts_with("200") { body.len() } else { 0 }
            );
            let _ = stream.write_all(head.as_bytes());
            if tiles_answer.starts_with("200") {
                let _ = stream.write_all(body);
            }
        }
    });
    base
}

fn fetcher(base: &str, dir: &PathBuf) -> Fetcher {
    let base = base.to_string();
    Fetcher::with(
        Some(dir.clone()),
        Arc::new(AtomicBool::new(false)),
        reqwest::blocking::Client::builder()
            .no_proxy()
            .build()
            .unwrap(),
        Arc::new(move |_, id| format!("{base}/{}/{}/{}", id.zoom, id.x, id.y)),
    )
}

fn wait(job: &Job) -> Progress {
    for _ in 0..1000 {
        let p = job.progress();
        if p.finished {
            return p;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    panic!("the download did not finish");
}

fn scratch(tag: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("marv-offline-{tag}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    dir
}

#[test]
fn a_download_stores_every_tile_and_a_second_one_fetches_nothing() {
    let base = serve("200 OK");
    let dir = scratch("job");
    let plan = plan(&small(15.0), &[Layer::Imagery]);
    let f = fetcher(&base, &dir);
    let ctx = egui::Context::default();
    let done = wait(&Job::start(&plan, &f, &ctx));
    assert_eq!(done.total, plan.tiles());
    assert_eq!(done.done, plan.tiles());
    assert_eq!(done.fetched, plan.tiles());
    assert_eq!(done.bytes, plan.tiles() * 10);
    assert_eq!((done.absent, done.failed), (0, 0));
    for (layer, id) in plan.list() {
        assert!(f.is_stored(layer, id), "{id:?}");
    }
    let again = wait(&Job::start(&plan, &f, &ctx));
    assert_eq!((again.done, again.fetched, again.bytes), (plan.tiles(), 0, 0));
    let _ = std::fs::remove_dir_all(dir);
}

#[test]
fn tiles_the_server_does_not_have_are_counted_and_do_not_stop_the_download() {
    let base = serve("404 Not Found");
    let dir = scratch("absent");
    let plan = plan(&small(15.0), &[Layer::Imagery]);
    let done = wait(&Job::start(&plan, &fetcher(&base, &dir), &egui::Context::default()));
    assert_eq!((done.absent, done.failed), (plan.tiles(), 0));
    assert_eq!(done.gave_up, None);
    let _ = std::fs::remove_dir_all(dir);
}

#[test]
fn a_cancelled_download_stops_and_says_so() {
    let base = serve("200 OK");
    let dir = scratch("cancel");
    let plan = plan(&small(12.0), &[Layer::Imagery]);
    let job = Job::start(&plan, &fetcher(&base, &dir), &egui::Context::default());
    job.cancel();
    let done = wait(&job);
    assert!(done.cancelled);
    assert!(done.done <= plan.tiles());
    assert!(progress_line(&done).starts_with("Cancelled after"));
    let _ = std::fs::remove_dir_all(dir);
}

#[test]
fn a_server_that_never_answers_stops_the_download() {
    let dead = TcpListener::bind("127.0.0.1:0").unwrap();
    let base = format!("http://{}", dead.local_addr().unwrap());
    drop(dead);
    let dir = scratch("dead");
    let area = Area {
        north: 26.25,
        south: 26.15,
        west: -98.3,
        east: -98.2,
        zoom: 14.0,
    };
    let plan = plan(&area, &[Layer::Imagery]);
    assert!(plan.tiles() > u64::from(GIVE_UP_AFTER) * 2);
    let done = wait(&Job::start(&plan, &fetcher(&base, &dir), &egui::Context::default()));
    assert!(done.gave_up.is_some());
    assert!(done.done < plan.tiles());
    assert!(!dir.exists());
}
