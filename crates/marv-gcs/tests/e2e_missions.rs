//! End to end: the Map tab's Upload, Read and Clear against the simulated vehicle's mission service over a real
//! TCP socket, in real time (docs/link-contract.md, section 17). The window is the real Map view drawn by
//! egui_kittest over the real session's state; its commands go to the session. Nothing is mocked. The points are
//! the contract's example (made-up points in the open ocean).

use std::cell::RefCell;
use std::net::TcpListener;
use std::path::PathBuf;
use std::rc::Rc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, PoisonError};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use eframe::egui;
use egui_kittest::Harness;
use egui_kittest::kittest::{NodeT, Queryable};
use gcs_mavlink::services::missions::MissionStatus;
use gcs_mavlink::{Access, Command, LinkStatus, Session, SessionConfig};
use gcs_sim::{SimConfig, SimVehicle};
use gcs_transport::Endpoint;
use marv_gcs::missions::{Store, Waypoint};
use marv_gcs::views::{self, ViewCtx};

const POLL: Duration = Duration::from_millis(20);
const UPLOAD: &str = "Upload to vehicle";
const READ: &str = "Read from vehicle";
const CLEAR: &str = "Clear on vehicle";
const COPY: &str = "Copy to a new list";
const MATCHES: &str = "The vehicle's list matches the edited list";
const VERIFIED_3: &str = "Upload verified: the vehicle holds 3 waypoints, plan id 0x99C3BD49";
const LEGEND: &str = "Edited list: amber discs. Vehicle's list as last read: blue dashed rings.";

fn example() -> Vec<Waypoint> {
    [
        (10.0, -30.0, 10.0),
        (10.001, -30.0, 12.5),
        (10.001, -29.999, 15.0),
    ]
    .into_iter()
    .map(|(lat, lon, alt_m)| Waypoint { lat, lon, alt_m })
    .collect()
}

struct Rig {
    vehicle: Arc<Mutex<SimVehicle>>,
    stop: Arc<AtomicBool>,
    server: Option<JoinHandle<()>>,
    session: Rc<Session>,
}

impl Rig {
    fn start(config: SimConfig) -> Rig {
        let listener = TcpListener::bind("127.0.0.1:0").expect("bind");
        let addr = listener.local_addr().expect("address").to_string();
        let vehicle = Arc::new(Mutex::new(SimVehicle::new(config)));
        let stop = Arc::new(AtomicBool::new(false));
        let server = {
            let vehicle = Arc::clone(&vehicle);
            let stop = Arc::clone(&stop);
            std::thread::spawn(move || gcs_sim::serve_tcp(listener, vehicle, stop))
        };
        let link = gcs_transport::open(&Endpoint::Tcp { addr }).expect("open the link");
        let session = Session::start(
            link.reader,
            link.writer,
            SessionConfig {
                access: Access::Configure,
                ..SessionConfig::default()
            },
            Box::new(|| {}),
        );
        let rig = Rig {
            vehicle,
            stop,
            server: Some(server),
            session: Rc::new(session),
        };
        let deadline = Instant::now() + Duration::from_secs(10);
        while !{
            let state = rig.session.state();
            state.link.status == LinkStatus::Connected && state.link.vehicle.is_some()
        } {
            assert!(Instant::now() < deadline, "never connected");
            std::thread::sleep(POLL);
        }
        rig
    }

    fn arm(&self) {
        self.vehicle
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .set_armed(true);
    }
}

impl Drop for Rig {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(server) = self.server.take() {
            let _ = server.join();
        }
    }
}

fn temp_store(tag: &str) -> Store {
    let dir: PathBuf = std::env::temp_dir().join(format!(
        "marv-gcs-e2e-missions-{tag}-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    Store::new(dir)
}

/// The Map view over the real session's state, with its lists in `store`.
struct Window {
    harness: Harness<'static>,
    store: Store,
    forwarded: Rc<RefCell<Vec<String>>>,
}

impl Window {
    fn open(session: &Rc<Session>, store: &Store) -> Window {
        let session = Rc::clone(session);
        let forwarded: Rc<RefCell<Vec<String>>> = Rc::default();
        let started = Instant::now();
        let mut view = views::map_with_store(Some(store.clone()));
        let f = forwarded.clone();
        let mut harness = Harness::builder()
            .with_size(egui::vec2(1500.0, 1300.0))
            .build_ui(move |ui| {
                let state = session.state().clone();
                let mut commands: Vec<Command> = Vec::new();
                let mut cx = ViewCtx {
                    state: &state,
                    commands: &mut commands,
                    now: Some(started.elapsed()),
                    access: Access::Configure,
                };
                view.ui(ui, &mut cx);
                for command in commands {
                    f.borrow_mut().push(format!("{command:?}"));
                    session.send(command);
                }
            });
        harness.run();
        Window {
            harness,
            store: store.clone(),
            forwarded,
        }
    }

    fn shows(&self, text: &str) -> bool {
        self.harness.query_all_by_label(text).next().is_some()
    }

    fn shows_containing(&self, text: &str) -> bool {
        self.harness
            .query_all_by_label_contains(text)
            .next()
            .is_some()
    }

    fn disabled(&self, label: &str) -> bool {
        self.harness
            .get_by_label(label)
            .accesskit_node()
            .is_disabled()
    }

    fn click(&mut self, label: &str) {
        self.harness.get_by_label(label).click();
        self.harness.run();
    }

    /// A button that asks first: the press, then Confirm.
    fn click_confirmed(&mut self, label: &str) {
        self.click(label);
        self.click("Confirm");
    }

    /// Draws until the window shows `text` (a whole label), for at most `seconds`; the buttons are checked on
    /// the way: while a progress line is shown, none of the three is enabled.
    fn wait_for(&mut self, text: &str, seconds: f64) {
        let deadline = Instant::now() + Duration::from_secs_f64(seconds);
        loop {
            self.harness.run();
            if self.shows(text) {
                return;
            }
            if ["Uploading ", "Reading ", "Clearing ", "Request sent"]
                .iter()
                .any(|lead| self.shows_containing(lead))
            {
                for button in [UPLOAD, READ, CLEAR] {
                    assert!(self.disabled(button), "{button} while an operation runs");
                }
            }
            assert!(
                Instant::now() < deadline,
                "never showed {text:?}; the state is {:?}",
                self.last_line()
            );
            std::thread::sleep(POLL);
        }
    }

    fn last_line(&self) -> String {
        [
            "Upload ",
            "Read ",
            "Clear ",
            "Reading",
            "Uploading",
            "Request",
        ]
        .iter()
        .flat_map(|lead| self.harness.query_all_by_label_contains(lead))
        .filter_map(|node| node.accesskit_node().label())
        .collect::<Vec<_>>()
        .join(" | ")
    }

    fn file(&self, name: &str) -> String {
        std::fs::read_to_string(self.store.dir().join(format!("{name}.waypoints")))
            .expect("the list's file")
    }

    fn sent(&self) -> Vec<String> {
        self.forwarded.borrow().clone()
    }
}

fn cleanup(store: &Store) {
    let _ = std::fs::remove_dir_all(store.dir());
}

fn store_with(tag: &str, name: &str, list: &[Waypoint]) -> Store {
    let store = temp_store(tag);
    store.save(name, list).expect("save the list");
    store
}

#[test]
fn upload_read_edit_and_clear_of_the_contracts_example() {
    let rig = Rig::start(SimConfig::default());
    let store = store_with("flow", "mission", &example());
    let mut window = Window::open(&rig.session, &store);
    window.harness.get_by_label_contains("3 points, ");
    for button in [UPLOAD, READ, CLEAR] {
        assert!(!window.disabled(button), "{button} with a vehicle");
    }
    assert!(
        !window.shows_containing("verified"),
        "nothing before a send"
    );
    assert!(!window.shows(LEGEND), "no vehicle list yet");

    // Upload: verified, with the count and the plan id of the contract's example.
    window.click_confirmed(UPLOAD);
    window.wait_for(VERIFIED_3, 20.0);
    assert!(window.sent()[0].starts_with("UploadMission"));
    assert_eq!(window.sent().len(), 1);
    assert_eq!(rig.session.state().missions.status, MissionStatus::Verified);
    window.harness.get_by_label(LEGEND);
    window.harness.get_by_label(MATCHES);
    for button in [UPLOAD, READ, CLEAR] {
        assert!(!window.disabled(button), "{button} when it is over");
    }

    // Read: the same list, still matching; the edited list and its file are what they were.
    let file = window.file("mission");
    window.click(READ);
    window.wait_for(
        "Read complete: the vehicle holds 3 waypoints, plan id 0x99C3BD49",
        20.0,
    );
    window.harness.get_by_label(MATCHES);
    assert_eq!(window.file("mission"), file, "Read does not touch the file");
    assert_eq!(store.load("mission").unwrap(), example());
    window.harness.get_by_label_contains("3 points, ");

    // Edit one point (the altitude of the second): it differs, and the file follows the edit only.
    window.harness.get_by_label("altitude of point 2").click();
    window.harness.run();
    window.harness.event(egui::Event::Text("20".to_string()));
    window.harness.run();
    window.harness.key_press(egui::Key::Enter);
    window.harness.run();
    let edited = store.load("mission").unwrap();
    assert_eq!(edited.len(), 3);
    assert_eq!(edited[1].alt_m, 20.0, "{edited:?}");
    assert_eq!((edited[0], edited[2]), (example()[0], example()[2]));
    window.harness.get_by_label(
        "The vehicle's list differs from the edited list: waypoint 2 is not the same",
    );
    assert!(!window.shows(MATCHES));

    // Read again: it changes neither the edited list nor its file, and it still differs.
    let file = window.file("mission");
    window.click(READ);
    window.wait_for(
        "Read complete: the vehicle holds 3 waypoints, plan id 0x99C3BD49",
        20.0,
    );
    assert_eq!(window.file("mission"), file);
    assert_eq!(store.load("mission").unwrap(), edited);
    window.harness.get_by_label(
        "The vehicle's list differs from the edited list: waypoint 2 is not the same",
    );

    // Clear: verified empty; the edited list stays.
    window.click_confirmed(CLEAR);
    window.wait_for(
        "Clear verified: the vehicle holds 0 waypoints, plan id 0x00000000",
        20.0,
    );
    assert_eq!(window.file("mission"), file);
    window.harness.get_by_label(
        "The vehicle's list differs from the edited list: the vehicle holds 0 waypoints, the edited list has 3 waypoints",
    );
    assert!(window.disabled(COPY), "nothing to copy from an empty list");
    drop(window);
    cleanup(&store);
}

#[test]
fn the_vehicles_list_becomes_a_list_of_its_own_only_when_asked() {
    let rig = Rig::start(SimConfig::default());
    let store = store_with("copy", "mission", &example());
    let mut window = Window::open(&rig.session, &store);
    window.click_confirmed(UPLOAD);
    window.wait_for(VERIFIED_3, 20.0);
    let original = window.file("mission");
    window.click(READ);
    window.wait_for(
        "Read complete: the vehicle holds 3 waypoints, plan id 0x99C3BD49",
        20.0,
    );
    assert_eq!(store.names().unwrap(), ["mission"], "Read made no list");

    window.click(COPY);
    assert_eq!(store.names().unwrap(), ["from vehicle", "mission"]);
    assert_eq!(store.load("from vehicle").unwrap(), example());
    assert_eq!(window.file("mission"), original, "the edited list's file");
    window
        .harness
        .get_by_label_contains("copied the vehicle's list to \"from vehicle\"");
    window.harness.get_by_label(MATCHES);
    drop(window);
    cleanup(&store);
}

#[test]
fn seventeen_points_are_sent_and_the_vehicle_refuses_with_no_space() {
    let rig = Rig::start(SimConfig::default());
    // The vehicle holds the example first.
    let first = store_with("seventeen-a", "mission", &example());
    let mut window = Window::open(&rig.session, &first);
    window.click_confirmed(UPLOAD);
    window.wait_for(VERIFIED_3, 20.0);
    drop(window);

    let seventeen: Vec<Waypoint> = (0..17)
        .map(|i| Waypoint {
            lat: 10.0 + f64::from(i) * 0.001,
            lon: -30.0,
            alt_m: 10.0,
        })
        .collect();
    let second = store_with("seventeen-b", "mission", &seventeen);
    let mut window = Window::open(&rig.session, &second);
    window.harness.get_by_label(
        "More than 16 points: the vehicle holds at most 16 and will refuse this list.",
    );
    window.click_confirmed(UPLOAD);
    window.wait_for(
        "Upload failed: refused by the vehicle, code 4 (MAV_MISSION_NO_SPACE): the vehicle holds at most 16 waypoints; the vehicle holds 3 waypoints, plan id 0x99C3BD49",
        20.0,
    );
    assert!(window.sent()[0].starts_with("UploadMission"));
    assert!(!window.shows_containing("verified"));
    window.harness.get_by_label(
        "The vehicle's list differs from the edited list: the vehicle holds 3 waypoints, the edited list has 17 waypoints",
    );
    // The vehicle's list is the example still.
    window.click(READ);
    window.wait_for(
        "Read complete: the vehicle holds 3 waypoints, plan id 0x99C3BD49",
        20.0,
    );
    drop(window);
    cleanup(&first);
    cleanup(&second);
}

#[test]
fn an_armed_vehicle_refuses_and_the_window_says_so() {
    let rig = Rig::start(SimConfig::default());
    rig.arm();
    let store = store_with("armed", "mission", &example());
    let mut window = Window::open(&rig.session, &store);
    window.click_confirmed(UPLOAD);
    window.wait_for(
        "Upload failed: refused by the vehicle, code 1 (MAV_MISSION_ERROR): the vehicle is armed, or no transfer was running; the vehicle holds 0 waypoints, plan id 0x00000000",
        20.0,
    );
    assert!(!window.shows_containing("verified"));
    window.click_confirmed(CLEAR);
    window.wait_for(
        "Clear failed: refused by the vehicle, code 1 (MAV_MISSION_ERROR): the vehicle is armed, or no transfer was running; the vehicle holds 0 waypoints, plan id 0x00000000",
        20.0,
    );
    // A read is allowed while armed.
    window.click(READ);
    window.wait_for(
        "Read complete: the vehicle holds 0 waypoints, plan id 0x00000000",
        20.0,
    );
    drop(window);
    cleanup(&store);
}

fn upload_on_a_lossy_link(config: SimConfig, tag: &str) {
    let rig = Rig::start(config);
    let store = store_with(tag, "mission", &example());
    let mut window = Window::open(&rig.session, &store);
    let started = Instant::now();
    window.click_confirmed(UPLOAD);
    // Every lost frame costs 1.5 s: the window shows the progress meanwhile, with the buttons disabled.
    window.wait_for(VERIFIED_3, 120.0);
    assert!(
        started.elapsed() >= Duration::from_millis(1400),
        "no frame was lost: {:?}",
        started.elapsed()
    );
    window.harness.get_by_label(MATCHES);
    assert_eq!(rig.session.state().missions.status, MissionStatus::Verified);
    drop(window);
    cleanup(&store);
}

#[test]
fn with_every_third_frame_to_the_vehicle_lost_the_upload_still_ends_verified() {
    upload_on_a_lossy_link(
        SimConfig {
            drop_mission_rx_every: 3,
            ..SimConfig::default()
        },
        "lossy-rx",
    );
}

#[test]
fn with_every_third_frame_from_the_vehicle_lost_the_upload_still_ends_verified() {
    upload_on_a_lossy_link(
        SimConfig {
            drop_mission_tx_every: 3,
            ..SimConfig::default()
        },
        "lossy-tx",
    );
}

#[test]
fn the_buttons_are_disabled_while_an_operation_runs_and_the_progress_is_shown() {
    let rig = Rig::start(SimConfig::default());
    rig.vehicle
        .lock()
        .unwrap_or_else(PoisonError::into_inner)
        .set_silent(true);
    let store = store_with("busy", "mission", &example());
    let mut window = Window::open(&rig.session, &store);
    window.click_confirmed(UPLOAD);
    // The vehicle says nothing: the upload waits, and the window says what it waits for.
    window.wait_for("Uploading 0 of 3 waypoints", 5.0);
    for button in [UPLOAD, READ, CLEAR] {
        assert!(window.disabled(button), "{button}");
    }
    assert!(!window.shows_containing("verified"));
    // Another press is not possible: nothing more was sent.
    assert_eq!(window.sent().len(), 1);
    drop(window);
    cleanup(&store);
}
