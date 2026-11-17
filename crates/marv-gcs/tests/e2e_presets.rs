//! End to end: the Devices tab's BARO and IMU preset cards against the simulated vehicle over a real
//! TCP socket, in real time (docs/link-contract.md section 13). A preset is the station writing the
//! fields one by one; the card judges the echoes against the table (`gcs_mavlink::presets`), and this
//! file compares what the simulator holds with the same table.

use std::cell::RefCell;
use std::net::TcpListener;
use std::rc::Rc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, PoisonError};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use eframe::egui;
use eframe::egui::accesskit::Role;
use egui_kittest::Harness;
use egui_kittest::kittest::Queryable;
use gcs_mavlink::presets::{BARO, IMU1, SENSORS, Value, preset};
use gcs_mavlink::services::params::FetchStatus;
use gcs_mavlink::{Access, Command, LinkStatus, Session, SessionConfig, VehicleState};
use gcs_sim::{ParamValue, SimConfig, SimVehicle};
use gcs_transport::Endpoint;
use marv_gcs::views::presets::{PENDING_TEXT, SAVE_AND_REBOOT};
use marv_gcs::views::{self, ViewCtx};

const POLL: Duration = Duration::from_millis(20);
const NOT_SAVED: &str = "Applied in RAM; not saved to flash";

struct Rig {
    vehicle: Arc<Mutex<SimVehicle>>,
    stop: Arc<AtomicBool>,
    server: Option<JoinHandle<()>>,
    session: Rc<Session>,
}

impl Rig {
    fn start() -> Rig {
        let listener = TcpListener::bind("127.0.0.1:0").expect("bind");
        let addr = listener.local_addr().expect("address").to_string();
        let vehicle = Arc::new(Mutex::new(SimVehicle::new(SimConfig::default())));
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
        assert!(
            rig.wait(10.0, |s| s.link.status == LinkStatus::Connected
                && s.params.fetch == FetchStatus::Complete)
                .is_some(),
            "never connected with the whole list"
        );
        rig
    }

    fn wait(&self, seconds: f64, mut condition: impl FnMut(&VehicleState) -> bool) -> Option<()> {
        let deadline = Instant::now() + Duration::from_secs_f64(seconds);
        while Instant::now() < deadline {
            if condition(&self.session.state()) {
                return Some(());
            }
            std::thread::sleep(POLL);
        }
        None
    }

    fn param(&self, name: &str) -> Option<ParamValue> {
        self.vehicle
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .param(name)
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

/// The Devices view drawn over the real session's state; its commands go to the session.
struct Window {
    harness: Harness<'static>,
    forwarded: Rc<RefCell<Vec<String>>>,
}

impl Window {
    fn open(session: Rc<Session>) -> Window {
        let forwarded: Rc<RefCell<Vec<String>>> = Rc::default();
        let started = Instant::now();
        let mut view = views::all()
            .into_iter()
            .find(|v| v.name() == "Devices")
            .expect("Devices view is registered");
        let f = forwarded.clone();
        let mut harness = Harness::builder()
            .with_size(egui::vec2(1500.0, 1800.0))
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
        Window { harness, forwarded }
    }

    fn shows(&self, text: &str) -> bool {
        self.harness.query_by_label_contains(text).is_some()
    }

    fn click_nth(&mut self, label: &str, n: usize) {
        self.harness
            .get_all_by_label(label)
            .nth(n)
            .expect("the button")
            .click();
        self.harness.run();
    }
}

#[test]
fn the_simulators_defaults_are_the_default_preset_of_every_sensor() {
    let rig = Rig::start();
    for sensor in SENSORS {
        let default = preset(sensor, sensor.default_id).expect("the default preset");
        for (field, want) in sensor.fields.iter().zip(default.values) {
            assert_eq!(stored(&rig, field), *want, "{field}");
        }
    }
    for selector in ["IMU1_PRESET", "IMU2_PRESET", "BARO_PRESET", "MAG_PRESET"] {
        assert_eq!(rig.param(selector), None, "{selector}");
    }
}

fn stored(rig: &Rig, field: &str) -> Value {
    match rig.param(field).unwrap_or_else(|| panic!("{field}")) {
        ParamValue::Int32(v) => Value::I32(v),
        ParamValue::Real32(v) => Value::F32(v),
        other => panic!("{field}: {other:?}"),
    }
}

#[test]
fn baro_smooth_is_applied_by_three_writes_and_saved_only_when_asked() {
    let rig = Rig::start();
    let mut window = Window::open(rig.session.clone());
    assert_eq!(rig.param("BARO_OSR_P"), Some(ParamValue::Int32(16)));

    window
        .harness
        .get_all_by_role(Role::ComboBox)
        .nth(2)
        .expect("the BARO drop-down")
        .click();
    window.harness.run();
    window
        .harness
        .get_all_by_role_and_label(Role::Button, "Smooth")
        .next()
        .expect("Smooth")
        .click();
    window.harness.run();
    assert!(
        window.forwarded.borrow().is_empty(),
        "choosing sends nothing"
    );
    assert_eq!(rig.param("BARO_OSR_P"), Some(ParamValue::Int32(16)));

    window.click_nth("Apply", 2);
    // The three changed fields left the window, the one that goes down first.
    let forwarded = window.forwarded.borrow().clone();
    assert_eq!(forwarded.len(), 3, "{forwarded:?}");
    for (sent, field) in forwarded
        .iter()
        .zip(["BARO_ODR_HZ", "BARO_OSR_P", "BARO_IIR_P"])
    {
        assert!(sent.starts_with("SetParam") && sent.contains(field), "{sent}");
    }

    let deadline = Instant::now() + Duration::from_secs(8);
    while !window.shows(NOT_SAVED) {
        assert!(
            Instant::now() < deadline,
            "the card never showed the preset applied"
        );
        std::thread::sleep(POLL);
        window.harness.run();
    }
    let chosen = preset(&BARO, 3).expect("preset 3");
    for (field, want) in BARO.fields.iter().zip(chosen.values) {
        assert_eq!(stored(&rig, field), *want, "{field}");
    }
    assert!(
        window
            .forwarded
            .borrow()
            .iter()
            .all(|c| !c.contains("PREFLIGHT_STORAGE")),
        "no save before the button"
    );

    window.click_nth("Save to flash", 0);
    let saves: Vec<String> = window
        .forwarded
        .borrow()
        .iter()
        .filter(|c| c.starts_with("SendCommand"))
        .cloned()
        .collect();
    assert_eq!(saves.len(), 1, "{saves:?}");
    assert!(saves[0].contains("MAV_CMD_PREFLIGHT_STORAGE"));
    assert!(saves[0].contains("[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"));
    let deadline = Instant::now() + Duration::from_secs(8);
    while window.shows(NOT_SAVED) {
        assert!(Instant::now() < deadline, "the save was never accepted");
        std::thread::sleep(POLL);
        window.harness.run();
    }
}

#[test]
fn imu1_smooth_is_pending_until_save_and_reboot_which_sends_245_then_246() {
    let rig = Rig::start();
    let mut window = Window::open(rig.session.clone());
    assert_eq!(rig.param("IMU1_ODR_HZ"), Some(ParamValue::Int32(800)));

    window
        .harness
        .get_all_by_role(Role::ComboBox)
        .next()
        .expect("the IMU1 drop-down")
        .click();
    window.harness.run();
    window
        .harness
        .get_all_by_role_and_label(Role::Button, "Smooth")
        .next()
        .expect("Smooth")
        .click();
    window.harness.run();
    window.click_nth("Apply", 0);
    let forwarded = window.forwarded.borrow().clone();
    assert_eq!(forwarded.len(), 3, "the three fields: {forwarded:?}");
    for (sent, field) in forwarded
        .iter()
        .zip(["IMU1_ACC_FS", "IMU1_GYR_FS", "IMU1_ODR_HZ"])
    {
        assert!(sent.starts_with("SetParam") && sent.contains(field), "{sent}");
    }

    let deadline = Instant::now() + Duration::from_secs(8);
    while !window.shows(PENDING_TEXT) {
        assert!(
            Instant::now() < deadline,
            "the card never showed the preset pending"
        );
        std::thread::sleep(POLL);
        window.harness.run();
    }
    let chosen = preset(&IMU1, 4).expect("preset 4");
    for (field, want) in IMU1.fields.iter().zip(chosen.values) {
        assert_eq!(stored(&rig, field), *want, "{field}");
    }
    assert!(
        rig.wait(2.0, |s| s
            .texts
            .entries
            .iter()
            .any(|t| t.text == "IMU1_ODR_HZ: applies after reboot"))
            .is_some(),
        "the vehicle's notice"
    );
    assert!(
        window
            .forwarded
            .borrow()
            .iter()
            .all(|c| !c.starts_with("SendCommand")),
        "no command before the button"
    );

    window.click_nth(SAVE_AND_REBOOT, 0);
    assert!(
        window
            .forwarded
            .borrow()
            .iter()
            .all(|c| !c.starts_with("SendCommand")),
        "the first click only arms"
    );
    window.click_nth(&format!("Confirm: {SAVE_AND_REBOOT}"), 0);

    // The card sends 246 in the frame in which it sees 245 accepted, so only the last record is
    // reliably seen: 246 finished and accepted. That 246 left at all proves the vehicle's ACCEPTED of 245.
    let mut finished: Vec<u32> = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(12);
    while !finished.contains(&246) {
        assert!(
            Instant::now() < deadline,
            "never saw 246 accepted: {finished:?}, {:?}",
            window.forwarded.borrow()
        );
        std::thread::sleep(POLL);
        window.harness.run();
        if let Some(r) = rig.session.state().command.clone()
            && matches!(
                r.status,
                gcs_mavlink::CommandStatus::Finished { accepted: true, .. }
            )
            && !finished.contains(&r.command)
        {
            finished.push(r.command);
        }
    }
    let commands: Vec<String> = window
        .forwarded
        .borrow()
        .iter()
        .filter(|c| c.starts_with("SendCommand"))
        .cloned()
        .collect();
    assert_eq!(commands.len(), 2, "{commands:?}");
    assert!(commands[0].contains("MAV_CMD_PREFLIGHT_STORAGE"));
    assert!(commands[1].contains("MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN"));
    assert!(commands[1].contains("[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"));
    // The simulator does not reboot: the stored values are as they were.
    assert_eq!(rig.param("IMU1_ODR_HZ"), Some(ParamValue::Int32(400)));
}
