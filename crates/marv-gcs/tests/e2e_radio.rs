//! End to end: the Radio tab's calibration against the simulated vehicle over a real TCP socket, in real
//! time (docs/link-contract.md section 12). The sticks are fed by the test (the simulator's own
//! RC_CHANNELS are a sine wave); everything else is the real session and the real simulator.

use std::cell::{Cell, RefCell};
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
use gcs_mavlink::services::messages::FieldValue;
use gcs_mavlink::services::params::FetchStatus;
use gcs_mavlink::{Access, Command, LinkStatus, Session, SessionConfig, VehicleState};
use gcs_sim::rcmap::{angle, manual_control, throttle};
use gcs_sim::{ParamValue, SimConfig, SimVehicle};
use gcs_transport::Endpoint;
use marv_gcs::views::{self, ViewCtx};

const POLL: Duration = Duration::from_millis(20);

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

    /// Polls until `condition` holds, for at most `seconds`.
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

/// The Radio view drawn over the real session's state, with the sticks the test sets in place of the
/// RC_CHANNELS the simulator sent. Its commands go to the session.
struct Window {
    harness: Harness<'static>,
    sticks: Rc<Cell<[u16; 16]>>,
    forwarded: Rc<RefCell<Vec<String>>>,
}

impl Window {
    fn open(session: Rc<Session>) -> Window {
        let sticks = Rc::new(Cell::new([1500u16; 16]));
        let forwarded: Rc<RefCell<Vec<String>>> = Rc::default();
        let started = Instant::now();
        let mut view = views::all()
            .into_iter()
            .find(|v| v.name() == "Radio")
            .expect("Radio view is registered");
        let (s, f) = (sticks.clone(), forwarded.clone());
        let mut harness = Harness::builder()
            .with_size(egui::vec2(1000.0, 1800.0))
            .build_ui(move |ui| {
                let now = started.elapsed();
                let mut state = session.state().clone();
                let entry = state
                    .messages
                    .entries
                    .values_mut()
                    .find(|e| e.name == "RC_CHANNELS")
                    .expect("the simulator sends RC_CHANNELS");
                entry.last_seen = now;
                for (i, v) in s.get().iter().enumerate() {
                    let name = format!("chan{}_raw", i + 1);
                    if let Some(field) = entry.fields.iter_mut().find(|f| f.name == name) {
                        field.value = FieldValue::Number(f64::from(*v));
                    }
                }
                let mut commands: Vec<Command> = Vec::new();
                let mut cx = ViewCtx {
                    state: &state,
                    commands: &mut commands,
                    now: Some(now),
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
            sticks,
            forwarded,
        }
    }

    fn feed(&mut self, sticks: [u16; 16]) {
        self.sticks.set(sticks);
        self.harness.run();
    }

    fn click(&mut self, label: &str) {
        self.harness.get_by_label(label).click();
        self.harness.run();
    }

    fn shows(&self, text: &str) -> bool {
        self.harness.query_by_label_contains(text).is_some()
    }
}

#[test]
fn apply_writes_twelve_values_the_vehicle_follows_them_and_save_to_flash_is_one_command() {
    let rig = Rig::start();
    let mut window = Window::open(rig.session.clone());
    let mut sweep = [1500u16; 16];

    window.click("Calibrate radio");
    for end in [988, 2012] {
        sweep[..4].fill(end);
        sweep[8..].fill(end);
        window.feed(sweep);
    }
    window.feed([1500; 16]);
    window.click("Done");
    let mut centre = [1500u16; 16];
    centre[..4].fill(1520);
    window.feed(centre);
    window.click("OK");
    assert!(window.forwarded.borrow().is_empty());
    window.click("Apply");

    // Twelve values in the simulator, and nothing else was set.
    let at_least = |n: &str, v: i32| rig.param(n) == Some(ParamValue::Int32(v));
    assert!(
        rig.wait(5.0, |s| s.params.writes.is_empty()
            && (1..=4).all(|n| at_least(&format!("RC{n}_MIN"), 988)
                && at_least(&format!("RC{n}_TRIM"), 1520)
                && at_least(&format!("RC{n}_MAX"), 2012)))
            .is_some(),
        "the simulator does not hold the written values"
    );
    for n in 5..=8 {
        assert!(at_least(&format!("RC{n}_MIN"), 1100), "RC{n}_MIN");
        assert!(at_least(&format!("RC{n}_TRIM"), 1500), "RC{n}_TRIM");
        assert!(at_least(&format!("RC{n}_MAX"), 1900), "RC{n}_MAX");
    }
    let forwarded = window.forwarded.borrow().clone();
    assert_eq!(forwarded.len(), 12, "{forwarded:?}");
    assert!(forwarded.iter().all(|c| c.starts_with("SetParam")));
    assert_eq!(
        rig.vehicle
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .param_sets_received(),
        12
    );
    assert!(window.shows("Applied in RAM; not saved to flash"));

    // The vehicle's MANUAL_CONTROL follows the contract's formulas for the new values. The formulas
    // on a few values of the written parameters first (MIN 988, TRIM 1520, MAX 2012).
    assert_eq!(angle(2012, 988, 1520, 2012, false, 0), 1000);
    assert_eq!(angle(988, 988, 1520, 2012, false, 0), -1000);
    assert_eq!(angle(1520, 988, 1520, 2012, false, 0), 0);
    assert_eq!(angle(1766, 988, 1520, 2012, false, 0), 500);
    assert_eq!(throttle(1500, 988, 2012, false, 0), 500);
    // Then the stream: a RC_CHANNELS and the MANUAL_CONTROL of the same instant (the same count, 5 Hz
    // both), read after the new values were in effect.
    let count = |s: &VehicleState, name: &str| s.messages.entry(name, 0).map(|e| e.count);
    let baseline = count(&rig.session.state(), "MANUAL_CONTROL").expect("MANUAL_CONTROL arrives");
    let mut checked = false;
    let deadline = Instant::now() + Duration::from_secs(6);
    while Instant::now() < deadline && !checked {
        std::thread::sleep(POLL);
        let state = rig.session.state();
        let (Some(rc), Some(mc)) = (
            state.messages.entry("RC_CHANNELS", 0),
            state.messages.entry("MANUAL_CONTROL", 0),
        ) else {
            continue;
        };
        if mc.count <= baseline + 1 || rc.count != mc.count {
            continue;
        }
        let raw: Vec<u16> = (1..=16)
            .map(|i| rc.number(&format!("chan{i}_raw")).expect("channel") as u16)
            .collect();
        let channels: [u16; 16] = raw.try_into().expect("16 channels");
        let want = manual_control(1, &channels, |name| rig.param(name)).expect("RC parameters");
        let got = |name: &str| mc.number(name).expect("field") as i16;
        assert_eq!(
            (got("x"), got("y"), got("z"), got("r")),
            (want.x, want.y, want.z, want.r),
            "channels {channels:?}"
        );
        checked = true;
    }
    assert!(checked, "no matching RC_CHANNELS and MANUAL_CONTROL pair");

    // Save to flash: one command 245 with param1 = 1; the line goes only after the vehicle accepts.
    window.click("Save to flash");
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
    assert!(window.shows("Applied in RAM; not saved to flash"));
    let mut shown_while_waiting = false;
    let deadline = Instant::now() + Duration::from_secs(8);
    loop {
        window.harness.run();
        let record = rig
            .session
            .state()
            .command
            .clone()
            .expect("a command record");
        assert_eq!(record.command, 245);
        let line = window.shows("Applied in RAM; not saved to flash");
        if line {
            shown_while_waiting |= record.outstanding();
        } else {
            assert_eq!(
                record.status,
                gcs_mavlink::CommandStatus::Finished {
                    text: "accepted",
                    accepted: true
                },
                "the line went before the vehicle accepted"
            );
            break;
        }
        assert!(
            Instant::now() < deadline,
            "the save never finished: {record:?}"
        );
        std::thread::sleep(POLL);
    }
    assert!(
        shown_while_waiting,
        "the line was not shown while the save ran"
    );
    assert_eq!(
        window
            .forwarded
            .borrow()
            .iter()
            .filter(|c| c.starts_with("SendCommand"))
            .count(),
        1
    );
}

#[test]
fn applying_rcmap_roll_2_makes_the_simulator_report_channel_2_as_roll() {
    let rig = Rig::start();
    let mut window = Window::open(rig.session.clone());

    // Roll is the first drop-down; pick channel 2 and apply.
    window
        .harness
        .get_all_by_role(Role::ComboBox)
        .next()
        .expect("the Roll drop-down")
        .click();
    window.harness.run();
    window
        .harness
        .get_all_by_role_and_label(Role::Button, "2")
        .next()
        .expect("the list of channels")
        .click();
    window.harness.run();
    assert!(
        window.forwarded.borrow().is_empty(),
        "staging sends nothing"
    );
    window.click("Apply mapping");
    assert_eq!(window.forwarded.borrow().len(), 1);
    assert!(window.forwarded.borrow()[0].contains("RCMAP_ROLL"));

    // The simulator holds it, and the write is confirmed.
    assert!(
        rig.wait(5.0, |s| s.params.writes.is_empty()
            && rig.param("RCMAP_ROLL") == Some(ParamValue::Int32(2)))
            .is_some(),
        "the simulator does not hold RCMAP_ROLL = 2"
    );
    assert_eq!(
        rig.vehicle
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .param_sets_received(),
        1
    );
    window.harness.run();
    assert!(window.shows("Applied in RAM; not saved to flash"));

    // The tag moved to channel 2 (rows 1 and 2 are the first two with the default MIN 1100).
    let rows: Vec<String> = window
        .harness
        .get_all_by_label_contains("MIN 1100")
        .map(|n| n.value().unwrap_or_default())
        .collect();
    assert_eq!(rows[0], "MIN 1100 TRIM 1500 MAX 1900", "{rows:?}");
    assert_eq!(
        rows[1], "MIN 1100 TRIM 1500 MAX 1900  [Roll, Pitch]",
        "{rows:?}"
    );

    // MANUAL_CONTROL `y` is the angle of channel 2 for the contract's formula (defaults: MIN 1100, TRIM 1500,
    // MAX 1900), taken from a RC_CHANNELS and the MANUAL_CONTROL of the same instant after the write.
    let count = |s: &VehicleState, name: &str| s.messages.entry(name, 0).map(|e| e.count);
    let baseline = count(&rig.session.state(), "MANUAL_CONTROL").expect("MANUAL_CONTROL arrives");
    let mut checked = false;
    let deadline = Instant::now() + Duration::from_secs(6);
    while Instant::now() < deadline && !checked {
        std::thread::sleep(POLL);
        let state = rig.session.state();
        let (Some(rc), Some(mc)) = (
            state.messages.entry("RC_CHANNELS", 0),
            state.messages.entry("MANUAL_CONTROL", 0),
        ) else {
            continue;
        };
        if mc.count <= baseline + 1 || rc.count != mc.count {
            continue;
        }
        let channel2 = rc.number("chan2_raw").expect("channel 2") as u16;
        let want = i32::from(angle(channel2, 1100, 1500, 1900, false, 0));
        assert_eq!(
            mc.number("y").expect("y") as i32,
            want,
            "channel 2 = {channel2}"
        );
        checked = true;
    }
    assert!(checked, "no matching RC_CHANNELS and MANUAL_CONTROL pair");
}
