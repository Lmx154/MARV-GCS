//! The Compass panel of the Devices tab against the simulated vehicle (decision `decision-compass-cal-20261120.md`
//! sections 2 to 5 and 8, invariant I5; link contract section 16).
//!
//! Time. The simulator is `gcs_sim`'s, served over a real TCP socket as the other end to end tests do, but by a copy of
//! `serve_tcp` whose clock runs [`SPEED`] times faster than the wall clock, so the 90 s of scripted turns of the good
//! user take about 10 s of wall time. The view is given the wall time plus an offset the test can move forward
//! (`Window::skip`): that is how a 90 s timeout is passed without waiting. The view's `update` and `ui` run on every
//! frame, as the application does.

use std::cell::{Cell, RefCell};
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::rc::Rc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, PoisonError};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use eframe::egui;
use egui_kittest::Harness;
use egui_kittest::kittest::{NodeT, Queryable};
use gcs_mavlink::services::messages::{Field, FieldValue, MessageEntry, MessageKey};
use gcs_mavlink::services::params::FetchStatus;
use gcs_mavlink::services::params::{Param, ParamEncoding, ParamType, ParamValue, encode};
use gcs_mavlink::{Access, Command, LinkStatus, Session, SessionConfig, VehicleState};
use gcs_sim::mag::{M3, MagEnvironment};
use gcs_sim::script::CalibrationUser;
use gcs_sim::{ParamValue as SimValue, SimConfig, SimVehicle};
use gcs_transport::Endpoint;
use marv_gcs::views::{self, ViewCtx};

/// How much faster than the wall clock the simulator runs.
const SPEED: u64 = 8;
const POLL: Duration = Duration::from_millis(10);
const SAVED_TEXT: &str = "Compass calibrated (guided): in use now, not saved to flash";
const GUIDED: &str = "Calibrate compass (guided)";
const PLAN: [&str; 13] = [
    "MAG_CAL_TYPE",
    "MAG_OFFX",
    "MAG_OFFY",
    "MAG_OFFZ",
    "MAG_DIAGX",
    "MAG_DIAGY",
    "MAG_DIAGZ",
    "MAG_ODIAGXY",
    "MAG_ODIAGXZ",
    "MAG_ODIAGYZ",
    "MAG_FLD_UT",
    "MAG_FLD_INC",
    "MAG_CAL_TYPE",
];
const S_TRUE: M3 = [
    [1.0625, 0.03125, -0.046875],
    [0.03125, 0.9375, 0.0625],
    [-0.046875, 0.0625, 0.78125],
];
const O_TRUE: [f64; 3] = [-14.2, -4.2, 1.5];

/// `gcs_sim::serve_tcp` with the simulator's clock [`SPEED`] times the wall clock.
fn serve_fast(listener: TcpListener, vehicle: Arc<Mutex<SimVehicle>>, stop: Arc<AtomicBool>) {
    listener.set_nonblocking(true).expect("non-blocking");
    let start = Instant::now();
    let mut client: Option<TcpStream> = None;
    let mut buf = [0u8; 1024];
    while !stop.load(Ordering::Relaxed) {
        let mut received: &[u8] = &[];
        let mut closed = false;
        if let Some(stream) = client.as_mut() {
            match stream.read(&mut buf) {
                Ok(0) => closed = true,
                Ok(n) => received = &buf[..n],
                Err(e)
                    if e.kind() == std::io::ErrorKind::WouldBlock
                        || e.kind() == std::io::ErrorKind::TimedOut
                        || e.kind() == std::io::ErrorKind::Interrupted => {}
                Err(_) => closed = true,
            }
        } else {
            match listener.accept() {
                Ok((stream, _)) => {
                    stream.set_nonblocking(false).expect("blocking");
                    stream.set_nodelay(true).expect("nodelay");
                    stream
                        .set_read_timeout(Some(Duration::from_millis(5)))
                        .expect("read timeout");
                    client = Some(stream);
                }
                Err(_) => std::thread::sleep(Duration::from_millis(5)),
            }
        }
        let now_ms = start.elapsed().as_millis() as u64 * SPEED;
        let out = vehicle
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .step(now_ms, received);
        if closed {
            client = None;
        } else if let Some(stream) = client.as_mut()
            && !out.is_empty()
            && stream.write_all(&out).is_err()
        {
            client = None;
        }
    }
}

struct Rig {
    vehicle: Arc<Mutex<SimVehicle>>,
    stop: Arc<AtomicBool>,
    server: Option<JoinHandle<()>>,
    session: Rc<Session>,
}

impl Rig {
    fn start(user: CalibrationUser, mag: impl FnOnce(&mut MagEnvironment), refuse: &[&str]) -> Rig {
        let mut env = MagEnvironment {
            s_true: S_TRUE,
            o_true: O_TRUE,
            noise_ut: 0.3,
            ..MagEnvironment::default()
        };
        mag(&mut env);
        let config = SimConfig {
            mag: env,
            user: Some(user),
            refuse_params: refuse.iter().map(|s| (*s).to_string()).collect(),
            ..SimConfig::default()
        };
        let listener = TcpListener::bind("127.0.0.1:0").expect("bind");
        let addr = listener.local_addr().expect("address").to_string();
        let vehicle = Arc::new(Mutex::new(SimVehicle::new(config)));
        let stop = Arc::new(AtomicBool::new(false));
        let server = {
            let (vehicle, stop) = (Arc::clone(&vehicle), Arc::clone(&stop));
            std::thread::spawn(move || serve_fast(listener, vehicle, stop))
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
        loop {
            let state = rig.session.state();
            if state.link.status == LinkStatus::Connected
                && state.params.fetch == FetchStatus::Complete
            {
                break;
            }
            drop(state);
            assert!(Instant::now() < deadline, "never connected with the list");
            std::thread::sleep(POLL);
        }
        rig
    }

    /// Every parameter of the simulator, by name, as text.
    fn params(&self) -> Vec<(String, String)> {
        let v = self.vehicle.lock().unwrap_or_else(PoisonError::into_inner);
        gcs_sim::default_params()
            .iter()
            .map(|p| {
                (
                    p.name.clone(),
                    format!("{:?}", v.param(&p.name).expect("a param")),
                )
            })
            .collect()
    }

    fn param(&self, name: &str) -> Option<SimValue> {
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

/// The Devices view drawn over a state; `update` and `ui` on every frame.
struct Window {
    harness: Harness<'static>,
    forwarded: Rc<RefCell<Vec<String>>>,
    offset: Rc<Cell<Duration>>,
}

impl Window {
    fn over_state(state: VehicleState, access: Access) -> Window {
        Window::build(Box::new(move || state.clone()), access, None)
    }

    fn build(
        get: Box<dyn Fn() -> VehicleState>,
        access: Access,
        session: Option<Rc<Session>>,
    ) -> Window {
        let forwarded: Rc<RefCell<Vec<String>>> = Rc::default();
        let offset: Rc<Cell<Duration>> = Rc::default();
        let started = Instant::now();
        let mut view = views::all()
            .into_iter()
            .find(|v| v.name() == "Devices")
            .expect("Devices view is registered");
        let (f, o) = (forwarded.clone(), offset.clone());
        let mut harness = Harness::builder()
            .with_size(egui::vec2(1500.0, 2200.0))
            .build_ui(move |ui| {
                let state = get();
                let now = Some(started.elapsed() + o.get());
                view.update(&state, now);
                let mut commands: Vec<Command> = Vec::new();
                let mut cx = ViewCtx {
                    state: &state,
                    commands: &mut commands,
                    now,
                    access,
                };
                view.ui(ui, &mut cx);
                for command in commands {
                    f.borrow_mut().push(format!("{command:?}"));
                    if let Some(s) = &session {
                        s.send(command);
                    }
                }
            });
        harness.run();
        Window {
            harness,
            forwarded,
            offset,
        }
    }

    /// With a session the commands reach it.
    fn with_commands(session: Rc<Session>) -> Window {
        let s = session.clone();
        Window::build(
            Box::new(move || s.state().clone()),
            Access::Configure,
            Some(session),
        )
    }

    fn shows(&self, text: &str) -> bool {
        self.harness
            .query_all_by_label_contains(text)
            .next()
            .is_some()
    }

    fn click(&mut self, label: &str) {
        self.harness
            .get_all_by_label(label)
            .next()
            .unwrap_or_else(|| panic!("no button {label}"))
            .click();
        self.harness.run();
    }

    fn pump_until(&mut self, seconds: f64, what: &str, mut done: impl FnMut(&Window) -> bool) {
        let deadline = Instant::now() + Duration::from_secs_f64(seconds);
        while !done(self) {
            if Instant::now() >= deadline {
                let dump = format!("{:#?}", self.harness.root());
                let lines: Vec<&str> = dump
                    .lines()
                    .filter(|l| l.contains("label") || l.contains("value"))
                    .collect();
                panic!("never showed: {what}\n{}", lines.join("\n"));
            }
            std::thread::sleep(POLL);
            self.harness.run();
        }
    }

    /// Moves the view's clock forward, as if `seconds` had passed.
    fn skip(&mut self, seconds: f64) {
        self.offset
            .set(self.offset.get() + Duration::from_secs_f64(seconds));
        self.harness.run();
    }

    fn set_params(&self) -> Vec<String> {
        self.forwarded
            .borrow()
            .iter()
            .filter(|c| c.starts_with("SetParam"))
            .cloned()
            .collect()
    }

    fn disabled(&self, label: &str) -> bool {
        self.harness
            .get_all_by_label(label)
            .next()
            .unwrap_or_else(|| panic!("no button {label}"))
            .accesskit_node()
            .is_disabled()
    }
}

fn user() -> CalibrationUser {
    CalibrationUser::new(20_000)
}

fn good_rig() -> (Rig, Window) {
    let rig = Rig::start(user(), |_| {}, &[]);
    let window = Window::with_commands(rig.session.clone());
    (rig, window)
}

/// Presses the guided button and waits until a sample has been taken in the first pose.
fn start_and_wait_for_progress(window: &mut Window) {
    window.click(GUIDED);
    assert!(window.shows("samples kept"), "the run did not start");
    window.pump_until(30.0, "a pose being turned", |w| w.shows("turn it slowly"));
}

#[test]
fn the_good_user_ends_with_the_thirteen_writes_in_order_read_back_equal_and_save_offered() {
    let (rig, mut window) = good_rig();
    window.click(GUIDED);
    window.pump_until(60.0, SAVED_TEXT, |w| w.shows(SAVED_TEXT));

    let sent = window.set_params();
    assert_eq!(sent.len(), 13, "{sent:?}");
    for (sent, name) in sent.iter().zip(PLAN) {
        assert!(
            sent.contains(&format!("\"{name}\"")),
            "{sent} is not {name}"
        );
    }
    assert!(
        sent[0].contains("I32(0)") && sent[12].contains("I32(2)"),
        "{sent:?}"
    );
    // The write is followed by the read-back: the list was asked for again after the last write.
    let forwarded = window.forwarded.borrow().clone();
    let last_write = forwarded
        .iter()
        .rposition(|c| c.starts_with("SetParam"))
        .unwrap();
    assert!(
        forwarded[last_write + 1..]
            .iter()
            .any(|c| c.starts_with("RefreshParams")),
        "{forwarded:?}"
    );

    assert_eq!(rig.param("MAG_CAL_TYPE"), Some(SimValue::Int32(2)));
    let SimValue::Real32(off_x) = rig.param("MAG_OFFX").unwrap() else {
        panic!("MAG_OFFX is not a float")
    };
    assert!((f64::from(off_x) - O_TRUE[0]).abs() < 0.8, "{off_x}");
    assert!(window.shows("Fitness"), "the fitness is shown");
    assert!(
        window.shows("Inclination measured"),
        "the inclination is shown"
    );
    // Save to flash and Discard are offered, and nothing was saved by itself.
    assert!(
        window
            .harness
            .query_all_by_label("Save to flash")
            .next()
            .is_some()
    );
    assert!(
        window
            .harness
            .query_all_by_label("Discard")
            .next()
            .is_some()
    );
    assert!(forwarded.iter().all(|c| !c.contains("PREFLIGHT_STORAGE")));
    assert!(
        window.shows("Compass calibration: Guided"),
        "the state line"
    );
}

/// A run that ends without a result: its text is shown verbatim and not one parameter changed.
fn ends_with_nothing_written(rig: Rig, mut window: Window, text: &str, skip_after_script: bool) {
    let before = rig.params();
    window.click(GUIDED);
    if skip_after_script {
        // The scripted user has finished when its last pose is done; the rest is waiting.
        window.pump_until(60.0, "the last pose done", |w| {
            w.shows("right side down 12/12 done")
        });
        window.skip(100.0);
    }
    window.pump_until(60.0, text, |w| w.shows(text));
    assert_eq!(rig.params(), before, "no parameter may change");
    assert!(window.set_params().is_empty());
    assert!(
        window
            .harness
            .query_all_by_label("Save to flash")
            .next()
            .is_none()
    );
}

#[test]
fn a_skipped_pose_times_out_naming_it_and_writes_nothing() {
    let rig = Rig::start(user().skipping(3), |_| {}, &[]);
    let window = Window::with_commands(rig.session.clone());
    ends_with_nothing_written(
        rig,
        window,
        "Timed out waiting for nose up: start again",
        true,
    );
}

#[test]
fn a_magnet_nearby_is_refused_as_too_scattered_and_writes_nothing() {
    let rig = Rig::start(user(), |e| e.magnet_ut = 6.0, &[]);
    let window = Window::with_commands(rig.session.clone());
    ends_with_nothing_written(rig, window, "Readings too scattered", false);
}

#[test]
fn an_anomaly_site_is_refused_as_a_disturbed_field_and_writes_nothing() {
    let rig = Rig::start(user(), |e| e.inclination_shift_deg = 20.0, &[]);
    let window = Window::with_commands(rig.session.clone());
    ends_with_nothing_written(rig, window, "The field here is disturbed", false);
}

#[test]
fn arming_in_the_middle_stops_the_run_and_writes_nothing() {
    let (rig, mut window) = good_rig();
    let before = rig.params();
    start_and_wait_for_progress(&mut window);
    rig.vehicle
        .lock()
        .unwrap_or_else(PoisonError::into_inner)
        .set_armed(true);
    window.pump_until(10.0, "armed", |w| {
        w.shows("The vehicle was armed: calibration stopped")
    });
    assert_eq!(rig.params(), before);
    assert!(window.set_params().is_empty());
    // And the guided button is off while armed, with its reason.
    assert!(window.disabled(GUIDED));
    assert!(window.shows("the vehicle is armed"));
}

#[test]
fn link_loss_in_the_middle_times_out_while_the_ticks_run_and_writes_nothing() {
    let (rig, mut window) = good_rig();
    let before = rig.params();
    start_and_wait_for_progress(&mut window);
    rig.vehicle
        .lock()
        .unwrap_or_else(PoisonError::into_inner)
        .set_silent(true);
    window.skip(100.0);
    window.pump_until(10.0, "a timeout", |w| w.shows("Timed out waiting for"));
    assert_eq!(rig.params(), before);
    assert!(window.set_params().is_empty());
}

#[test]
fn a_write_refused_in_the_middle_of_the_commit_stops_it_with_type_0_and_no_save() {
    let rig = Rig::start(user(), |_| {}, &["MAG_FLD_UT"]);
    let mut window = Window::with_commands(rig.session.clone());
    window.click(GUIDED);
    window.pump_until(60.0, "a refusal", |w| w.shows("MAG_FLD_UT was refused"));

    assert!(window.shows("marked not calibrated (type 0)"));
    assert!(window.shows("Discard or a reboot restores"));
    let sent = window.set_params();
    assert_eq!(
        sent.len(),
        11,
        "the sequence stops at the refused write: {sent:?}"
    );
    assert!(sent[10].contains("MAG_FLD_UT"));
    assert_eq!(rig.param("MAG_CAL_TYPE"), Some(SimValue::Int32(0)));
    assert!(
        window
            .harness
            .query_all_by_label("Save to flash")
            .next()
            .is_none()
    );
    assert!(
        window
            .harness
            .query_all_by_label("Discard")
            .next()
            .is_some()
    );
    assert!(!window.shows(SAVED_TEXT));
}

fn hand_state(cal_type: i32, offset_x: f32, field: f32) -> VehicleState {
    let mut state = VehicleState::default();
    state.params.encoding_known = true;
    let mut put = |name: &str, value: ParamValue| {
        let kind: ParamType = value.kind();
        state.params.params.insert(
            name.to_string(),
            Param {
                name: name.to_string(),
                index: 0,
                kind,
                raw: encode(value, ParamEncoding::default()),
            },
        );
    };
    put("MAG_CAL_TYPE", ParamValue::I32(cal_type));
    put("MAG_OFFX", ParamValue::F32(offset_x));
    put("MAG_OFFY", ParamValue::F32(0.0));
    put("MAG_OFFZ", ParamValue::F32(0.0));
    put("MAG_FLD_UT", ParamValue::F32(field));
    put(
        "MAG_FLD_INC",
        ParamValue::F32(if field > 0.0 { 63.5 } else { 0.0 }),
    );
    state
}

#[test]
fn the_state_line_shows_not_calibrated_best_effort_and_guided() {
    for (state, text) in [
        (
            hand_state(0, 0.0, 0.0),
            "Compass calibration: Not calibrated",
        ),
        (
            hand_state(1, -14.0, 0.0),
            "Compass calibration: Best effort",
        ),
        (
            hand_state(0, -14.0, 0.0),
            "Compass calibration: best effort (older calibration)",
        ),
        (
            hand_state(2, -14.0, 48.2),
            "Compass calibration: Guided (expected field 48.2 uT, inclination 63.5 deg)",
        ),
    ] {
        let window = Window::over_state(state, Access::Configure);
        assert!(window.shows(text), "{text}");
    }
}

#[test]
fn an_operate_link_disables_the_buttons_with_the_reason_and_so_does_an_armed_vehicle() {
    let window = Window::over_state(hand_state(0, 0.0, 0.0), Access::Operate);
    assert!(window.disabled(GUIDED));
    assert!(window.disabled("Quick calibration (best effort)"));
    assert!(window.shows("Configure needs a serial or TCP link; this link is operate-only"));
    assert!(
        window
            .shows("Heading errors of 15 deg and more were measured with this kind of calibration")
    );

    let mut armed = hand_state(0, 0.0, 0.0);
    armed.messages.entries.insert(
        MessageKey {
            system_id: 1,
            component_id: 1,
            message_id: 0,
            instance: 0,
        },
        MessageEntry {
            name: "HEARTBEAT".into(),
            label: "HEARTBEAT".into(),
            count: 1,
            rate_hz: 1.0,
            last_seen: Duration::ZERO,
            fields: vec![Field {
                name: "base_mode".into(),
                value: FieldValue::Number(128.0),
            }],
        },
    );
    let window = Window::over_state(armed, Access::Configure);
    assert!(window.disabled(GUIDED));
    assert!(window.shows("the vehicle is armed"));
}
