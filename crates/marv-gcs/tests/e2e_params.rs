//! End-to-end tests of the parameter feature: the ground station's MAVLink layer against the
//! simulated vehicle, over a real TCP socket (docs/link-contract.md, "Parameters"). Nothing is
//! mocked: `gcs_transport::open` -> `gcs_mavlink::Session` <-> `gcs_sim::serve_tcp`.

use std::io::Write;
use std::net::TcpListener;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use gcs_mavlink::param_file;
use gcs_mavlink::services::params::{
    FetchStatus, ParamEncoding, ParamTable, ParamValue, WriteStatus,
};
use gcs_mavlink::{Access, Body, Command, Deframer, Message, Session, SessionConfig, VehicleState};
use gcs_sim::{SimConfig, SimVehicle};
use gcs_transport::Endpoint;

const POLL: Duration = Duration::from_millis(20);

/// Counts the PARAM_REQUEST_LIST frames that leave the station, as the simulator receives them.
struct CountingWriter {
    inner: Box<dyn Write + Send>,
    deframer: Deframer,
    list_requests: Arc<AtomicUsize>,
}

impl Write for CountingWriter {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let written = self.inner.write(buf)?;
        let mut frames = Vec::new();
        self.deframer.push(&buf[..written], &mut frames);
        for frame in frames {
            if matches!(frame.body, Body::Known(Message::PARAM_REQUEST_LIST(_))) {
                self.list_requests.fetch_add(1, Ordering::Relaxed);
            }
        }
        Ok(written)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        self.inner.flush()
    }
}

/// A simulator behind a TCP listener, and a ground-station session connected to it.
struct Rig {
    vehicle: Arc<Mutex<SimVehicle>>,
    names: Vec<String>,
    stop: Arc<AtomicBool>,
    server: Option<JoinHandle<()>>,
    session: Option<Session>,
    connected_at: Instant,
    list_requests: Arc<AtomicUsize>,
}

impl Rig {
    fn start(config: SimConfig) -> Rig {
        Rig::start_with_access(config, Access::Configure)
    }

    fn start_with_access(config: SimConfig, access: Access) -> Rig {
        let listener = TcpListener::bind("127.0.0.1:0").expect("bind 127.0.0.1:0");
        let addr = listener.local_addr().expect("local address").to_string();
        let names = config.params.iter().map(|p| p.name.clone()).collect();
        let vehicle = Arc::new(Mutex::new(SimVehicle::new(config)));
        let stop = Arc::new(AtomicBool::new(false));
        let server = {
            let vehicle = Arc::clone(&vehicle);
            let stop = Arc::clone(&stop);
            std::thread::spawn(move || gcs_sim::serve_tcp(listener, vehicle, stop))
        };
        let mut rig = Rig {
            vehicle,
            names,
            stop,
            server: Some(server),
            session: None,
            connected_at: Instant::now(),
            list_requests: Arc::default(),
        };
        let link = gcs_transport::open(&Endpoint::Tcp { addr }).expect("open the TCP link");
        rig.connected_at = Instant::now();
        let writer = CountingWriter {
            inner: link.writer,
            deframer: Deframer::new(),
            list_requests: Arc::clone(&rig.list_requests),
        };
        rig.session = Some(Session::start(
            link.reader,
            Box::new(writer),
            SessionConfig {
                access,
                ..SessionConfig::default()
            },
            Box::new(|| {}),
        ));
        rig
    }

    fn session(&self) -> &Session {
        self.session.as_ref().expect("session")
    }

    fn state(&self) -> MutexGuard<'_, VehicleState> {
        self.session().state()
    }

    fn sim(&self) -> MutexGuard<'_, SimVehicle> {
        self.vehicle.lock().unwrap_or_else(PoisonError::into_inner)
    }

    fn sim_param(&self, name: &str) -> Option<gcs_sim::ParamValue> {
        self.sim().param(name)
    }

    fn sim_params(&self) -> Vec<(String, gcs_sim::ParamValue)> {
        let sim = self.sim();
        self.names
            .iter()
            .map(|name| {
                let value = sim.param(name).expect("the simulator has its own param");
                (name.clone(), value)
            })
            .collect()
    }

    /// Polls the session state every 20 ms until `condition` holds; the time it took, or `None`
    /// when `seconds` passed first. The state lock is never held while sleeping.
    fn wait(
        &self,
        seconds: f64,
        mut condition: impl FnMut(&VehicleState) -> bool,
    ) -> Option<Duration> {
        let begun = Instant::now();
        let deadline = begun + Duration::from_secs_f64(seconds);
        loop {
            if condition(&self.state()) {
                return Some(begun.elapsed());
            }
            if Instant::now() >= deadline {
                return None;
            }
            std::thread::sleep(POLL);
        }
    }

    /// Waits for `fetch == Complete`; panics with the state it found otherwise.
    fn wait_complete(&self, seconds: f64) -> Duration {
        self.wait(seconds, |s| {
            matches!(
                s.params.fetch,
                FetchStatus::Complete | FetchStatus::Failed(_)
            )
        })
        .map(|took| {
            let s = self.state();
            assert!(
                s.params.fetch == FetchStatus::Complete,
                "the fetch failed after {took:?}: fetch={:?}, {} params in the table",
                s.params.fetch,
                s.params.params.len()
            );
            took
        })
        .unwrap_or_else(|| {
            let s = self.state();
            panic!(
                "no Complete within {seconds} s: fetch={:?}, {} params in the table",
                s.params.fetch,
                s.params.params.len()
            )
        })
    }
}

impl Drop for Rig {
    fn drop(&mut self) {
        self.session = None;
        self.stop.store(true, Ordering::Relaxed);
        if let Some(server) = self.server.take() {
            let _ = server.join();
        }
    }
}

fn from_sim(value: gcs_sim::ParamValue) -> ParamValue {
    match value {
        gcs_sim::ParamValue::Uint8(v) => ParamValue::U8(v),
        gcs_sim::ParamValue::Int8(v) => ParamValue::I8(v),
        gcs_sim::ParamValue::Uint16(v) => ParamValue::U16(v),
        gcs_sim::ParamValue::Int16(v) => ParamValue::I16(v),
        gcs_sim::ParamValue::Uint32(v) => ParamValue::U32(v),
        gcs_sim::ParamValue::Int32(v) => ParamValue::I32(v),
        gcs_sim::ParamValue::Real32(v) => ParamValue::F32(v),
    }
}

fn to_sim(value: ParamValue) -> gcs_sim::ParamValue {
    match value {
        ParamValue::U8(v) => gcs_sim::ParamValue::Uint8(v),
        ParamValue::I8(v) => gcs_sim::ParamValue::Int8(v),
        ParamValue::U16(v) => gcs_sim::ParamValue::Uint16(v),
        ParamValue::I16(v) => gcs_sim::ParamValue::Int16(v),
        ParamValue::U32(v) => gcs_sim::ParamValue::Uint32(v),
        ParamValue::I32(v) => gcs_sim::ParamValue::Int32(v),
        ParamValue::F32(v) => gcs_sim::ParamValue::Real32(v),
    }
}

/// What a 32-bit float can carry of `value`. A C-cast vehicle puts the number into a float, so an
/// integer above 2^24 arrives rounded to the nearest float; every other type is exact.
fn through_f32(value: ParamValue) -> ParamValue {
    match value {
        ParamValue::U32(v) => ParamValue::U32(v as f32 as u32),
        ParamValue::I32(v) => ParamValue::I32(v as f32 as i32),
        other => other,
    }
}

/// Every difference between the table and the simulator's parameters, as text; empty when equal.
fn mismatches(
    table: &ParamTable,
    sim: &[(String, gcs_sim::ParamValue)],
    expected: impl Fn(ParamValue) -> ParamValue,
) -> Vec<String> {
    let mut problems = Vec::new();
    if table.params.len() != sim.len() {
        problems.push(format!(
            "table has {} params, simulator {}",
            table.params.len(),
            sim.len()
        ));
    }
    for (name, value) in sim {
        let want = expected(from_sim(*value));
        match table.value(name) {
            None => problems.push(format!("{name}: missing from the table")),
            Some(got) if got != want || got.kind() != want.kind() => {
                problems.push(format!("{name}: table {got:?}, simulator {want:?}"));
            }
            Some(_) => {}
        }
    }
    problems
}

fn config_with_encoding(config: SimConfig, encoding: gcs_sim::ParamEncoding) -> SimConfig {
    SimConfig {
        param_encoding: encoding,
        ..config
    }
}

#[test]
fn default_vehicle_list_is_complete_and_exact() {
    let rig = Rig::start(SimConfig::default());
    let took = rig.wait_complete(4.0);
    let sim = rig.sim_params();
    let state = rig.state();
    assert_eq!(sim.len(), SimConfig::default().params.len());
    assert!(
        mismatches(&state.params, &sim, |v| v).is_empty(),
        "{:?} (Complete after {took:?})",
        mismatches(&state.params, &sim, |v| v)
    );
    assert!(state.params.encoding_known, "encoding_known");
    assert_eq!(state.params.encoding, ParamEncoding::Bytewise);
}

#[test]
fn lossy_link_still_completes_with_every_value_right() {
    let config = SimConfig {
        drop_tx_percent: 20,
        drop_rx_percent: 20,
        ..SimConfig::default().with_synthetic_params(300)
    };
    let rig = Rig::start(config);
    let took = rig.wait_complete(60.0);
    eprintln!("lossy link (20% each way, 357 params): Complete after {took:?}");
    let sim = rig.sim_params();
    let state = rig.state();
    let problems = mismatches(&state.params, &sim, |v| v);
    assert!(
        problems.is_empty(),
        "{} problems, first: {:?}",
        problems.len(),
        &problems[..problems.len().min(5)]
    );
    assert_eq!(state.params.params.len(), 357);
}

#[test]
fn writes_are_confirmed_and_reach_the_vehicle() {
    let rig = Rig::start(SimConfig::default());
    rig.wait_complete(4.0);
    let cases = [
        ("IMU_RATE_HZ", ParamValue::I32(400)),
        ("BAT_LOW_V", ParamValue::F32(13.5)),
    ];
    for (name, value) in cases {
        let old = rig.sim_param(name).expect("param exists");
        assert_ne!(from_sim(old), value, "the test must change {name}");
        rig.session().send(Command::SetParam {
            name: name.to_owned(),
            value,
        });
        let done = rig.wait(3.0, |s| {
            s.params.value(name) == Some(value) && !s.params.writes.contains_key(name)
        });
        let state = rig.state();
        assert!(
            done.is_some(),
            "{name}: table {:?}, writes {:?}, texts {:?}",
            state.params.value(name),
            state.params.writes.get(name),
            state
                .texts
                .entries
                .iter()
                .map(|e| &e.text)
                .collect::<Vec<_>>()
        );
        drop(state);
        assert_eq!(
            rig.sim_param(name),
            Some(to_sim(value)),
            "{name} in the simulator"
        );
    }
}

#[test]
fn refused_write_shows_the_value_in_effect() {
    let rig = Rig::start(SimConfig::default());
    rig.wait_complete(4.0);
    let name = "SYS_UID";
    let old = rig.sim_param(name).expect("read-only param exists");
    let old_gcs = from_sim(old);
    rig.session().send(Command::SetParam {
        name: name.to_owned(),
        value: ParamValue::I32(5),
    });
    let done = rig.wait(3.0, |s| {
        matches!(s.params.writes.get(name), Some(WriteStatus::Refused { .. }))
    });
    let state = rig.state();
    assert!(
        done.is_some(),
        "no Refused: writes {:?}",
        state.params.writes.get(name)
    );
    assert_eq!(
        state.params.writes.get(name),
        Some(&WriteStatus::Refused {
            value: ParamValue::I32(5),
            in_effect: old_gcs
        })
    );
    assert_eq!(state.params.value(name), Some(old_gcs));
    drop(state);
    assert_eq!(rig.sim_param(name), Some(old), "the simulator's value");
}

#[test]
fn a_refused_write_is_one_param_set_one_text_and_refused_within_one_echo() {
    let rig = Rig::start(SimConfig::default());
    rig.wait_complete(4.0);
    let name = "SYS_UID";
    let sent = Instant::now();
    rig.session().send(Command::SetParam {
        name: name.to_owned(),
        value: ParamValue::I32(5),
    });
    let done = rig.wait(3.0, |s| {
        matches!(s.params.writes.get(name), Some(WriteStatus::Refused { .. }))
            && s.texts
                .entries
                .iter()
                .any(|e| e.text == "SYS_UID: read only")
    });
    let elapsed = sent.elapsed();
    assert!(
        done.is_some(),
        "no Refused and text: {:?}",
        rig.state().params.writes.get(name)
    );
    assert!(
        elapsed < Duration::from_millis(400),
        "the refusal took {elapsed:?}"
    );
    // Wait past the old retry interval: nothing more is sent.
    std::thread::sleep(Duration::from_millis(1700));
    assert_eq!(rig.sim().param_sets_received(), 1);
    let state = rig.state();
    let texts: Vec<_> = state
        .texts
        .entries
        .iter()
        .filter(|e| e.text.starts_with("SYS_UID"))
        .collect();
    assert_eq!(texts.len(), 1);
    assert_eq!(
        texts[0].severity,
        gcs_mavlink::services::texts::Severity::Warning
    );
}

#[test]
fn a_write_whose_first_two_echoes_were_lost_succeeds_on_the_third_send() {
    let rig = Rig::start(SimConfig {
        drop_first_param_echoes: 2,
        ..SimConfig::default()
    });
    rig.wait_complete(4.0);
    let name = "LOG_RATE_HZ";
    rig.session().send(Command::SetParam {
        name: name.to_owned(),
        value: ParamValue::I32(7),
    });
    let done = rig.wait(4.0, |s| {
        s.params.value(name) == Some(ParamValue::I32(7)) && !s.params.writes.contains_key(name)
    });
    assert!(
        done.is_some(),
        "not confirmed: {:?}",
        rig.state().params.writes.get(name)
    );
    assert_eq!(rig.sim().param_sets_received(), 3);
    assert_eq!(rig.sim_param(name), Some(to_sim(ParamValue::I32(7))));
}

#[test]
fn unanswered_write_is_reported_after_three_sends() {
    let rig = Rig::start(SimConfig::default());
    rig.wait_complete(4.0);
    let name = "LOG_RATE_HZ";
    rig.sim().set_silent(true);
    let sent = Instant::now();
    rig.session().send(Command::SetParam {
        name: name.to_owned(),
        value: ParamValue::I32(7),
    });
    let done = rig.wait(3.0, |s| {
        matches!(
            s.params.writes.get(name),
            Some(WriteStatus::Unanswered { .. })
        )
    });
    let elapsed = sent.elapsed();
    let writes = rig.state().params.writes.get(name).cloned();
    rig.sim().set_silent(false);
    assert!(done.is_some(), "no Unanswered within 3 s: {writes:?}");
    assert_eq!(
        writes,
        Some(WriteStatus::Unanswered {
            value: ParamValue::I32(7)
        })
    );
    assert!(
        elapsed >= Duration::from_millis(1200),
        "Unanswered after only {elapsed:?}"
    );
}

#[test]
fn fallback_when_the_version_request_is_never_answered() {
    let config = SimConfig {
        answer_version_request: false,
        ..SimConfig::default()
    };
    let rig = Rig::start(config);
    rig.wait_complete(4.0);
    let took = rig
        .wait(3.0, |s| s.params.encoding_known)
        .map(|_| rig.connected_at.elapsed());
    let state = rig.state();
    assert!(took.is_some(), "encoding_known still false after 3 s");
    eprintln!("fallback: encoding known {took:?} after connecting");
    assert_eq!(state.params.encoding, ParamEncoding::Bytewise);
    let sim = rig.sim_params();
    assert!(mismatches(&state.params, &sim, |v| v).is_empty());
    assert!(
        took.unwrap() >= Duration::from_millis(1000),
        "fallback applied after only {took:?}: the version request cannot have been tried 3 times"
    );
}

#[test]
fn every_type_bytewise() {
    let rig = Rig::start(config_with_encoding(
        SimConfig::default().with_every_param_type(),
        gcs_sim::ParamEncoding::Bytewise,
    ));
    rig.wait_complete(4.0);
    let sim = rig.sim_params();
    let state = rig.state();
    assert_eq!(state.params.encoding, ParamEncoding::Bytewise);
    assert!(state.params.encoding_known);
    assert_eq!(sim.len(), 62);
    let problems = mismatches(&state.params, &sim, |v| v);
    assert!(problems.is_empty(), "{problems:?}");
    assert_eq!(
        state.params.value("TYPE_U32"),
        Some(ParamValue::U32(3_735_928_559))
    );
    assert_eq!(state.params.value("TYPE_I8"), Some(ParamValue::I8(-3)));
}

#[test]
fn every_type_ccast() {
    let rig = Rig::start(config_with_encoding(
        SimConfig::default().with_every_param_type(),
        gcs_sim::ParamEncoding::CCast,
    ));
    rig.wait_complete(4.0);
    let known = rig.wait(3.0, |s| s.params.encoding_known);
    let sim = rig.sim_params();
    let state = rig.state();
    assert!(known.is_some(), "encoding_known still false");
    assert_eq!(state.params.encoding, ParamEncoding::CCast);
    // A C-cast vehicle sends the number as a 32-bit float, which holds an integer exactly only up
    // to 2^24: TYPE_U32 and SYS_UID arrive as the nearest float, so the expectation goes through
    // an `as f32` round trip. The simulator's own value stays exact.
    let problems = mismatches(&state.params, &sim, through_f32);
    assert!(problems.is_empty(), "{problems:?}");
    assert_eq!(
        state.params.value("TYPE_U32"),
        Some(ParamValue::U32(3_735_928_559_u32 as f32 as u32))
    );
    assert_eq!(state.params.value("TYPE_I8"), Some(ParamValue::I8(-3)));
}

#[test]
fn an_operate_session_downloads_nothing_at_connect_and_a_refresh_brings_the_list() {
    let rig = Rig::start_with_access(SimConfig::default(), Access::Operate);
    assert!(
        rig.wait(4.0, |s| s.link.status == gcs_mavlink::LinkStatus::Connected)
            .is_some(),
        "never connected"
    );
    std::thread::sleep(Duration::from_millis(2_500));
    assert_eq!(rig.list_requests.load(Ordering::Relaxed), 0);
    {
        let state = rig.state();
        assert_eq!(state.params.fetch, FetchStatus::Idle);
        assert!(state.params.params.is_empty());
    }
    rig.session().send(Command::RefreshParams);
    rig.wait_complete(6.0);
    assert_eq!(rig.list_requests.load(Ordering::Relaxed), 1);
    assert!(!rig.state().params.params.is_empty());
    assert_eq!(rig.state().link.refused, None);
}

#[test]
fn an_operate_session_cannot_write_a_parameter_but_a_configure_session_can() {
    let rig = Rig::start_with_access(SimConfig::default(), Access::Operate);
    rig.wait(4.0, |s| s.link.status == gcs_mavlink::LinkStatus::Connected)
        .expect("connected");
    rig.session().send(Command::RefreshParams);
    rig.wait_complete(6.0);
    let name = rig.names[0].clone();
    let before = rig.sim_param(&name).expect("sim has it");
    let value = rig.state().params.value(&name).expect("in the table");
    rig.session().send(Command::SetParam {
        name: name.clone(),
        value,
    });
    std::thread::sleep(Duration::from_millis(600));
    assert_eq!(rig.sim_param(&name), Some(before));
    let state = rig.state();
    assert!(state.params.writes.is_empty());
    assert_eq!(state.link.refused.as_ref().map(|r| r.count), Some(1));
}

#[test]
fn a_configure_session_still_downloads_at_connect() {
    let rig = Rig::start_with_access(SimConfig::default(), Access::Configure);
    rig.wait_complete(4.0);
    assert_eq!(rig.list_requests.load(Ordering::Relaxed), 1);
}

#[test]
fn refresh_runs_the_whole_fetch_again() {
    let rig = Rig::start(SimConfig::default().with_synthetic_params(300));
    rig.wait_complete(10.0);
    let count = rig.state().params.params.len();
    assert_eq!(count, 357);
    rig.session().send(Command::RefreshParams);
    let left = rig.wait(4.0, |s| s.params.fetch != FetchStatus::Complete);
    assert!(left.is_some(), "the status never left Complete");
    let back = rig.wait(4.0, |s| s.params.fetch == FetchStatus::Complete);
    let state = rig.state();
    assert!(
        back.is_some(),
        "no Complete again: {:?}",
        state.params.fetch
    );
    assert_eq!(state.params.params.len(), count);
}

#[test]
fn parameter_file_round_trips_on_real_data() {
    let rig = Rig::start(SimConfig::default());
    rig.wait_complete(4.0);
    let table = rig.state().params.clone();
    let text = param_file::to_text(&table);
    let wanted = param_file::parse(&text).expect("the saved text parses");
    assert_eq!(wanted.len(), table.params.len());
    let (changed, problems) = param_file::changes(&table, &wanted);
    assert!(
        changed.is_empty() && problems.is_empty(),
        "{changed:?} {problems:?}"
    );

    let edit = |name: &str, new: &str| {
        text.lines()
            .map(|line| {
                if line.starts_with(&format!("{name},")) {
                    format!("{name},{new}")
                } else {
                    line.to_owned()
                }
            })
            .collect::<Vec<_>>()
            .join("\n")
    };
    for (name, number, expected) in [
        ("IMU_RATE_HZ", "801", ParamValue::I32(801)),
        ("BAT_LOW_V", "13.5", ParamValue::F32(13.5)),
    ] {
        let edited = edit(name, number);
        assert_ne!(edited, text, "the line for {name} was not found");
        let wanted = param_file::parse(&edited).expect("the edited text parses");
        let (changed, problems) = param_file::changes(&table, &wanted);
        assert!(problems.is_empty(), "{problems:?}");
        assert_eq!(changed, vec![(name.to_owned(), expected)]);
    }
}
