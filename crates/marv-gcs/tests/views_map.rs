//! Headless test of the Map tab with a hand-built message store. The tab comes from `views::all()`, which
//! gives the Map no tiles: nothing here touches the network.

use std::cell::RefCell;
use std::rc::Rc;
use std::time::Duration;

use egui_kittest::Harness;
use egui_kittest::kittest::{NodeT, Queryable};
use gcs_mavlink::services::messages::{Field, FieldValue, MessageEntry, MessageKey};
use gcs_mavlink::services::missions::{
    MissionKind, MissionPoint, MissionStatus, MissionsState, VehicleList,
};
use gcs_mavlink::{
    Access, Command, FlightCommand, FlightOutcome, FlightResult, LinkState, LinkStatus, VehicleId,
    VehicleState,
};
use marv_gcs::missions::{Store, Waypoint};
use marv_gcs::views::{self, View, ViewCtx};

const ATTRIBUTION: &str = "OpenFreeMap © OpenMapTiles © OpenStreetMap contributors";

fn view() -> Box<dyn View> {
    views::all()
        .into_iter()
        .find(|v| v.name() == "Map")
        .expect("Map view is registered")
}

fn number(name: &str, v: f64) -> Field {
    Field {
        name: name.into(),
        value: FieldValue::Number(v),
    }
}

fn with_gps(state: &mut VehicleState, fix: &str, lat: f64, lon: f64, seen: f64) {
    state.messages.entries.insert(
        MessageKey {
            system_id: 1,
            component_id: 1,
            message_id: 24,
            instance: 0,
        },
        MessageEntry {
            name: "GPS_RAW_INT".into(),
            label: "GPS_RAW_INT".into(),
            count: 1,
            rate_hz: 1.0,
            last_seen: Duration::from_secs_f64(seen),
            fields: vec![
                Field {
                    name: "fix_type".into(),
                    value: FieldValue::Text(format!("GPS_FIX_TYPE_{fix}")),
                },
                number("lat", (lat * 1e7).round()),
                number("lon", (lon * 1e7).round()),
                number("cog", 9000.0),
            ],
        },
    );
}

fn draw<'a>(view: &'a mut dyn View, state: &'a VehicleState, now: Option<f64>) -> Harness<'a> {
    let mut harness = Harness::new_ui(move |ui| {
        let mut commands = Vec::new();
        let mut cx = ViewCtx {
            state,
            commands: &mut commands,
            now: now.map(Duration::from_secs_f64),
            access: gcs_mavlink::Access::Configure,
        };
        view.ui(ui, &mut cx);
        assert!(commands.is_empty(), "the Map sends nothing");
    });
    harness.run();
    harness
}

#[test]
fn before_a_position_the_map_says_it_waits_and_always_shows_the_attribution() {
    let mut view = view();
    let state = VehicleState::default();
    let harness = draw(&mut *view, &state, None);
    harness.get_by_label("waiting for a GPS fix");
    harness.get_by_label(ATTRIBUTION);
    harness.get_by_label("Follow vehicle");
    assert!(harness.query_by_label("no position").is_none());
}

#[test]
fn a_fresh_fix_shows_the_coordinates_and_no_warning() {
    let mut view = view();
    let mut state = VehicleState::default();
    with_gps(&mut state, "3D_FIX", 45.0, 9.0, 10.0);
    let harness = draw(&mut *view, &state, Some(11.0));
    harness.get_by_label("45.000000, 9.000000");
    harness.get_by_label(ATTRIBUTION);
    for text in ["waiting for a GPS fix", "no GPS fix", "no position"] {
        assert!(harness.query_by_label(text).is_none(), "{text}");
    }
}

#[test]
fn an_old_message_or_a_message_without_a_fix_greys_the_last_position_and_says_why() {
    let mut view = view();
    let mut state = VehicleState::default();
    with_gps(&mut state, "3D_FIX", 45.0, 9.0, 10.0);
    drop(draw(&mut *view, &state, Some(11.0)));
    let harness = draw(&mut *view, &state, Some(14.0));
    harness.get_by_label("no position");
    harness.get_by_label("45.000000, 9.000000");
    drop(harness);

    with_gps(&mut state, "NO_FIX", 0.0, 0.0, 15.0);
    let harness = draw(&mut *view, &state, Some(15.5));
    harness.get_by_label("no GPS fix");
    harness.get_by_label("45.000000, 9.000000");
    harness.get_by_label(ATTRIBUTION);
}

#[test]
fn a_message_without_a_fix_alone_is_no_position() {
    let mut view = view();
    let mut state = VehicleState::default();
    with_gps(&mut state, "NO_FIX", 45.0, 9.0, 10.0);
    let harness = draw(&mut *view, &state, Some(10.5));
    harness.get_by_label("waiting for a GPS fix");
    assert!(harness.query_by_label("45.000000, 9.000000").is_none());
}

#[test]
fn centre_on_vehicle_is_for_when_the_map_is_not_following() {
    let mut view = view();
    let mut state = VehicleState::default();
    with_gps(&mut state, "3D_FIX", 45.0, 9.0, 10.0);
    let mut harness = draw(&mut *view, &state, Some(10.5));
    assert!(
        harness
            .get_by_label("Centre on vehicle")
            .accesskit_node()
            .is_disabled(),
        "following already centres on the vehicle"
    );
    harness.get_by_label("Follow vehicle").click();
    harness.run();
    assert!(
        !harness
            .get_by_label("Centre on vehicle")
            .accesskit_node()
            .is_disabled()
    );
    harness.get_by_label("Follow vehicle").click();
    harness.run();
    assert!(
        harness
            .get_by_label("Centre on vehicle")
            .accesskit_node()
            .is_disabled()
    );
}

#[test]
fn a_new_session_empties_the_map() {
    let mut view = view();
    let mut state = VehicleState::default();
    with_gps(&mut state, "3D_FIX", 45.0, 9.0, 10.0);
    drop(draw(&mut *view, &state, Some(10.5)));
    view.session_changed();
    let state = VehicleState::default();
    let harness = draw(&mut *view, &state, None);
    harness.get_by_label("waiting for a GPS fix");
    assert!(harness.query_by_label("45.000000, 9.000000").is_none());
}

const UPLOAD: &str = "Upload to vehicle";
const READ: &str = "Read from vehicle";
const CLEAR: &str = "Clear on vehicle";

fn connected() -> VehicleState {
    VehicleState {
        link: LinkState {
            status: LinkStatus::Connected,
            vehicle: Some(VehicleId {
                system_id: 1,
                component_id: 1,
                mav_type: "MAV_TYPE_QUADROTOR".into(),
                autopilot: "MAV_AUTOPILOT_GENERIC".into(),
            }),
            ..LinkState::default()
        },
        ..VehicleState::default()
    }
}

fn store_with_example(tag: &str) -> Store {
    let store = Store::new(std::env::temp_dir().join(format!(
        "marv-gcs-views-map-{tag}-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    )));
    let list: Vec<Waypoint> = [(10.0, -30.0, 10.0), (10.001, -30.0, 12.5)]
        .into_iter()
        .map(|(lat, lon, alt_m)| Waypoint { lat, lon, alt_m })
        .collect();
    store.save("mission", &list, None).unwrap();
    store
}

/// The Map over a state that the test can change between frames; the commands it sends are collected.
struct Window {
    harness: Harness<'static>,
    state: Rc<RefCell<VehicleState>>,
    sent: Rc<RefCell<Vec<Command>>>,
}

fn window(store: &Store, state: VehicleState, now: Option<f64>, access: Access) -> Window {
    let state = Rc::new(RefCell::new(state));
    let sent: Rc<RefCell<Vec<Command>>> = Rc::default();
    let mut view = views::map_with_store(Some(store.clone()));
    let (shared, collected) = (state.clone(), sent.clone());
    let mut harness = Harness::builder()
        .with_size(eframe::egui::vec2(1500.0, 1300.0))
        .build_ui(move |ui| {
            let state = shared.borrow().clone();
            let mut commands = Vec::new();
            let mut cx = ViewCtx {
                state: &state,
                commands: &mut commands,
                now: now.map(Duration::from_secs_f64),
                access,
            };
            view.ui(ui, &mut cx);
            collected.borrow_mut().extend(commands);
        });
    harness.run();
    Window {
        harness,
        state,
        sent,
    }
}

impl Window {
    fn disabled(&self, label: &str) -> bool {
        self.harness
            .get_by_label(label)
            .accesskit_node()
            .is_disabled()
    }

    fn shows(&self, text: &str) -> bool {
        self.harness.query_all_by_label(text).next().is_some()
    }
}

#[test]
fn without_a_vehicle_upload_read_and_clear_are_disabled_and_send_nothing() {
    let store = store_with_example("none");
    let mut window = window(&store, VehicleState::default(), None, Access::Configure);
    for button in [UPLOAD, READ, CLEAR] {
        assert!(window.disabled(button), "{button}");
        window.harness.get_by_label(button).click();
        window.harness.run();
    }
    assert!(window.sent.borrow().is_empty());
    // A link that is up but has heard no vehicle is not a vehicle either.
    let mut state = connected();
    state.link.vehicle = None;
    let window = self::window(&store, state, Some(5.0), Access::Configure);
    for button in [UPLOAD, READ, CLEAR] {
        assert!(window.disabled(button), "{button} with no heartbeat");
    }
    drop(window);
    let _ = std::fs::remove_dir_all(store.dir());
}

#[test]
fn with_a_vehicle_the_three_are_enabled_also_on_an_operate_only_link_and_the_upload_needs_a_list() {
    let store = store_with_example("enabled");
    for access in [Access::Configure, Access::Operate] {
        let window = window(&store, connected(), Some(5.0), access);
        for button in [UPLOAD, READ, CLEAR] {
            assert!(!window.disabled(button), "{button} on {access:?}");
        }
        assert!(
            window
                .shows("The vehicle only stores the list; nothing here makes it follow the list.")
        );
        for text in ["Edited list: amber discs.", "matches", "differs"] {
            assert!(
                window
                    .harness
                    .query_all_by_label_contains(text)
                    .next()
                    .is_none(),
                "{text}: nothing read yet"
            );
        }
    }
    let empty = Store::new(store.dir().join("none"));
    let window = window(&empty, connected(), Some(5.0), Access::Configure);
    assert!(window.disabled(UPLOAD), "an empty list is not an upload");
    assert!(!window.disabled(READ) && !window.disabled(CLEAR));
    let _ = std::fs::remove_dir_all(store.dir());
}

#[test]
fn a_press_sends_one_command_and_waits_for_the_service_before_believing_its_status() {
    let store = store_with_example("press");
    // The previous operation ended verified.
    let mut state = connected();
    state.missions = MissionsState {
        status: MissionStatus::Verified,
        kind: Some(MissionKind::Upload),
        runs: 1,
        vehicle: Some(VehicleList {
            plan_id: 5,
            items: Vec::new(),
            return_height: Some(30.0),
        }),
        ..MissionsState::default()
    };
    let mut window = window(&store, state, Some(5.0), Access::Configure);
    window.harness.get_by_label_contains("Upload verified");
    window.harness.get_by_label(READ).click();
    window.harness.run();
    assert_eq!(window.sent.borrow().len(), 1);
    assert!(matches!(window.sent.borrow()[0], Command::DownloadMission));
    // The service has not started it: the old verdict is not shown as this one's, and no button is live.
    window
        .harness
        .get_by_label("Request sent, waiting for the vehicle");
    assert!(
        window
            .harness
            .query_all_by_label_contains("verified")
            .next()
            .is_none()
    );
    for button in [UPLOAD, READ, CLEAR] {
        assert!(window.disabled(button), "{button}");
    }
    window.harness.run();
    assert_eq!(window.sent.borrow().len(), 1, "no second command");

    // The service starts it: the progress shows and the buttons stay off.
    {
        let mut state = window.state.borrow_mut();
        state.missions.runs = 2;
        state.missions.kind = Some(MissionKind::Download);
        state.missions.status = MissionStatus::Downloading {
            received: 1,
            total: Some(2),
        };
        state.missions.vehicle = None;
    }
    window.harness.run();
    assert!(window.shows("Reading 1 of 2 waypoints"));
    for button in [UPLOAD, READ, CLEAR] {
        assert!(window.disabled(button), "{button} while reading");
    }
    assert!(!window.shows("Request sent, waiting for the vehicle"));

    // The service ends it: the list is drawn as the vehicle's and compared.
    {
        let mut state = window.state.borrow_mut();
        state.missions.status = MissionStatus::Complete;
        state.missions.vehicle = Some(VehicleList {
            plan_id: 0,
            items: vec![MissionPoint {
                lat_e7: 100_000_000,
                lon_e7: -300_000_000,
                z: 10.0,
            }],
            return_height: Some(30.0),
        });
    }
    window.harness.run();
    window.harness.get_by_label_contains(
        "Read complete: the vehicle holds 1 waypoint, return height 30 m, plan id 0x00000000",
    );
    window
        .harness
        .get_by_label("Edited list: amber discs. Vehicle's list as last read: blue dashed rings.");
    window.harness.get_by_label(
        "The vehicle's list differs from the edited list: the vehicle holds 1 waypoint, the edited list has 2 waypoints",
    );
    for button in [UPLOAD, READ, CLEAR] {
        assert!(!window.disabled(button), "{button} when it is over");
    }
    assert_eq!(window.sent.borrow().len(), 1);
    drop(window);
    let _ = std::fs::remove_dir_all(store.dir());
}

#[test]
fn a_command_the_service_refused_does_not_leave_the_buttons_off() {
    let store = store_with_example("refused");
    let mut window = window(&store, connected(), Some(5.0), Access::Configure);
    window.harness.get_by_label(CLEAR).click();
    window.harness.run();
    window.harness.get_by_label("Confirm").click();
    window.harness.run();
    assert!(window.disabled(READ));
    // The service counted a refusal and started nothing.
    window.state.borrow_mut().missions.refused = 1;
    window.harness.run();
    assert!(!window.disabled(READ));
    drop(window);
    let _ = std::fs::remove_dir_all(store.dir());
}

#[test]
fn the_panel_has_a_return_height_field_and_upload_sends_it_with_the_list() {
    // The example list is saved without a return height: the field starts at the default altitude (30 m).
    let store = store_with_example("return-height");
    let mut window = window(&store, connected(), Some(5.0), Access::Configure);
    window.harness.get_by_label("Return height (m)");
    window.harness.get_by_label(UPLOAD).click();
    window.harness.run();
    window.harness.get_by_label("Confirm").click();
    window.harness.run();
    let sent = window.sent.borrow();
    let [
        Command::UploadMission {
            items,
            return_height,
        },
    ] = &sent[..]
    else {
        panic!("{sent:?}");
    };
    assert_eq!((items.len(), *return_height), (2, 30.0));
    drop(sent);
    drop(window);
    let _ = std::fs::remove_dir_all(store.dir());
}

const FLIGHT: [&str; 5] = ["Start", "Pause", "Abort", "Land", "Return"];

#[test]
fn the_five_flight_buttons_are_disabled_without_a_vehicle_and_there_is_no_stop_or_disarm_button() {
    let store = store_with_example("flight-off");
    let mut window = window(&store, VehicleState::default(), None, Access::Operate);
    for button in FLIGHT {
        assert!(window.disabled(button), "{button}");
        window.harness.get_by_label(button).click();
        window.harness.run();
    }
    assert!(window.sent.borrow().is_empty());
    for word in ["Kill", "Disarm", "Stop motors", "Emergency"] {
        assert!(
            window
                .harness
                .query_all_by_label_contains(word)
                .next()
                .is_none(),
            "{word}"
        );
    }
    let _ = std::fs::remove_dir_all(store.dir());
}

#[test]
fn a_flight_button_sends_its_command_and_start_asks_first() {
    let store = store_with_example("flight-send");
    let mut window = window(&store, connected(), Some(5.0), Access::Operate);
    for button in FLIGHT {
        assert!(!window.disabled(button), "{button}");
    }
    for (button, command) in [
        ("Pause", FlightCommand::Pause),
        ("Abort", FlightCommand::Abort),
        ("Land", FlightCommand::Land),
        ("Return", FlightCommand::Return),
    ] {
        window.sent.borrow_mut().clear();
        window.harness.get_by_label(button).click();
        window.harness.run();
        assert!(
            matches!(window.sent.borrow().as_slice(), [Command::Flight(c)] if *c == command),
            "{button}"
        );
    }
    window.sent.borrow_mut().clear();
    window.harness.get_by_label("Start").click();
    window.harness.run();
    assert!(
        window.sent.borrow().is_empty(),
        "Start waits for its confirmation"
    );
    window.harness.get_by_label("Start the stored mission?");
    window.harness.get_by_label("Confirm").click();
    window.harness.run();
    assert!(matches!(
        window.sent.borrow().as_slice(),
        [Command::Flight(FlightCommand::Start)]
    ));
    let _ = std::fs::remove_dir_all(store.dir());
}

#[test]
fn a_denial_shows_as_an_error_until_dismissed_and_the_mode_is_in_words() {
    let store = store_with_example("flight-deny");
    let mut state = connected();
    state.flight.mode_code = Some(3);
    state
        .flight
        .begin(FlightCommand::Start, Duration::from_secs(4));
    state.flight.finish(
        FlightResult {
            command: FlightCommand::Start,
            outcome: FlightOutcome::Denied { code: 11 },
        },
        Duration::from_secs(5),
    );
    let mut window = window(&store, state, Some(5.0), Access::Operate);
    let refused = |w: &Window| {
        w.harness
            .query_all_by_label_contains(": A waypoint is more than 1 km from the take-off point")
            .next()
            .is_some()
    };
    window.harness.get_by_label("Mode: Holding position");
    assert!(refused(&window));
    window.harness.get_by_label_contains("Start REFUSED at ");
    window.harness.get_by_label("Dismiss").click();
    window.harness.run();
    assert!(!refused(&window));
    let _ = std::fs::remove_dir_all(store.dir());
}

#[test]
fn the_home_set_badge_appears_with_a_home_and_goes_with_stop() {
    let store = store_with_example("flight-home");
    let mut state = connected();
    state.flight.heartbeat(3);
    let window_without = window(&store, state.clone(), Some(5.0), Access::Operate);
    assert!(window_without.harness.query_by_label("HOME SET").is_none());
    state.flight.home = Some(gcs_mavlink::Home {
        lat_e7: 473_977_000,
        lon_e7: 85_456_000,
        alt_mm: 488_000,
    });
    let with_home = window(&store, state.clone(), Some(5.0), Access::Operate);
    with_home.harness.get_by_label("HOME SET");
    state.flight.heartbeat(7);
    let stopped = window(&store, state, Some(5.0), Access::Operate);
    assert!(stopped.harness.query_by_label("HOME SET").is_none());
    let _ = std::fs::remove_dir_all(store.dir());
}

#[test]
fn an_acceptance_is_stated_and_stays_on_screen_until_dismissed() {
    let store = store_with_example("flight-accept");
    let mut state = connected();
    state.flight.mode_code = Some(4);
    state
        .flight
        .begin(FlightCommand::Start, Duration::from_secs(4));
    state.flight.finish(
        FlightResult {
            command: FlightCommand::Start,
            outcome: FlightOutcome::Accepted,
        },
        Duration::from_secs(5),
    );
    let mut window = window(&store, state, Some(5.0), Access::Operate);
    window.harness.run();
    window
        .harness
        .get_by_label_contains("Start ACCEPTED by the vehicle at ");
    window.harness.run();
    window
        .harness
        .get_by_label_contains("Start ACCEPTED by the vehicle at ");
    window.harness.get_by_label("Dismiss").click();
    window.harness.run();
    assert!(
        window
            .harness
            .query_all_by_label_contains("Start ACCEPTED")
            .next()
            .is_none()
    );
    let _ = std::fs::remove_dir_all(store.dir());
}

#[test]
fn the_takeover_panel_shows_with_a_hands_off_mode_and_full_width_when_the_link_is_lost() {
    let store = store_with_example("flight-panel");
    let mut state = connected();
    state.link.last_heartbeat = Some(Duration::from_secs(1));
    state.flight.mode_code = Some(4);
    let window_up = window(&store, state.clone(), Some(2.0), Access::Operate);
    assert!(
        window_up
            .harness
            .query_all_by_label_contains("MISSION SWITCH ON: the laptop flies.")
            .next()
            .is_some()
    );
    assert!(window_up.shows("The vehicle reports: Flying the stored mission"));
    state.link.status = LinkStatus::Lost;
    let window_lost = window(&store, state.clone(), Some(13.0), Access::Operate);
    assert!(
        window_lost
            .harness
            .query_all_by_label_contains("MISSION SWITCH ON: the laptop flies.")
            .next()
            .is_some()
    );
    assert!(window_lost.shows(
        "The link to the vehicle stopped. Last reported 12 s ago: Flying the stored mission"
    ));
    state.flight.mode_code = Some(2);
    let window_takeover = window(&store, state.clone(), Some(13.0), Access::Operate);
    assert!(window_takeover.shows("Control locked: mode switch to POSITION, ARM ON"));
    assert!(
        window_takeover
            .shows("When the link returns with the mission switch ON the vehicle carries on.")
    );
    state.flight.mode_code = Some(4);
    let window_auto = window(&store, state.clone(), Some(13.0), Access::Operate);
    assert!(!window_auto.shows("Control locked: mode switch to POSITION, ARM ON"));
    state.flight.mode_code = Some(1);
    let window_pilot = window(&store, state, Some(13.0), Access::Operate);
    assert!(
        window_pilot
            .harness
            .query_all_by_label_contains("MISSION SWITCH ON: the laptop flies.")
            .next()
            .is_none()
    );
    let _ = std::fs::remove_dir_all(store.dir());
}
