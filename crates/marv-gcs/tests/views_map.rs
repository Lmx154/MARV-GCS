//! Headless test of the Map tab with a hand-built message store. The tab comes from `views::all()`, which
//! gives the Map no tiles: nothing here touches the network.

use std::time::Duration;

use egui_kittest::Harness;
use egui_kittest::kittest::{NodeT, Queryable};
use gcs_mavlink::VehicleState;
use gcs_mavlink::services::messages::{Field, FieldValue, MessageEntry, MessageKey};
use marv_gcs::views::{self, View, ViewCtx};

const ATTRIBUTION: &str = "© OpenStreetMap contributors";

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
