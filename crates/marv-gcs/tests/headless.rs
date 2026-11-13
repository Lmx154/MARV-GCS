//! Headless tests: the views and the connection bar drawn by egui_kittest, no window.

use std::time::Duration;

use egui_kittest::Harness;
use egui_kittest::kittest::Queryable;
use gcs_mavlink::{LinkState, LinkStatus, VehicleId, VehicleState};
use gcs_transport::Endpoint;
use marv_gcs::connection::{Action, Bar};
use marv_gcs::status;
use marv_gcs::views::{self, ViewCtx};

fn connected_state() -> VehicleState {
    let mut state = VehicleState {
        link: LinkState {
            status: LinkStatus::Connected,
            vehicle: Some(VehicleId {
                system_id: 42,
                component_id: 7,
                mav_type: "MAV_TYPE_QUADROTOR".into(),
                autopilot: "MAV_AUTOPILOT_ARDUPILOTMEGA".into(),
            }),
            last_heartbeat: Some(Duration::from_secs(9)),
            tx_frames: 11,
            ..LinkState::default()
        },
        ..VehicleState::default()
    };
    state.link.rx.frames = 12345;
    state
}

fn draw_all_views(state: &VehicleState, now: Option<Duration>) {
    let mut all = views::all();
    assert!(!all.is_empty());
    for view in &mut all {
        let name = view.name();
        let mut harness = Harness::new_ui(|ui| {
            let mut commands = Vec::new();
            let mut cx = ViewCtx {
                state,
                commands: &mut commands,
                now,
            };
            view.ui(ui, &mut cx);
        });
        harness.run();
        assert!(!name.is_empty());
    }
}

#[test]
fn every_view_draws_a_default_state() {
    draw_all_views(&VehicleState::default(), None);
}

#[test]
fn every_view_draws_a_connected_state() {
    draw_all_views(&connected_state(), Some(Duration::from_secs(10)));
}

#[test]
fn link_view_shows_system_id_and_received_frames() {
    let state = connected_state();
    let mut link = views::all()
        .into_iter()
        .find(|v| v.name() == "Link")
        .expect("Link view is registered");
    let mut harness = Harness::new_ui(|ui| {
        let mut commands = Vec::new();
        let mut cx = ViewCtx {
            state: &state,
            commands: &mut commands,
            now: Some(Duration::from_secs(10)),
        };
        link.ui(ui, &mut cx);
    });
    harness.run();
    harness.get_by_label("42");
    harness.get_by_label("12345");
    harness.get_by_label("Connected — system 42, component 7, MAV_TYPE_QUADROTOR");
    harness.get_by_label("1.0 s");
}

#[test]
fn top_bar_offers_connect_without_a_session() {
    let mut bar = Bar::default();
    let mut harness = Harness::new_ui(|ui| {
        bar.ui(ui, false, status::not_connected());
    });
    harness.run();
    harness.get_by_label("Connect");
    assert!(harness.query_by_label("Disconnect").is_none());
}

#[test]
fn top_bar_offers_disconnect_with_a_session() {
    let mut bar = Bar::default();
    let mut harness = Harness::new_ui(|ui| {
        bar.ui(ui, true, status::not_connected());
    });
    harness.run();
    harness.get_by_label("Disconnect");
    assert!(harness.query_by_label("Connect").is_none());
}

#[test]
fn the_longest_status_stays_on_the_line_of_the_buttons_in_a_1100_pixel_window() {
    let reasons = [
        "Connected — system 255, component 255, MAV_TYPE_VTOL_TILTROTOR_WITH_A_LONG_NAME"
            .to_string(),
        format!(
            "Closed — read failed: {}",
            "connection reset by peer ".repeat(8)
        ),
    ];
    for text in reasons {
        let mut bar = Bar::default();
        let shown = text.clone();
        let mut harness = Harness::builder()
            .with_size(eframe::egui::vec2(1100.0, 120.0))
            .build_ui(move |ui| {
                bar.ui(ui, true, (shown.clone(), status::GREY));
            });
        harness.run();
        let button = harness.get_by_label("Disconnect").rect();
        let status = harness
            .get_by_label_contains(text.split(" —").next().unwrap())
            .rect();
        assert!(status.max.x <= 1100.0, "{status:?}");
        assert!(
            status.center().y > button.min.y && status.center().y < button.max.y,
            "status {status:?} is not on the line of the button {button:?}"
        );
    }
}

/// Draws the bar, lets `interact` click on it, and returns the action the bar produced.
fn bar_action(mut bar: Bar, interact: impl FnOnce(&mut Harness<'_>)) -> Option<Action> {
    let produced = std::cell::RefCell::new(None);
    {
        let mut harness = Harness::new_ui(|ui| {
            if let Some(action) = bar.ui(ui, false, status::not_connected()) {
                *produced.borrow_mut() = Some(action);
            }
        });
        harness.run();
        interact(&mut harness);
    }
    produced.into_inner()
}

#[test]
fn top_bar_selects_udp_and_reads_the_default_endpoint_back() {
    let action = bar_action(Bar::default(), |harness| {
        harness.get_by_label("UDP").click();
        harness.run();
        harness.get_by_label("Listen on");
        harness.get_by_label("Connect").click();
        harness.run();
    });
    assert_eq!(
        action,
        Some(Action::Connect(Endpoint::Udp {
            bind: "0.0.0.0:14550".parse().unwrap()
        }))
    );
}

#[test]
fn top_bar_shows_a_udp_endpoint_from_the_command_line() {
    let endpoint = Endpoint::Udp {
        bind: "127.0.0.1:14551".parse().unwrap(),
    };
    let mut bar = Bar::default();
    bar.set_endpoint(&endpoint);
    let action = bar_action(bar, |harness| {
        harness.get_by_label("Connect").click();
        harness.run();
    });
    assert_eq!(action, Some(Action::Connect(endpoint)));
}
