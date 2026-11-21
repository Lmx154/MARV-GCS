//! Headless test of the Devices view with a hand-built message store.

use std::time::Duration;

use egui_kittest::Harness;
use egui_kittest::kittest::Queryable;
use gcs_mavlink::VehicleState;
use gcs_mavlink::services::messages::{Field, FieldValue, MessageEntry, MessageKey};
use marv_gcs::views::{self, ViewCtx};

fn device(instance: u8, name: &str, state: &str) -> (MessageKey, MessageEntry) {
    let text = |n: &str, v: String| Field {
        name: n.into(),
        value: FieldValue::Text(v),
    };
    (
        MessageKey {
            system_id: 1,
            component_id: 1,
            message_id: 42_421,
            instance,
        },
        MessageEntry {
            name: "MARV_DEVICE_STATUS".into(),
            label: format!("MARV_DEVICE_STATUS[{instance}]"),
            count: 3,
            rate_hz: 1.0,
            last_seen: Duration::from_secs(9),
            fields: vec![
                text("device_id", format!("MARV_DEVICE_{name}")),
                text("state", format!("MARV_DEVICE_STATE_{state}")),
                text("last_error", "MARV_DEVICE_ERROR_NONE".into()),
                Field {
                    name: "samples".into(),
                    value: FieldValue::Number(12_345.0),
                },
            ],
        },
    )
}

#[test]
fn the_devices_view_shows_a_row_per_device_and_says_when_there_is_none() {
    let mut view = views::all()
        .into_iter()
        .find(|v| v.name() == "Devices")
        .expect("Devices view is registered");
    let mut state = VehicleState::default();
    let mut harness = Harness::new_ui(|ui| {
        let mut commands = Vec::new();
        let mut cx = ViewCtx {
            state: &state,
            commands: &mut commands,
            now: Some(Duration::from_secs(30)),
            access: gcs_mavlink::Access::Configure,
        };
        view.ui(ui, &mut cx);
        assert!(commands.is_empty());
    });
    harness.run();
    harness.get_by_label("No device status yet");
    drop(harness);

    for (key, entry) in [device(2, "IMU2", "DEGRADED"), device(1, "IMU1", "OK")] {
        state.messages.entries.insert(key, entry);
    }
    // Tall enough for the preset cards above the table: only the rows in view are drawn.
    let mut harness = Harness::builder()
        .with_size(eframe::egui::vec2(800.0, 1400.0))
        .build_ui(|ui| {
            let mut commands = Vec::new();
            let mut cx = ViewCtx {
                state: &state,
                commands: &mut commands,
                now: Some(Duration::from_secs(30)),
                access: gcs_mavlink::Access::Configure,
            };
            view.ui(ui, &mut cx);
        });
    harness.run();
    harness.get_by_label("IMU1");
    harness.get_by_label("IMU2");
    harness.get_by_label("DEGRADED");
    assert_eq!(harness.get_all_by_label("12345").count(), 2);
    assert_eq!(harness.get_all_by_label("21.0 s (stale)").count(), 2);
}

#[test]
fn a_tall_view_in_a_short_window_is_reached_by_scrolling_the_page() {
    // The app wraps every view that scrolls() in one vertical page scroll (Map excepted).
    for view in views::all() {
        assert_eq!(view.scrolls(), view.name() != "Map", "{}", view.name());
    }
    let mut view = views::all()
        .into_iter()
        .find(|v| v.name() == "Devices")
        .expect("Devices view is registered");
    let mut state = VehicleState::default();
    let (key, entry) = device(1, "IMU1", "OK");
    state.messages.entries.insert(key, entry);
    let seen = std::cell::RefCell::new((0.0_f32, 0.0_f32, 0.0_f32));
    let mut harness = Harness::builder()
        .with_size(eframe::egui::vec2(1280.0, 300.0))
        .build_ui(|ui| {
            let mut commands = Vec::new();
            let mut cx = ViewCtx {
                state: &state,
                commands: &mut commands,
                now: Some(Duration::from_secs(30)),
                access: gcs_mavlink::Access::Configure,
            };
            let out = eframe::egui::ScrollArea::vertical()
                .auto_shrink(false)
                .show(ui, |ui| view.ui(ui, &mut cx));
            *seen.borrow_mut() = (
                out.state.clamped_offset().y,
                out.content_size.y,
                out.inner_rect.height(),
            );
        });
    harness.run();
    let (offset, content, shown) = *seen.borrow();
    assert!(content > 2.0 * shown, "the view is taller than the window");
    assert_eq!(offset, 0.0);
    harness.hover_at(eframe::egui::pos2(600.0, 150.0));
    for _ in 0..40 {
        harness.event(eframe::egui::Event::MouseWheel {
            unit: eframe::egui::MouseWheelUnit::Point,
            delta: eframe::egui::vec2(0.0, -400.0),
            phase: eframe::egui::TouchPhase::Move,
            source: eframe::egui::MouseWheelSource::Wheel,
            modifiers: Default::default(),
        });
        harness.run();
    }
    let (offset, content, shown) = *seen.borrow();
    assert!(
        offset >= content - shown - 1.0,
        "scrolled to the end: {offset}"
    );
    harness.get_by_label("IMU1");
}
