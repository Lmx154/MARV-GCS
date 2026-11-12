//! Link view: the state of the connection and of the vehicle's identity, as plain numbers.

use eframe::egui;

use super::{View, ViewCtx};
use crate::status;

pub struct Link;

impl View for Link {
    fn name(&self) -> &'static str {
        "Link"
    }

    fn ui(&mut self, ui: &mut egui::Ui, cx: &mut ViewCtx) {
        let link = &cx.state.link;
        let (status_text, status_colour) = match cx.now {
            Some(now) => status::describe(link, now),
            None => status::not_connected(),
        };
        let (system, component, mav_type, autopilot) = match &link.vehicle {
            Some(v) => (
                v.system_id.to_string(),
                v.component_id.to_string(),
                v.mav_type.clone(),
                v.autopilot.clone(),
            ),
            None => (dash(), dash(), dash(), dash()),
        };
        let heartbeat_age = match (cx.now, link.last_heartbeat) {
            (Some(now), Some(last)) => {
                format!("{:.1} s", now.saturating_sub(last).as_secs_f64())
            }
            _ => dash(),
        };

        egui::Grid::new("link_grid")
            .num_columns(2)
            .spacing([24.0, 6.0])
            .show(ui, |ui| {
                ui.label("Status");
                ui.colored_label(status_colour, status_text);
                ui.end_row();
                row(ui, "System", system);
                row(ui, "Component", component);
                row(ui, "Type", mav_type);
                row(ui, "Autopilot", autopilot);
                row(ui, "Last heartbeat", heartbeat_age);
                row(ui, "Received frames", link.rx.frames.to_string());
                row(ui, "Checksum errors", link.rx.crc_errors.to_string());
                row(ui, "Unknown frames", link.rx.unknown.to_string());
                row(ui, "Undecodable frames", link.rx.undecodable.to_string());
                row(ui, "Discarded bytes", link.rx.discarded_bytes.to_string());
                row(ui, "Sent frames", link.tx_frames.to_string());
            });
    }
}

fn row(ui: &mut egui::Ui, name: &str, value: String) {
    ui.label(name);
    ui.label(value);
    ui.end_row();
}

fn dash() -> String {
    "—".to_string()
}
