//! Messages view: every message that arrives, its fields, and a plot of any numeric field.

use eframe::egui;
use egui_extras::{Column, TableBuilder};
use gcs_mavlink::services::messages::{
    Field, FieldValue, MessageEntry, MessageKey, MessageStore, SeriesKey,
};

use super::plot;
use super::{View, ViewCtx};

const ROW_HEIGHT: f32 = 20.0;
const TABLE_HEIGHT: f32 = 220.0;
const FIELDS_WIDTH: f32 = 300.0;

#[derive(Default)]
pub struct Messages {
    choice: Choice,
}

/// What the user chose.
#[derive(Default)]
struct Choice {
    selected: Option<MessageKey>,
    plotted: Vec<(MessageKey, String)>,
}

impl View for Messages {
    fn name(&self) -> &'static str {
        "Messages"
    }

    fn session_changed(&mut self) {
        self.choice = Choice::default();
    }

    fn ui(&mut self, ui: &mut egui::Ui, cx: &mut ViewCtx) {
        let store = &cx.state.messages;
        let choice = &mut self.choice;

        ui.label(format!("Lost frames: {}", store.lost_frames));
        if store.entries.is_empty() {
            ui.label("No messages yet");
        } else {
            table(ui, cx, choice);
        }
        ui.separator();
        let window = plot::window_selector(ui, "messages");
        let height = ui.available_height().max(120.0);
        ui.horizontal_top(|ui| {
            ui.allocate_ui(egui::vec2(FIELDS_WIDTH, height), |ui| {
                fields(ui, store, choice);
            });
            ui.vertical(|ui| chosen_plot(ui, store, choice, window, height));
        });
    }
}

fn table(ui: &mut egui::Ui, cx: &ViewCtx, choice: &mut Choice) {
    let mut clicked = None;
    TableBuilder::new(ui)
        .id_salt("messages_table")
        .striped(true)
        .sense(egui::Sense::click())
        .max_scroll_height(TABLE_HEIGHT)
        .column(Column::auto().at_least(180.0))
        .column(Column::auto().at_least(70.0))
        .column(Column::auto().at_least(70.0))
        .column(Column::auto().at_least(80.0))
        .column(Column::remainder())
        .header(ROW_HEIGHT, |mut header| {
            for title in ["Message", "Sender", "Count", "Rate", "Age"] {
                header.col(|ui| {
                    ui.strong(title);
                });
            }
        })
        .body(|body| {
            let entries: Vec<_> = cx.state.messages.entries.iter().collect();
            body.rows(ROW_HEIGHT, entries.len(), |mut row| {
                let (key, entry) = entries[row.index()];
                row.set_selected(choice.selected.as_ref() == Some(key));
                let age = match cx.now {
                    Some(now) => {
                        format!("{:.1} s", now.saturating_sub(entry.last_seen).as_secs_f64())
                    }
                    None => "—".to_string(),
                };
                let cells = [
                    entry.label.clone(),
                    format!("{}/{}", key.system_id, key.component_id),
                    entry.count.to_string(),
                    format!("{:.1} Hz", entry.rate_hz),
                    age,
                ];
                for text in cells {
                    let mut hit = false;
                    row.col(|ui| {
                        hit = ui
                            .add(egui::Label::new(text).sense(egui::Sense::click()))
                            .clicked();
                    });
                    if hit {
                        clicked = Some(key.clone());
                    }
                }
                if row.response().clicked() {
                    clicked = Some(key.clone());
                }
            });
        });
    if clicked.is_some() {
        choice.selected = clicked;
    }
}

fn fields(ui: &mut egui::Ui, store: &MessageStore, choice: &mut Choice) {
    let Some((key, entry)) = choice
        .selected
        .clone()
        .and_then(|key| store.entries.get(&key).map(|entry| (key, entry)))
    else {
        ui.label("Click a message to see its fields.");
        return;
    };
    ui.strong(&entry.label);
    egui::ScrollArea::vertical()
        .id_salt("messages_fields")
        .show(ui, |ui| {
            egui::Grid::new("messages_fields_grid")
                .num_columns(2)
                .spacing([16.0, 4.0])
                .show(ui, |ui| {
                    for field in &entry.fields {
                        field_row(ui, &key, field, choice);
                        ui.end_row();
                    }
                });
        });
}

fn field_row(ui: &mut egui::Ui, key: &MessageKey, field: &Field, choice: &mut Choice) {
    match &field.value {
        FieldValue::Number(n) => {
            let position = choice
                .plotted
                .iter()
                .position(|(k, name)| k == key && *name == field.name);
            let mut on = position.is_some();
            if ui.checkbox(&mut on, &field.name).changed() {
                match position {
                    Some(i) => {
                        choice.plotted.remove(i);
                    }
                    None => choice.plotted.push((key.clone(), field.name.clone())),
                }
            }
            ui.label(format_number(*n));
        }
        FieldValue::Text(text) => {
            ui.label(&field.name);
            ui.label(text);
        }
    }
}

fn chosen_plot(
    ui: &mut egui::Ui,
    store: &MessageStore,
    choice: &mut Choice,
    window: f64,
    height: f32,
) {
    if ui
        .add_enabled(!choice.plotted.is_empty(), egui::Button::new("Clear plot"))
        .clicked()
    {
        choice.plotted.clear();
    }
    let traces: Vec<_> = choice
        .plotted
        .iter()
        .filter_map(|(key, field)| {
            let entry = store.entries.get(key)?;
            let series = store.series.get(&series_key(key, entry, field))?;
            Some((format!("{}.{field}", entry.label), series))
        })
        .collect();
    if choice.plotted.is_empty() {
        ui.label("Tick a numeric field to plot it.");
    } else {
        let plot_height = (height - ui.min_rect().height()).max(100.0);
        plot::draw(ui, "messages_plot", plot_height, window, &traces);
    }
}

fn series_key(key: &MessageKey, entry: &MessageEntry, field: &str) -> SeriesKey {
    SeriesKey {
        system_id: key.system_id,
        component_id: key.component_id,
        message: entry.name.clone(),
        instance: key.instance,
        field: field.to_string(),
    }
}

/// Up to 6 significant digits, no trailing zeros; `NaN` for NaN.
fn format_number(n: f64) -> String {
    if n.is_nan() {
        return "NaN".to_string();
    }
    if n.is_infinite() {
        return if n > 0.0 { "inf" } else { "-inf" }.to_string();
    }
    if n == 0.0 {
        return "0".to_string();
    }
    let exponent = n.abs().log10().floor() as i32;
    if (-5..6).contains(&exponent) {
        let decimals = (5 - exponent).max(0) as usize;
        trim_zeros(format!("{n:.decimals$}"))
    } else {
        let text = format!("{n:.5e}");
        match text.split_once('e') {
            Some((mantissa, exp)) => format!("{}e{exp}", trim_zeros(mantissa.to_string())),
            None => text,
        }
    }
}

fn trim_zeros(text: String) -> String {
    if text.contains('.') {
        text.trim_end_matches('0').trim_end_matches('.').to_string()
    } else {
        text
    }
}

#[cfg(test)]
mod tests {
    use super::format_number;

    #[test]
    fn numbers_have_six_significant_digits() {
        assert_eq!(format_number(f64::NAN), "NaN");
        assert_eq!(format_number(0.0), "0");
        assert_eq!(format_number(9.81), "9.81");
        assert_eq!(format_number(1.0 / 3.0), "0.333333");
        assert_eq!(format_number(123456.789), "123457");
        assert_eq!(format_number(-0.000123456789), "-0.000123457");
        assert_eq!(format_number(1.5e9), "1.5e9");
        assert_eq!(format_number(2.5e-7), "2.5e-7");
    }
}
