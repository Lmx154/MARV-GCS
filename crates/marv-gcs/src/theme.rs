//! The look of the window: one dark "ground station" palette, the egui style made from it, and the
//! small drawing helpers the views share (cards, section titles, status dots and pills, segmented
//! controls). Applied once at startup ([`apply`]); the bundled egui fonts are kept.

use eframe::egui::{
    self, Color32, CornerRadius, FontFamily, FontId, Margin, Response, RichText, Shadow, Stroke,
    TextStyle, Theme, ThemePreference, Ui, Vec2,
};

// Background layers, darkest to lightest.
/// Inputs, plot backgrounds and the inside of segmented controls.
pub const INSET: Color32 = Color32::from_rgb(11, 14, 19);
/// The central area behind the views.
pub const WINDOW: Color32 = Color32::from_rgb(15, 19, 25);
/// The connection bar, the tab row and the side panel.
pub const PANEL: Color32 = Color32::from_rgb(21, 26, 33);
/// Cards and sections.
pub const CARD: Color32 = Color32::from_rgb(26, 32, 41);
/// A widget at rest (buttons, combo boxes).
pub const WIDGET: Color32 = Color32::from_rgb(36, 44, 56);
const WIDGET_HOVER: Color32 = Color32::from_rgb(46, 56, 71);
const WIDGET_ACTIVE: Color32 = Color32::from_rgb(56, 68, 86);

/// Card and panel outlines, separators.
pub const BORDER: Color32 = Color32::from_rgb(41, 49, 61);
/// Outline of a hovered widget.
pub const BORDER_STRONG: Color32 = Color32::from_rgb(66, 78, 96);

// Text.
pub const TEXT: Color32 = Color32::from_rgb(214, 221, 230);
pub const TEXT_STRONG: Color32 = Color32::from_rgb(242, 246, 250);
pub const TEXT_SECONDARY: Color32 = Color32::from_rgb(150, 161, 176);
pub const TEXT_MUTED: Color32 = Color32::from_rgb(105, 116, 132);

/// The one accent: selection, the selected tab, links, focus.
pub const ACCENT: Color32 = Color32::from_rgb(64, 168, 245);
/// The accent as a fill behind selected items.
pub const ACCENT_FILL: Color32 = Color32::from_rgb(22, 74, 115);

// Status.
pub const OK: Color32 = Color32::from_rgb(72, 204, 120);
pub const WARN: Color32 = Color32::from_rgb(240, 184, 64);
pub const ERROR: Color32 = Color32::from_rgb(239, 90, 90);
pub const STALE: Color32 = Color32::from_rgb(140, 150, 164);

/// Line colours of the plots, in order: the first three are the x, y and z of a sensor.
pub const SERIES: [Color32; 8] = [
    Color32::from_rgb(244, 114, 114),
    Color32::from_rgb(86, 214, 136),
    Color32::from_rgb(96, 165, 250),
    Color32::from_rgb(250, 204, 21),
    Color32::from_rgb(192, 132, 252),
    Color32::from_rgb(45, 212, 191),
    Color32::from_rgb(251, 146, 60),
    Color32::from_rgb(244, 114, 182),
];

/// Corner radius of widgets; cards use [`CARD_RADIUS`].
pub const RADIUS: u8 = 6;
pub const CARD_RADIUS: u8 = 8;
/// Space inside a card.
pub const CARD_MARGIN: i8 = 12;
/// Space between cards.
pub const GAP: f32 = 10.0;

/// The colour of the `index`-th line of a plot.
pub fn series(index: usize) -> Color32 {
    SERIES[index % SERIES.len()]
}

/// Make the window dark and styled, whatever the system theme. Called once with the context.
pub fn apply(ctx: &egui::Context) {
    ctx.set_theme(ThemePreference::Dark);
    ctx.set_style_of(Theme::Dark, style());
}

/// The whole egui style of the window.
pub fn style() -> egui::Style {
    let mut style = egui::Style {
        visuals: visuals(),
        ..egui::Style::default()
    };
    style.text_styles = [
        (
            TextStyle::Small,
            FontId::new(11.5, FontFamily::Proportional),
        ),
        (TextStyle::Body, FontId::new(14.0, FontFamily::Proportional)),
        (
            TextStyle::Button,
            FontId::new(14.0, FontFamily::Proportional),
        ),
        (
            TextStyle::Heading,
            FontId::new(19.0, FontFamily::Proportional),
        ),
        (
            TextStyle::Monospace,
            FontId::new(13.0, FontFamily::Monospace),
        ),
    ]
    .into();
    let spacing = &mut style.spacing;
    spacing.item_spacing = Vec2::new(8.0, 6.0);
    spacing.button_padding = Vec2::new(10.0, 4.0);
    spacing.interact_size = Vec2::new(40.0, 24.0);
    spacing.window_margin = Margin::same(12);
    spacing.menu_margin = Margin::same(6);
    spacing.combo_height = 300.0;
    spacing.icon_width = 16.0;
    spacing.icon_width_inner = 9.0;
    spacing.icon_spacing = 6.0;
    style
}

fn visuals() -> egui::Visuals {
    let mut v = egui::Visuals::dark();
    let radius = CornerRadius::same(RADIUS);
    let w = &mut v.widgets;
    w.noninteractive.bg_fill = CARD;
    w.noninteractive.weak_bg_fill = CARD;
    w.noninteractive.bg_stroke = Stroke::new(1.0, BORDER);
    w.noninteractive.fg_stroke = Stroke::new(1.0, TEXT);
    w.noninteractive.corner_radius = radius;

    w.inactive.bg_fill = WIDGET;
    w.inactive.weak_bg_fill = WIDGET;
    w.inactive.bg_stroke = Stroke::new(1.0, BORDER);
    w.inactive.fg_stroke = Stroke::new(1.0, TEXT);
    w.inactive.corner_radius = radius;

    w.hovered.bg_fill = WIDGET_HOVER;
    w.hovered.weak_bg_fill = WIDGET_HOVER;
    w.hovered.bg_stroke = Stroke::new(1.0, BORDER_STRONG);
    w.hovered.fg_stroke = Stroke::new(1.5, TEXT_STRONG);
    w.hovered.corner_radius = radius;
    w.hovered.expansion = 0.0;

    w.active.bg_fill = WIDGET_ACTIVE;
    w.active.weak_bg_fill = WIDGET_ACTIVE;
    w.active.bg_stroke = Stroke::new(1.0, ACCENT);
    w.active.fg_stroke = Stroke::new(2.0, TEXT_STRONG);
    w.active.corner_radius = radius;
    w.active.expansion = 0.0;

    w.open.bg_fill = WIDGET;
    w.open.weak_bg_fill = WIDGET;
    w.open.bg_stroke = Stroke::new(1.0, BORDER_STRONG);
    w.open.fg_stroke = Stroke::new(1.0, TEXT_STRONG);
    w.open.corner_radius = radius;

    v.selection.bg_fill = ACCENT_FILL;
    v.selection.stroke = Stroke::new(1.0, TEXT_STRONG);
    v.hyperlink_color = ACCENT;
    v.faint_bg_color = Color32::from_rgb(20, 25, 32);
    v.extreme_bg_color = INSET;
    v.text_edit_bg_color = Some(INSET);
    v.code_bg_color = WIDGET;
    v.warn_fg_color = WARN;
    v.error_fg_color = ERROR;
    v.window_corner_radius = CornerRadius::same(CARD_RADIUS);
    v.menu_corner_radius = CornerRadius::same(CARD_RADIUS);
    v.window_fill = CARD;
    v.window_stroke = Stroke::new(1.0, BORDER_STRONG);
    v.window_shadow = Shadow {
        offset: [0, 8],
        blur: 20,
        spread: 0,
        color: Color32::from_black_alpha(110),
    };
    v.popup_shadow = Shadow {
        offset: [0, 4],
        blur: 12,
        spread: 0,
        color: Color32::from_black_alpha(100),
    };
    v.panel_fill = WINDOW;
    v.text_cursor.stroke = Stroke::new(2.0, ACCENT);
    v.slider_trailing_fill = true;
    v
}

/// A card: the raised surface the views put their sections on.
pub fn card() -> egui::Frame {
    egui::Frame::new()
        .fill(CARD)
        .stroke(Stroke::new(1.0, BORDER))
        .corner_radius(CornerRadius::same(CARD_RADIUS))
        .inner_margin(Margin::same(CARD_MARGIN))
}

/// A full-width card with a title over its contents.
pub fn section<R>(ui: &mut Ui, title: &str, add: impl FnOnce(&mut Ui) -> R) -> R {
    card()
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            section_title(ui, title);
            add(ui)
        })
        .inner
}

/// The title of a section: strong, slightly larger, with a little space under it.
pub fn section_title(ui: &mut Ui, title: &str) -> Response {
    let response = ui.label(title_text(title));
    ui.add_space(2.0);
    response
}

/// The words of a section title.
pub fn title_text(title: &str) -> RichText {
    RichText::new(title).size(15.0).strong().color(TEXT_STRONG)
}

/// A number or telemetry value: monospace, so columns of figures line up.
pub fn value(text: impl Into<String>) -> RichText {
    RichText::new(text).monospace()
}

/// Secondary words (names next to values, hints).
pub fn muted(text: impl Into<String>) -> RichText {
    RichText::new(text).color(TEXT_SECONDARY)
}

/// A small filled circle in `colour`, on the line of the text around it.
pub fn status_dot(ui: &mut Ui, colour: Color32) -> Response {
    let size = Vec2::splat(ui.text_style_height(&TextStyle::Body).min(16.0));
    let (rect, response) = ui.allocate_exact_size(size, egui::Sense::hover());
    let radius = 4.0;
    ui.painter()
        .circle_filled(rect.center(), radius + 2.5, colour.gamma_multiply(0.25));
    ui.painter().circle_filled(rect.center(), radius, colour);
    response
}

/// A rounded badge: a dot and `text` on a tint of `colour`. The text is cut with an ellipsis to fit
/// `max_width` (the whole text is then the hover text). Returns the label's response.
pub fn pill(ui: &mut Ui, text: &str, colour: Color32, max_width: f32) -> Response {
    egui::Frame::new()
        .fill(colour.gamma_multiply(0.14))
        .stroke(Stroke::new(1.0, colour.gamma_multiply(0.45)))
        .corner_radius(CornerRadius::same(12))
        .inner_margin(Margin::symmetric(10, 3))
        .show(ui, |ui| {
            ui.set_max_width((max_width - 22.0).max(0.0));
            ui.spacing_mut().item_spacing.x = 6.0;
            ui.horizontal(|ui| {
                status_dot(ui, colour);
                ui.add(egui::Label::new(RichText::new(text).color(colour)).truncate())
                    .on_hover_text(text)
            })
            .inner
        })
        .inner
}

/// A group of mutually exclusive choices drawn as one segmented control. Each choice is an ordinary
/// selectable button, so it keeps its name for screen readers and tests.
pub fn segmented<T: PartialEq + Copy>(ui: &mut Ui, current: &mut T, choices: &[(T, &str)]) {
    egui::Frame::new()
        .fill(INSET)
        .stroke(Stroke::new(1.0, BORDER))
        .corner_radius(CornerRadius::same(RADIUS + 1))
        .inner_margin(Margin::same(2))
        .show(ui, |ui| {
            ui.spacing_mut().item_spacing.x = 2.0;
            ui.spacing_mut().button_padding.y = 2.0;
            ui.horizontal(|ui| {
                for (value, label) in choices {
                    ui.selectable_value(current, *value, *label);
                }
            });
        });
}

/// A tab of the tab row: a selectable button with a coloured underline when it is the selected one.
pub fn tab(ui: &mut Ui, selected: bool, label: &str) -> Response {
    let text = if selected {
        RichText::new(label).color(TEXT_STRONG).strong()
    } else {
        RichText::new(label).color(TEXT_SECONDARY)
    };
    let response = ui.add(
        egui::Button::selectable(selected, text)
            .min_size(Vec2::new(0.0, 30.0))
            .corner_radius(CornerRadius {
                nw: RADIUS,
                ne: RADIUS,
                sw: 0,
                se: 0,
            }),
    );
    if selected {
        let rect = response.rect;
        let line = egui::Rect::from_min_max(
            egui::pos2(rect.left() + 2.0, rect.bottom() - 2.5),
            egui::pos2(rect.right() - 2.0, rect.bottom()),
        );
        ui.painter().rect_filled(line, 1.0, ACCENT);
    }
    response
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_theme_is_dark_with_larger_text_whatever_the_system_says() {
        let ctx = egui::Context::default();
        apply(&ctx);
        let style = ctx.style_of(Theme::Dark);
        assert_eq!(ctx.options(|o| o.theme_preference), ThemePreference::Dark);
        assert_eq!(style.visuals.panel_fill, WINDOW);
        assert_eq!(style.visuals.selection.bg_fill, ACCENT_FILL);
        assert_eq!(style.text_styles[&TextStyle::Body].size, 14.0);
        assert_eq!(
            style.text_styles[&TextStyle::Monospace].family,
            FontFamily::Monospace
        );
    }

    #[test]
    fn plot_lines_cycle_through_the_palette() {
        assert_eq!(series(0), SERIES[0]);
        assert_eq!(series(SERIES.len() + 2), SERIES[2]);
    }
}
