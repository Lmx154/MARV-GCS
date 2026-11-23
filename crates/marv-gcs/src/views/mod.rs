//! The views (tabs). A view is one file in this directory and one line in [`all`].

use std::time::Duration;

use eframe::egui;
use gcs_mavlink::{Access, Command, VehicleState};

mod actions;
mod compass;
mod compass_quad;
mod devices;
pub mod hud;
mod link;
mod log;
mod log_review;
mod map;
mod messages;
mod motors;
mod params;
mod plot;
pub mod presets;
pub mod radio;
pub mod sensors;
mod sidebar;

pub struct ViewCtx<'a> {
    /// Locked for the frame.
    pub state: &'a VehicleState,
    /// Sent after the frame.
    pub commands: &'a mut Vec<Command>,
    /// Time since the session started; `None` when there is no session.
    pub now: Option<Duration>,
    /// What the session may do; configure controls are shown disabled on `Operate`.
    pub access: Access,
}

pub trait View {
    /// Tab title.
    fn name(&self) -> &'static str;
    fn ui(&mut self, ui: &mut egui::Ui, cx: &mut ViewCtx);
    /// Whether the app wraps the view in one vertical page scroll; false for a view that fills the window.
    fn scrolls(&self) -> bool {
        true
    }
    /// A session started or ended: drop whatever belongs to the previous vehicle.
    fn session_changed(&mut self) {}
    /// Called every frame for every view, selected or not, with the state of the frame: a view that
    /// keeps a history of its own (the Map's track) takes in what arrived while it was hidden.
    fn update(&mut self, _state: &VehicleState, _now: Option<Duration>) {}
}

/// The registry for the window: the Map fetches OpenFreeMap vector tiles over HTTPS.
pub fn all_with_tiles() -> Vec<Box<dyn View>> {
    registry(true)
}

/// The registry without any network access (the Map has no tiles): for the tests.
pub fn all() -> Vec<Box<dyn View>> {
    registry(false)
}

/// The Map tab with no tiles and its waypoint lists in `store` (`None`: in memory only): for the tests, which need
/// lists on disk of their own.
pub fn map_with_store(store: Option<crate::missions::Store>) -> Box<dyn View> {
    Box::new(map::MapView::with_store(false, store))
}

/// In tab order: the one line to touch when a view is added.
fn registry(tiles: bool) -> Vec<Box<dyn View>> {
    let debug = presets::DebugFlag::default();
    vec![
        Box::<sensors::Sensors>::default(),
        Box::<radio::Radio>::default(),
        Box::<messages::Messages>::default(),
        Box::new(devices::Devices::new(debug.clone())),
        Box::new(params::Params::new(debug)),
        Box::<log::Log>::default(),
        Box::<link::Link>::default(),
        Box::new(map::MapView::new(tiles)),
        Box::<motors::Motors>::default(),
        // Box::new(<module>::<View>),
    ]
}
