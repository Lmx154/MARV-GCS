//! The views (tabs). A view is one file in this directory and one line in [`all`].

use std::time::Duration;

use eframe::egui;
use gcs_mavlink::{Access, Command, VehicleState};

mod devices;
mod link;
mod log;
mod messages;
mod params;
mod plot;
mod sensors;

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
    /// A session started or ended: drop whatever belongs to the previous vehicle.
    fn session_changed(&mut self) {}
}

/// The registry, in tab order: the one line to touch when a view is added.
pub fn all() -> Vec<Box<dyn View>> {
    vec![
        Box::new(sensors::Sensors),
        Box::<messages::Messages>::default(),
        Box::<devices::Devices>::default(),
        Box::<params::Params>::default(),
        Box::<log::Log>::default(),
        Box::new(link::Link),
        // Box::new(<module>::<View>),
    ]
}
