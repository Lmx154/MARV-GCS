//! Protocol services: one sans-IO state machine per sub-protocol (layer rule 5).

use std::time::Duration;

use crate::session::SessionConfig;
use crate::{Command, Frame, Message, VehicleState};

pub mod commands;
pub mod heartbeat;
pub mod messages;
pub mod params;
pub mod texts;

/// What a service sees on each call.
pub struct Ctx<'a> {
    /// Time since the session started.
    pub now: Duration,
    /// Unix time, for the log.
    pub wall_time_us: u64,
    pub state: &'a mut VehicleState,
    /// Messages to send.
    pub out: &'a mut Vec<Message>,
    /// A service sets this when it changed `state`; the window is then woken.
    pub changed: bool,
}

pub trait Service: Send {
    fn on_frame(&mut self, _frame: &Frame, _cx: &mut Ctx) {}
    fn on_sent(&mut self, _raw: &[u8], _cx: &mut Ctx) {}
    /// At least every 50 ms.
    fn on_tick(&mut self, _cx: &mut Ctx) {}
    fn on_command(&mut self, _command: &Command, _cx: &mut Ctx) {}
}

/// The registry: the one line to touch when a service is added.
pub fn default_services(config: &SessionConfig) -> Vec<Box<dyn Service>> {
    vec![
        Box::new(heartbeat::Heartbeat::new(config)),
        Box::new(commands::Commands::new(config)),
        Box::new(messages::Messages::new()),
        Box::new(params::Params::new(config)),
        Box::new(texts::Texts::new()),
        Box::new(crate::tlog::Recorder::new()),
        // Add the next service here, one line.
    ]
}
