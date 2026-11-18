//! The window: connection bar, tab row, selected view, session and screenshot plumbing.

use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use eframe::egui;
use gcs_mavlink::tlog::ReplaySpeed;
use gcs_mavlink::{Access, Command, Session, VehicleState};
use gcs_transport::Endpoint;

use crate::cli::Cli;
use crate::connection::{self, Action, Bar};
use crate::logfile;
use crate::screenshot::{Outcome, Screenshot};
use crate::sessions::{self, Request};
use crate::status;
use crate::views::{self, View, ViewCtx};

/// While a session exists the window repaints at least this often, so ages keep moving.
const REPAINT_EVERY: Duration = Duration::from_millis(250);

pub struct App {
    bar: Bar,
    views: Vec<Box<dyn View>>,
    selected: usize,
    session: Option<Session>,
    /// The role of the session, from its link; `Operate` while there is none.
    access: Access,
    /// When the session was started: the session time is measured from here.
    session_started: Instant,
    no_session_state: VehicleState,
    screenshot: Option<Screenshot>,
    log_dir: PathBuf,
    auto_log: bool,
    /// The session replays a log: nothing is recorded.
    replaying: bool,
    /// The monitoring session of the Sensors tab's bar.
    sessions: sessions::Manager,
}

impl App {
    pub fn new(cc: &eframe::CreationContext<'_>, cli: Cli, outcome: Outcome) -> Self {
        Self::with_context(&cc.egui_ctx, cli, outcome)
    }

    /// As [`App::new`], from the egui context alone (the tests have no window).
    pub fn with_context(ctx: &egui::Context, cli: Cli, outcome: Outcome) -> Self {
        let sessions = sessions::Manager::new(ctx, &cli.log_dir);
        let mut app = App {
            bar: Bar::default(),
            views: views::all_with_tiles(),
            selected: cli.view.unwrap_or(0),
            session: None,
            access: Access::Operate,
            session_started: Instant::now(),
            no_session_state: VehicleState::default(),
            screenshot: cli.screenshot.map(|path| {
                Screenshot::new(path, Duration::from_secs_f64(cli.after_secs), outcome)
            }),
            log_dir: cli.log_dir,
            auto_log: !cli.no_log,
            replaying: false,
            sessions,
        };
        logfile::set_log_dir(ctx, &app.log_dir);
        if let Some(path) = &cli.review {
            logfile::set_review(ctx, path);
        }
        if let Some(endpoint) = cli.connect {
            app.bar.set_endpoint(&endpoint);
            app.connect(&endpoint, ctx);
        } else if let Some((path, speed)) = cli.replay {
            app.bar.set_replay(&path, speed);
            app.replay(&path, speed, ctx);
        }
        app
    }

    /// Replace how sessions are written (a test holds the export up with it).
    pub fn set_exporter(&mut self, exporter: sessions::Exporter) {
        self.sessions.set_exporter(exporter);
    }

    /// Drop the link session. A monitoring session running on it ends first: the recording is closed
    /// (dropping joins the session thread) and the export starts.
    fn retire_session(&mut self) {
        let Some(session) = self.session.take() else {
            return;
        };
        self.sessions.stop(Some(&session));
        drop(session);
        self.sessions.session_gone();
    }

    /// Why a monitoring session cannot start now, or `None`.
    fn session_blocked(&self) -> Option<String> {
        if self.session.is_none() {
            Some(status::NOT_CONNECTED.to_owned())
        } else if self.replaying {
            Some(sessions::REPLAYING.to_owned())
        } else if !self.auto_log {
            Some(sessions::RECORDING_OFF.to_owned())
        } else {
            None
        }
    }

    fn handle_request(&mut self, request: Request, ctx: &egui::Context) {
        match request {
            Request::Start(name) => {
                if self.session_blocked().is_none()
                    && !self.sessions.busy()
                    && let Some(session) = &self.session
                    && let Err(e) = self.sessions.start(session, &name)
                {
                    self.sessions.report(e);
                }
            }
            Request::Mark(note) => self.sessions.mark(&note),
            Request::Stop => self.sessions.stop(self.session.as_ref()),
            Request::Replay(path) => {
                self.bar.set_replay(&path, ReplaySpeed::RealTime);
                self.replay(&path, ReplaySpeed::RealTime, ctx);
            }
        }
    }

    /// A session started or ended: every view drops what belonged to the previous one.
    fn session_changed(&mut self, ctx: &egui::Context) {
        logfile::set_replaying(ctx, self.replaying);
        for view in &mut self.views {
            view.session_changed();
        }
    }

    fn connect(&mut self, endpoint: &Endpoint, ctx: &egui::Context) {
        self.bar.error = None;
        logfile::set_error(ctx, None);
        let access = gcs_link::access_of(endpoint);
        let repaint = ctx.clone();
        match gcs_link::start_session(
            endpoint,
            access,
            Box::new(move || repaint.request_repaint()),
        ) {
            Ok(session) => {
                self.retire_session();
                self.access = access;
                self.session_started = Instant::now();
                self.session = Some(session);
                self.replaying = false;
                self.session_changed(ctx);
                if self.auto_log {
                    self.start_recording(ctx);
                }
            }
            Err(e) => self.bar.error = Some(e.to_string()),
        }
    }

    fn replay(&mut self, path: &Path, speed: ReplaySpeed, ctx: &egui::Context) {
        self.bar.error = None;
        logfile::set_error(ctx, None);
        match connection::start_replay(path, speed, ctx) {
            Ok(session) => {
                self.retire_session();
                self.access = connection::REPLAY_ACCESS;
                self.session_started = Instant::now();
                self.session = Some(session);
                self.replaying = true;
                self.session_changed(ctx);
            }
            Err(e) => self.bar.error = Some(e),
        }
    }

    /// Record the session to a new file; on failure the reason goes to the Log view.
    fn start_recording(&mut self, ctx: &egui::Context) {
        let Some(session) = &self.session else {
            return;
        };
        let result = if self.replaying {
            Err("nothing is recorded during a replay".to_string())
        } else {
            logfile::start_command(&self.log_dir)
        };
        match result {
            Ok(command) => {
                logfile::set_error(ctx, None);
                session.send(command);
            }
            Err(e) => logfile::set_error(ctx, Some(e)),
        }
    }

    /// One frame: lock the state once, draw, release it, then act on what the user asked for.
    pub fn frame(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();
        if let Some(screenshot) = &mut self.screenshot {
            screenshot.tick(&ctx);
        }

        let mut commands: Vec<Command> = Vec::new();
        let mut action = None;
        let mut closed = None;
        {
            let guard = self.session.as_ref().map(Session::state);
            let state = guard.as_deref().unwrap_or(&self.no_session_state);
            closed.clone_from(&state.session_log.closed);
            let now = self
                .session
                .as_ref()
                .map(|_| self.session_started.elapsed());
            for view in &mut self.views {
                view.update(state, now);
            }
            let status = match now {
                Some(now) => {
                    let (text, colour) = status::describe_session(&state.link, now, self.replaying);
                    (format!("{text} — {}", status::role(self.access)), colour)
                }
                None => status::not_connected(),
            };

            egui::Panel::top("connection_bar").show(ui, |ui| {
                ui.add_space(4.0);
                action = self.bar.ui(ui, self.session.is_some(), status);
                ui.add_space(4.0);
            });
            egui::Panel::top("tabs").show(ui, |ui| {
                ui.horizontal(|ui| {
                    for (index, view) in self.views.iter().enumerate() {
                        ui.selectable_value(&mut self.selected, index, view.name());
                    }
                });
            });
            egui::Panel::left("hud")
                .exact_size(views::hud::PANEL_WIDTH)
                .resizable(false)
                .show(ui, |ui| views::hud::ui(ui, state, now));
            egui::CentralPanel::default().show(ui, |ui| {
                if let Some(view) = self.views.get_mut(self.selected) {
                    let mut cx = ViewCtx {
                        state,
                        commands: &mut commands,
                        now,
                        access: self.access,
                    };
                    view.ui(ui, &mut cx);
                }
            });
        }

        match action {
            Some(Action::Connect(endpoint)) => self.connect(&endpoint, &ctx),
            Some(Action::Replay(path, speed)) => self.replay(&path, speed, &ctx),
            Some(Action::Disconnect) => {
                self.retire_session();
                self.replaying = false;
                self.session_changed(&ctx);
            }
            None => {}
        }
        self.sessions.poll(closed.as_deref());
        for request in sessions::take_requests(&ctx) {
            self.handle_request(request, &ctx);
        }
        sessions::publish(&ctx, self.sessions.bar_status(self.session_blocked()));
        if logfile::take_request(&ctx) && self.session.is_some() {
            self.start_recording(&ctx);
        }
        if let Some(session) = &self.session {
            for command in commands {
                session.send(command);
            }
            ctx.request_repaint_after(REPAINT_EVERY);
        }
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.frame(ui);
    }

    /// The window closes: a session that runs is ended and written before the process goes. This waits for
    /// the exports (the window is gone, nothing waits on the screen); a failure goes to stderr.
    fn on_exit(&mut self, _gl: Option<&eframe::glow::Context>) {
        self.retire_session();
        self.sessions.finish();
    }
}
