//! The connection bar: transport choice, inputs, Connect / Disconnect, status and the last error.

use std::fs::File;
use std::path::{Path, PathBuf};

use eframe::egui::{self, Color32};
use gcs_mavlink::tlog::{ReplaySpeed, replay};
use gcs_mavlink::{Session, SessionConfig};
use gcs_transport::{DEFAULT_BAUD, Endpoint, OpenError, PortInfo};

use crate::status;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Transport {
    Serial,
    Tcp,
    Udp,
    LogFile,
}

/// What the user asked for with the buttons; the app carries it out.
#[derive(Debug, Clone, PartialEq)]
pub enum Action {
    Connect(Endpoint),
    /// Replay a `.tlog`; nothing is recorded.
    Replay(PathBuf, ReplaySpeed),
    Disconnect,
}

pub struct Bar {
    transport: Transport,
    ports: Vec<PortInfo>,
    all_ports: bool,
    port: Option<String>,
    baud: String,
    tcp_addr: String,
    udp_addr: String,
    log_path: String,
    replay_speed: ReplaySpeed,
    /// Shown in red until the next attempt.
    pub error: Option<String>,
}

impl Default for Bar {
    fn default() -> Self {
        Bar {
            transport: Transport::Serial,
            ports: gcs_transport::list_serial_ports(),
            all_ports: false,
            port: None,
            baud: DEFAULT_BAUD.to_string(),
            tcp_addr: "127.0.0.1:5760".to_string(),
            udp_addr: "0.0.0.0:14550".to_string(),
            log_path: String::new(),
            replay_speed: ReplaySpeed::RealTime,
            error: None,
        }
    }
}

impl Bar {
    /// Show an endpoint (from the command line) in the inputs.
    pub fn set_endpoint(&mut self, endpoint: &Endpoint) {
        match endpoint {
            Endpoint::Serial { path, baud } => {
                self.transport = Transport::Serial;
                self.port = Some(path.clone());
                self.baud = baud.to_string();
                if !self.ports.iter().any(|p| &p.path == path) {
                    self.ports.push(PortInfo {
                        path: path.clone(),
                        usb: None,
                    });
                }
            }
            Endpoint::Tcp { addr } => {
                self.transport = Transport::Tcp;
                self.tcp_addr = addr.clone();
            }
            Endpoint::Udp { bind } => {
                self.transport = Transport::Udp;
                self.udp_addr = bind.to_string();
            }
        }
    }

    /// Show a log file to replay (from the command line) in the inputs.
    pub fn set_replay(&mut self, path: &Path, speed: ReplaySpeed) {
        self.transport = Transport::LogFile;
        self.log_path = path.display().to_string();
        self.replay_speed = speed;
    }

    /// What the inputs describe.
    fn action(&self) -> Result<Action, String> {
        match self.transport {
            Transport::LogFile => {
                let path = self.log_path.trim();
                if path.is_empty() {
                    return Err("missing log file path".to_string());
                }
                Ok(Action::Replay(PathBuf::from(path), self.replay_speed))
            }
            _ => self.endpoint().map(Action::Connect),
        }
    }

    /// The endpoint the inputs describe (serial, TCP or UDP).
    fn endpoint(&self) -> Result<Endpoint, String> {
        match self.transport {
            Transport::LogFile => Err("no endpoint for a log file".to_string()),
            Transport::Serial => {
                let path = self.port.clone().ok_or("no serial port selected")?;
                let baud = self
                    .baud
                    .trim()
                    .parse::<u32>()
                    .ok()
                    .filter(|b| *b > 0)
                    .ok_or("invalid baud rate")?;
                Ok(Endpoint::Serial { path, baud })
            }
            Transport::Tcp => {
                let addr = self.tcp_addr.trim();
                if addr.is_empty() {
                    return Err("missing TCP address".to_string());
                }
                Ok(Endpoint::Tcp {
                    addr: addr.to_string(),
                })
            }
            Transport::Udp => {
                let addr = self.udp_addr.trim();
                if addr.is_empty() {
                    return Err("missing UDP address".to_string());
                }
                format!("udp:{addr}")
                    .parse::<Endpoint>()
                    .map_err(|_| format!("invalid UDP address {addr:?}: expected ADDR:PORT"))
            }
        }
    }

    /// Ports offered in the combo box: USB only unless "all ports" is ticked.
    fn shown_ports(&self) -> impl Iterator<Item = &PortInfo> {
        self.ports
            .iter()
            .filter(|p| self.all_ports || p.usb.is_some())
    }

    /// Draw the bar. `connected` is true while a session exists.
    pub fn ui(
        &mut self,
        ui: &mut egui::Ui,
        connected: bool,
        status: (String, Color32),
    ) -> Option<Action> {
        let mut action = None;
        ui.horizontal(|ui| {
            ui.add_enabled_ui(!connected, |ui| {
                ui.selectable_value(&mut self.transport, Transport::Serial, "Serial");
                ui.selectable_value(&mut self.transport, Transport::Tcp, "TCP");
                ui.selectable_value(&mut self.transport, Transport::Udp, "UDP");
                ui.selectable_value(&mut self.transport, Transport::LogFile, "Log file");
                ui.separator();
                match self.transport {
                    Transport::Serial => self.serial_inputs(ui),
                    Transport::Tcp => {
                        ui.label("Address");
                        ui.add(egui::TextEdit::singleline(&mut self.tcp_addr).desired_width(150.0));
                    }
                    Transport::Udp => {
                        let label = ui.label("Listen on");
                        ui.add(egui::TextEdit::singleline(&mut self.udp_addr).desired_width(150.0))
                            .labelled_by(label.id);
                    }
                    Transport::LogFile => {
                        let label = ui.label("Path");
                        ui.add(egui::TextEdit::singleline(&mut self.log_path).desired_width(300.0))
                            .labelled_by(label.id);
                        ui.selectable_value(
                            &mut self.replay_speed,
                            ReplaySpeed::RealTime,
                            "real time",
                        );
                        ui.selectable_value(&mut self.replay_speed, ReplaySpeed::Fast, "fast");
                    }
                }
            });
            ui.separator();
            if connected {
                if ui.button("Disconnect").clicked() {
                    action = Some(Action::Disconnect);
                }
            } else if ui.button("Connect").clicked() {
                self.error = None;
                match self.action() {
                    Ok(a) => action = Some(a),
                    Err(e) => self.error = Some(e),
                }
            }
            ui.separator();
            let rest = ui.available_width();
            let share = if self.error.is_some() {
                rest / 2.0
            } else {
                rest
            };
            truncated(ui, share, &status.0, status.1);
            if let Some(error) = &self.error {
                truncated(ui, ui.available_width(), error, status::RED);
            }
        });
        action
    }

    fn serial_inputs(&mut self, ui: &mut egui::Ui) {
        let labels: Vec<(String, String)> = self
            .shown_ports()
            .map(|p| (p.path.clone(), port_label(p)))
            .collect();
        if !labels
            .iter()
            .any(|(path, _)| Some(path) == self.port.as_ref())
        {
            self.port = labels.first().map(|(path, _)| path.clone());
        }
        let selected_text = labels
            .iter()
            .find(|(path, _)| Some(path) == self.port.as_ref())
            .map_or("no port", |(_, label)| label.as_str())
            .to_string();
        let port_label_widget = ui.label("Port");
        egui::ComboBox::from_id_salt("serial_port")
            .selected_text(selected_text)
            .width(320.0)
            .show_ui(ui, |ui| {
                for (path, label) in &labels {
                    ui.selectable_value(&mut self.port, Some(path.clone()), label);
                }
            })
            .response
            .labelled_by(port_label_widget.id);
        if ui.button("Refresh").clicked() {
            self.ports = gcs_transport::list_serial_ports();
        }
        ui.checkbox(&mut self.all_ports, "all ports");
        let baud_label = ui.label("Baud");
        ui.add(egui::TextEdit::singleline(&mut self.baud).desired_width(70.0))
            .labelled_by(baud_label.id);
    }
}

/// A one-line label at most `width` wide: longer text ends in an ellipsis and is the hover text.
fn truncated(ui: &mut egui::Ui, width: f32, text: &str, colour: Color32) {
    ui.scope(|ui| {
        ui.set_max_width(width.max(0.0));
        ui.add(egui::Label::new(egui::RichText::new(text).color(colour)).truncate())
            .on_hover_text(text);
    });
}

/// `<path> — <product> (<vid>:<pid>)`, the path alone for a port that is not USB.
pub fn port_label(port: &PortInfo) -> String {
    match &port.usb {
        None => port.path.clone(),
        Some(usb) => {
            let ids = format!("({:04x}:{:04x})", usb.vid, usb.pid);
            match &usb.product {
                Some(product) => format!("{} — {product} {ids}", port.path),
                None => format!("{} — {ids}", port.path),
            }
        }
    }
}

/// Open the endpoint and start a session on it; `wake` repaints the window.
pub fn start_session(endpoint: &Endpoint, ctx: &egui::Context) -> Result<Session, OpenError> {
    let link = gcs_transport::open(endpoint)?;
    Ok(start(link.reader, link.writer, ctx))
}

/// Open a `.tlog` and start a session that replays it.
pub fn start_replay(
    path: &Path,
    speed: ReplaySpeed,
    ctx: &egui::Context,
) -> Result<Session, String> {
    let file = File::open(path).map_err(|e| format!("cannot open {}: {e}", path.display()))?;
    let (reader, writer) = replay(Box::new(file), speed);
    Ok(start(reader, writer, ctx))
}

fn start(
    reader: Box<dyn std::io::Read + Send>,
    writer: Box<dyn std::io::Write + Send>,
    ctx: &egui::Context,
) -> Session {
    let ctx = ctx.clone();
    Session::start(
        reader,
        writer,
        SessionConfig::default(),
        Box::new(move || ctx.request_repaint()),
    )
}
