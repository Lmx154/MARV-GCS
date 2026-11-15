//! The session: owns the threads and the clock, runs the services on a byte link.

use std::any::Any;
use std::io::{ErrorKind, Read, Write};
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::thread::JoinHandle;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use crate::services::{Ctx, Service, default_services};
use crate::{
    Access, Command, Deframer, Encoder, Frame, LinkStatus, Refused, RxStats, VehicleState,
};

/// The services are ticked at least this often (the contract says every 50 ms).
const TICK: Duration = Duration::from_millis(20);
/// Minimum time between two `wake` calls (about 30 a second).
const WAKE_INTERVAL: Duration = Duration::from_millis(34);
const READ_BUFFER: usize = 4096;

#[derive(Debug, Clone)]
pub struct SessionConfig {
    pub system_id: u8,
    pub component_id: u8,
    pub heartbeat_timeout: Duration,
    /// What the session may do; `Session::send` refuses what needs more.
    pub access: Access,
}

impl Default for SessionConfig {
    fn default() -> Self {
        Self {
            system_id: 255,
            component_id: 190,
            heartbeat_timeout: Duration::from_secs(5),
            access: Access::Configure,
        }
    }
}

enum Input {
    /// What the reader thread got from one `read`: frames, and the deframer statistics after it.
    Received(Vec<Frame>, RxStats),
    /// The reader thread ended because the link is down.
    LinkDown(String),
    Command(Command),
}

pub struct Session {
    state: Arc<Mutex<VehicleState>>,
    stop: Arc<AtomicBool>,
    input: Sender<Input>,
    access: Access,
    threads: Vec<JoinHandle<()>>,
}

impl Session {
    /// Starts the threads. `wake` is called (at most about 30 times a second) when the state changed.
    pub fn start(
        reader: Box<dyn Read + Send>,
        writer: Box<dyn Write + Send>,
        config: SessionConfig,
        wake: Box<dyn Fn() + Send + Sync>,
    ) -> Session {
        let services = default_services(&config);
        Self::start_with_services(reader, writer, config, wake, services)
    }

    pub(crate) fn start_with_services(
        reader: Box<dyn Read + Send>,
        writer: Box<dyn Write + Send>,
        config: SessionConfig,
        wake: Box<dyn Fn() + Send + Sync>,
        services: Vec<Box<dyn Service>>,
    ) -> Session {
        let state = Arc::new(Mutex::new(VehicleState::default()));
        let stop = Arc::new(AtomicBool::new(false));
        let access = config.access;
        let (input, inbox) = mpsc::channel();

        let reader_thread = {
            let input = input.clone();
            let stop = Arc::clone(&stop);
            std::thread::Builder::new()
                .name("gcs-reader".into())
                .spawn(move || read_loop(reader, &input, &stop))
                .expect("spawn gcs-reader thread")
        };
        let session_thread = {
            let worker = Worker {
                state: Arc::clone(&state),
                stop: Arc::clone(&stop),
                services,
                encoder: Encoder::new(config.system_id, config.component_id),
                writer,
                out: Vec::new(),
                changed: false,
                started: Instant::now(),
                wake,
            };
            std::thread::Builder::new()
                .name("gcs-session".into())
                .spawn(move || worker.run_guarded(&inbox))
                .expect("spawn gcs-session thread")
        };

        Session {
            state,
            stop,
            input,
            access,
            threads: vec![reader_thread, session_thread],
        }
    }

    pub fn state(&self) -> MutexGuard<'_, VehicleState> {
        lock(&self.state)
    }

    /// The one choke point for what the application asks of the session. A command that needs more
    /// access than the session has is refused: nothing is written and `link.refused` records it.
    pub fn send(&self, command: Command) {
        if !self.access.allows(command.access()) {
            let mut state = lock(&self.state);
            let count = state.link.refused.as_ref().map_or(0, |r| r.count);
            state.link.refused = Some(Refused {
                command: command.name(),
                count: count + 1,
            });
            return;
        }
        self.send_unchecked(command);
    }

    fn send_unchecked(&self, command: Command) {
        // The session thread is gone when the link is down: the command has nowhere to go.
        let _ = self.input.send(Input::Command(command));
    }

    /// Test seam for the negative control: the same send with the gate removed.
    #[cfg(test)]
    pub(crate) fn send_ungated(&self, command: Command) {
        self.send_unchecked(command);
    }
}

impl Drop for Session {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        for thread in self.threads.drain(..) {
            let _ = thread.join();
        }
    }
}

fn lock(state: &Mutex<VehicleState>) -> MutexGuard<'_, VehicleState> {
    state.lock().unwrap_or_else(PoisonError::into_inner)
}

fn read_loop(mut reader: Box<dyn Read + Send>, input: &Sender<Input>, stop: &AtomicBool) {
    let mut deframer = Deframer::new();
    let mut buffer = [0u8; READ_BUFFER];
    let mut frames = Vec::new();
    let mut reported = RxStats::default();
    while !stop.load(Ordering::Relaxed) {
        let reason = match reader.read(&mut buffer) {
            Ok(0) => "link closed by the other end".to_owned(),
            Ok(n) => {
                deframer.push(&buffer[..n], &mut frames);
                let stats = deframer.stats();
                if !frames.is_empty() || stats != reported {
                    reported = stats;
                    if input
                        .send(Input::Received(std::mem::take(&mut frames), stats))
                        .is_err()
                    {
                        return;
                    }
                }
                continue;
            }
            Err(error) if error.kind() == ErrorKind::TimedOut => continue,
            Err(error) => format!("read failed: {error}"),
        };
        let _ = input.send(Input::LinkDown(reason));
        return;
    }
}

struct Worker {
    state: Arc<Mutex<VehicleState>>,
    stop: Arc<AtomicBool>,
    services: Vec<Box<dyn Service>>,
    encoder: Encoder,
    writer: Box<dyn Write + Send>,
    out: Vec<crate::Message>,
    /// Something the window shows changed since the last wake check.
    changed: bool,
    started: Instant,
    wake: Box<dyn Fn() + Send + Sync>,
}

impl Worker {
    /// Runs the session; a panic in a service closes the link visibly instead of dying silently.
    fn run_guarded(mut self, inbox: &Receiver<Input>) {
        if let Err(payload) = catch_unwind(AssertUnwindSafe(|| self.run(inbox))) {
            self.close(format!("internal error: {}", panic_text(payload.as_ref())));
            (self.wake)();
        }
    }

    fn run(&mut self, inbox: &Receiver<Input>) {
        let mut next_tick = Instant::now();
        let mut dirty = false;
        let mut last_wake: Option<Instant> = None;
        let mut ended = false;
        while !self.stop.load(Ordering::Relaxed) && !ended {
            let mut deadline = next_tick;
            if dirty && let Some(last) = last_wake {
                deadline = deadline.min(last + WAKE_INTERVAL);
            }
            match inbox.recv_timeout(deadline.saturating_duration_since(Instant::now())) {
                Ok(input) => {
                    ended = self.handle(input).is_err();
                }
                Err(RecvTimeoutError::Timeout) => {}
                Err(RecvTimeoutError::Disconnected) => break,
            }
            let now = Instant::now();
            if !ended && now >= next_tick {
                next_tick = now + TICK;
                self.tick();
                ended = self.flush().is_err();
            }
            dirty |= std::mem::take(&mut self.changed);
            dirty |= ended;
            if dirty
                && (ended || last_wake.is_none_or(|last| now.duration_since(last) >= WAKE_INTERVAL))
            {
                dirty = false;
                last_wake = Some(now);
                (self.wake)();
            }
        }
    }

    /// Runs `f` on all services with a context.
    fn with_ctx(&mut self, f: impl FnOnce(&mut [Box<dyn Service>], &mut Ctx)) {
        let mut state = lock(&self.state);
        let mut cx = Ctx {
            now: self.started.elapsed(),
            wall_time_us: wall_time_us(),
            state: &mut state,
            out: &mut self.out,
            changed: false,
        };
        f(&mut self.services, &mut cx);
        self.changed |= cx.changed;
    }

    fn tick(&mut self) {
        self.with_ctx(|services, cx| {
            for service in services {
                service.on_tick(cx);
            }
        });
    }

    /// `Err` when the session has to end (link down or write failed); the state says why.
    fn handle(&mut self, input: Input) -> Result<(), ()> {
        match input {
            Input::Received(frames, rx) => {
                self.changed = true;
                self.with_ctx(|services, cx| {
                    cx.state.link.rx = rx;
                    for frame in &frames {
                        for service in services.iter_mut() {
                            service.on_frame(frame, cx);
                        }
                    }
                });
                self.flush()
            }
            Input::Command(command) => {
                self.with_ctx(|services, cx| {
                    for service in services {
                        service.on_command(&command, cx);
                    }
                });
                self.flush()
            }
            Input::LinkDown(reason) => {
                self.close(reason);
                Err(())
            }
        }
    }

    /// Encodes and writes what the services queued, then tells them what went out.
    fn flush(&mut self) -> Result<(), ()> {
        while !self.out.is_empty() && !self.stop.load(Ordering::Relaxed) {
            let messages = std::mem::take(&mut self.out);
            let mut sent = Vec::with_capacity(messages.len());
            for message in &messages {
                let raw = self.encoder.encode(message);
                if let Err(error) = self
                    .writer
                    .write_all(&raw)
                    .and_then(|()| self.writer.flush())
                {
                    self.close(format!("write failed: {error}"));
                    return Err(());
                }
                lock(&self.state).link.tx_frames += 1;
                self.changed = true;
                sent.push(raw);
            }
            self.with_ctx(|services, cx| {
                for raw in &sent {
                    for service in services.iter_mut() {
                        service.on_sent(raw, cx);
                    }
                }
            });
        }
        Ok(())
    }

    fn close(&mut self, reason: String) {
        lock(&self.state).link.status = LinkStatus::Closed(reason);
        self.stop.store(true, Ordering::Relaxed);
    }
}

fn panic_text(payload: &(dyn Any + Send)) -> String {
    if let Some(text) = payload.downcast_ref::<&str>() {
        (*text).to_owned()
    } else if let Some(text) = payload.downcast_ref::<String>() {
        text.clone()
    } else {
        "a service panicked".to_owned()
    }
}

fn wall_time_us() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |elapsed| {
            u64::try_from(elapsed.as_micros()).unwrap_or(u64::MAX)
        })
}

#[cfg(test)]
#[path = "session_tests.rs"]
mod tests;
