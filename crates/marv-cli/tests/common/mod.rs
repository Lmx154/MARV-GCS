//! A simulated vehicle on a real TCP socket, and the CLI run against it. Every wait is bounded.
#![allow(dead_code)]

use std::io::Write;
use std::net::TcpListener;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

use gcs_sim::{SimConfig, SimVehicle};

pub struct Rig {
    pub addr: String,
    /// The simulated vehicle, for a test that arms it.
    pub vehicle: Arc<Mutex<SimVehicle>>,
    stop: Arc<AtomicBool>,
    server: Option<JoinHandle<()>>,
}

impl Rig {
    pub fn start() -> Rig {
        Rig::with(SimConfig::default().with_device_status(1.0))
    }

    pub fn with(config: SimConfig) -> Rig {
        let listener = TcpListener::bind("127.0.0.1:0").expect("bind");
        let addr = listener.local_addr().expect("address").to_string();
        let vehicle = Arc::new(Mutex::new(SimVehicle::new(config)));
        let stop = Arc::new(AtomicBool::new(false));
        let server = {
            let stop = Arc::clone(&stop);
            let vehicle = Arc::clone(&vehicle);
            std::thread::spawn(move || gcs_sim::serve_tcp(listener, vehicle, stop))
        };
        Rig {
            addr,
            vehicle,
            stop,
            server: Some(server),
        }
    }

    /// Runs `marv-cli --connect tcp:<addr> <words>`; returns the status, stdout and stderr.
    pub fn cli(&self, words: &str) -> Output {
        run(&format!("--connect tcp:{} {words}", self.addr))
    }
}

impl Drop for Rig {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(server) = self.server.take() {
            let _ = server.join();
        }
    }
}

#[derive(Debug)]
pub struct Output {
    pub code: i32,
    pub out: String,
    pub err: String,
}

impl Output {
    pub fn lines(&self) -> Vec<&str> {
        self.out.lines().collect()
    }
}

pub fn run(words: &str) -> Output {
    let args: Vec<String> = words.split_whitespace().map(str::to_string).collect();
    let (mut out, mut err) = (Vec::new(), Vec::new());
    let code = marv_cli::run(&args, &mut out, &mut err);
    Output {
        code,
        out: String::from_utf8_lossy(&out).into_owned(),
        err: String::from_utf8_lossy(&err).into_owned(),
    }
}

/// A writer whose bytes another thread can read while the command still runs.
#[derive(Clone, Default)]
pub struct Shared(pub Arc<Mutex<Vec<u8>>>);

impl Shared {
    pub fn text(&self) -> String {
        String::from_utf8_lossy(&self.0.lock().unwrap_or_else(|e| e.into_inner())).into_owned()
    }
}

impl Write for Shared {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        self.0
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .extend_from_slice(buf);
        Ok(buf.len())
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

/// A directory of its own under the system's temporary directory, removed when dropped.
pub struct TempDir(pub PathBuf);

impl TempDir {
    pub fn new(name: &str) -> TempDir {
        let dir = std::env::temp_dir().join(format!("marv-cli-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("create the temporary directory");
        TempDir(dir)
    }

    pub fn file(&self, name: &str) -> PathBuf {
        self.0.join(name)
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}
