//! Serial tests against a pty pair made by socat. A missing socat is a failure, not a skip.

use std::fs::{File, OpenOptions};
use std::io::{BufRead, BufReader, ErrorKind, Read, Write};
use std::process::{Child, ChildStderr, Command, Stdio};
use std::sync::mpsc::{self, Receiver};
use std::thread;
use std::time::{Duration, Instant};

use gcs_transport::{Endpoint, Link, READ_TIMEOUT, list_serial_ports, open};

const SOCAT: &str = "/usr/bin/socat";

struct Socat {
    child: Child,
    _stderr: BufReader<ChildStderr>,
}

impl Drop for Socat {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

impl Socat {
    fn kill(&mut self) {
        self.child.kill().unwrap();
        self.child.wait().unwrap();
    }
}

fn pty_pair() -> (Socat, String, String) {
    let child = Command::new(SOCAT)
        .args(["-d", "-d", "pty,raw,echo=0", "pty,raw,echo=0"])
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap_or_else(|e| {
            panic!("cannot run {SOCAT} (socat must be installed for these tests): {e}")
        });
    let mut child = child;
    let stderr = BufReader::new(child.stderr.take().unwrap());
    let mut s = Socat {
        child,
        _stderr: stderr,
    };
    let mut paths = Vec::new();
    let mut line = String::new();
    let deadline = Instant::now() + Duration::from_secs(10);
    while paths.len() < 2 {
        assert!(
            Instant::now() < deadline,
            "socat did not report two ptys: {paths:?}"
        );
        line.clear();
        let n = s._stderr.read_line(&mut line).unwrap();
        assert!(n > 0, "socat exited before reporting two ptys: {paths:?}");
        if let Some(i) = line.find("PTY is ") {
            paths.push(line[i + 7..].trim().to_string());
        }
    }
    let b = paths.pop().unwrap();
    let a = paths.pop().unwrap();
    (s, a, b)
}

/// Link on one pty, a plain file on the other (read by a thread, chunks arrive on the channel).
fn setup() -> (Socat, Link, File, Receiver<Vec<u8>>) {
    let (socat, a, b) = pty_pair();
    let link = open(&Endpoint::Serial {
        path: a,
        baud: 115_200,
    })
    .expect("open pty");
    let file = OpenOptions::new().read(true).write(true).open(&b).unwrap();
    let mut rd = file.try_clone().unwrap();
    let (tx, rx) = mpsc::channel();
    thread::spawn(move || {
        let mut buf = [0u8; 256];
        while let Ok(n) = rd.read(&mut buf) {
            if n == 0 || tx.send(buf[..n].to_vec()).is_err() {
                break;
            }
        }
    });
    (socat, link, file, rx)
}

fn read_n(r: &mut dyn Read, n: usize) -> Vec<u8> {
    let mut out = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut buf = [0u8; 64];
    while out.len() < n {
        assert!(Instant::now() < deadline, "got {out:?}, wanted {n} bytes");
        match r.read(&mut buf) {
            Ok(0) => panic!("closed early after {out:?}"),
            Ok(k) => out.extend_from_slice(&buf[..k]),
            Err(e) if e.kind() == ErrorKind::TimedOut => {}
            Err(e) => panic!("{e}"),
        }
    }
    out
}

fn recv_n(rx: &Receiver<Vec<u8>>, n: usize) -> Vec<u8> {
    let mut out = Vec::new();
    while out.len() < n {
        out.extend(
            rx.recv_timeout(Duration::from_secs(5))
                .expect("no data from the link"),
        );
    }
    out
}

#[test]
fn bytes_both_ways() {
    let (_s, mut link, mut file, rx) = setup();
    file.write_all(b"to the link").unwrap();
    assert_eq!(read_n(&mut link.reader, 11), b"to the link");

    link.writer.write_all(b"from the link").unwrap();
    link.writer.flush().unwrap();
    assert_eq!(recv_n(&rx, 13), b"from the link");
}

#[test]
fn idle_read_times_out_with_timed_out_kind() {
    let (_s, mut link, _file, _rx) = setup();
    let mut buf = [0u8; 16];
    let t = Instant::now();
    let e = link.reader.read(&mut buf).unwrap_err();
    assert_eq!(e.kind(), ErrorKind::TimedOut);
    assert!(t.elapsed() >= READ_TIMEOUT / 2, "returned too early");
    assert!(t.elapsed() < Duration::from_secs(1), "returned too late");
}

#[test]
fn timeout_loses_no_bytes() {
    let (_s, mut link, mut file, _rx) = setup();
    file.write_all(b"HEAD").unwrap();
    assert_eq!(read_n(&mut link.reader, 4), b"HEAD");
    let mut buf = [0u8; 16];
    assert_eq!(
        link.reader.read(&mut buf).unwrap_err().kind(),
        ErrorKind::TimedOut
    );
    thread::sleep(READ_TIMEOUT * 2);
    file.write_all(b"TAIL").unwrap();
    assert_eq!(read_n(&mut link.reader, 4), b"TAIL");
}

#[test]
fn reader_and_writer_work_from_two_threads() {
    let (_s, link, mut file, rx) = setup();
    let Link {
        mut reader,
        mut writer,
    } = link;
    let rd = thread::spawn(move || read_n(&mut reader, 500));
    let out: Vec<u8> = (0..500u32).map(|i| (i % 253) as u8).collect();
    let wr = {
        let out = out.clone();
        thread::spawn(move || {
            writer.write_all(&out).unwrap();
            writer.flush().unwrap();
        })
    };
    let inn: Vec<u8> = (0..500u32).map(|i| (i % 247) as u8).collect();
    file.write_all(&inn).unwrap();
    assert_eq!(recv_n(&rx, 500), out);
    wr.join().unwrap();
    assert_eq!(rd.join().unwrap(), inn);
}

#[test]
fn peer_gone_is_reported_and_stays_reported() {
    let (mut s, mut link, _file, _rx) = setup();
    s.kill();
    let mut buf = [0u8; 16];
    for i in 0..3 {
        match link.reader.read(&mut buf) {
            Ok(0) => {}
            Ok(n) => panic!("got {n} bytes after the peer was killed"),
            Err(e) => assert_ne!(e.kind(), ErrorKind::TimedOut, "read {i}: {e}"),
        }
    }
}

#[test]
fn open_of_a_missing_port_names_the_endpoint() {
    let ep = Endpoint::Serial {
        path: "/dev/does-not-exist-marv".into(),
        baud: 9600,
    };
    let e = match open(&ep) {
        Ok(_) => panic!("opened a missing port"),
        Err(e) => e,
    };
    assert!(
        e.to_string()
            .contains("serial:/dev/does-not-exist-marv:9600"),
        "{e}"
    );
}

#[test]
fn list_serial_ports_does_not_panic_and_paths_are_not_empty() {
    for p in list_serial_ports() {
        assert!(!p.path.is_empty(), "{p:?}");
    }
}
