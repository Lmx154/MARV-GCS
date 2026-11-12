use std::io::{ErrorKind, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::thread;
use std::time::{Duration, Instant};

use gcs_transport::{Endpoint, Link, READ_TIMEOUT, open};

fn connect() -> (Link, TcpStream, Endpoint) {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let ep = Endpoint::Tcp {
        addr: listener.local_addr().unwrap().to_string(),
    };
    let link = open(&ep).expect("open");
    let (peer, _) = listener.accept().unwrap();
    (link, peer, ep)
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

#[test]
fn read_timeout_constant_is_100_ms() {
    assert_eq!(READ_TIMEOUT, Duration::from_millis(100));
}

#[test]
fn bytes_both_ways_and_read_does_not_wait_to_fill() {
    let (mut link, mut peer, _) = connect();
    peer.write_all(b"abc").unwrap();
    let mut buf = [0u8; 1024];
    let t = Instant::now();
    let n = link.reader.read(&mut buf).unwrap();
    assert!((1..=3).contains(&n));
    assert!(t.elapsed() < Duration::from_secs(1));
    let mut got = buf[..n].to_vec();
    got.extend(read_n(&mut link.reader, 3 - n));
    assert_eq!(got, b"abc");

    link.writer.write_all(b"xyz").unwrap();
    link.writer.flush().unwrap();
    let mut back = [0u8; 3];
    peer.read_exact(&mut back).unwrap();
    assert_eq!(&back, b"xyz");
}

#[test]
fn idle_read_times_out_with_timed_out_kind() {
    let (mut link, _peer, _) = connect();
    let mut buf = [0u8; 16];
    let t = Instant::now();
    let e = link.reader.read(&mut buf).unwrap_err();
    assert_eq!(e.kind(), ErrorKind::TimedOut);
    assert!(t.elapsed() >= READ_TIMEOUT / 2, "returned too early");
    assert!(t.elapsed() < Duration::from_secs(1), "returned too late");
}

#[test]
fn timeout_loses_no_bytes() {
    let (mut link, mut peer, _) = connect();
    peer.write_all(b"HEAD").unwrap();
    assert_eq!(read_n(&mut link.reader, 4), b"HEAD");
    let mut buf = [0u8; 16];
    let e = link.reader.read(&mut buf).unwrap_err();
    assert_eq!(e.kind(), ErrorKind::TimedOut);
    thread::sleep(READ_TIMEOUT * 2);
    peer.write_all(b"TAIL").unwrap();
    assert_eq!(read_n(&mut link.reader, 4), b"TAIL");
}

#[test]
fn bytes_sent_while_nobody_reads_survive_several_timeouts() {
    let (mut link, mut peer, _) = connect();
    peer.write_all(b"one").unwrap();
    thread::sleep(READ_TIMEOUT * 3);
    peer.write_all(b"two").unwrap();
    thread::sleep(READ_TIMEOUT * 3);
    assert_eq!(read_n(&mut link.reader, 6), b"onetwo");
}

#[test]
fn peer_close_is_reported_and_stays_reported() {
    let (mut link, peer, _) = connect();
    drop(peer);
    let mut buf = [0u8; 16];
    for _ in 0..3 {
        match link.reader.read(&mut buf) {
            Ok(0) => {}
            Ok(n) => panic!("got {n} bytes from a closed peer"),
            Err(e) => assert_ne!(e.kind(), ErrorKind::TimedOut, "{e}"),
        }
    }
}

#[test]
fn data_before_close_is_still_delivered() {
    let (mut link, mut peer, _) = connect();
    peer.write_all(b"last words").unwrap();
    drop(peer);
    assert_eq!(read_n(&mut link.reader, 10), b"last words");
    let mut buf = [0u8; 16];
    assert!(matches!(link.reader.read(&mut buf), Ok(0)));
}

#[test]
fn reader_and_writer_work_from_two_threads() {
    let (link, mut peer, _) = connect();
    let Link {
        mut reader,
        mut writer,
    } = link;
    let echo = {
        let mut peer2 = peer.try_clone().unwrap();
        thread::spawn(move || {
            // Peer sends 1000 bytes while the link is writing 1000 bytes.
            let data: Vec<u8> = (0..1000u32).map(|i| (i % 251) as u8).collect();
            peer2.write_all(&data).unwrap();
        })
    };
    let rd = thread::spawn(move || read_n(&mut reader, 1000));
    let data: Vec<u8> = (0..1000u32).map(|i| (i % 249) as u8).collect();
    let wr = {
        let data = data.clone();
        thread::spawn(move || {
            writer.write_all(&data).unwrap();
            writer.flush().unwrap();
        })
    };
    let mut got = vec![0u8; 1000];
    peer.read_exact(&mut got).unwrap();
    assert_eq!(got, data);
    wr.join().unwrap();
    echo.join().unwrap();
    let expect: Vec<u8> = (0..1000u32).map(|i| (i % 251) as u8).collect();
    assert_eq!(rd.join().unwrap(), expect);
}

#[test]
fn dropping_both_halves_closes_the_connection() {
    let (link, mut peer, _) = connect();
    drop(link);
    peer.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
    let mut buf = [0u8; 4];
    assert!(matches!(peer.read(&mut buf), Ok(0)));
}

#[test]
fn refused_address_is_an_error_and_is_quick() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap().to_string();
    drop(listener);
    let ep = Endpoint::Tcp { addr: addr.clone() };
    let t = Instant::now();
    let e = match open(&ep) {
        Ok(_) => panic!("opened a refused address"),
        Err(e) => e,
    };
    assert!(t.elapsed() < Duration::from_secs(3));
    let text = e.to_string();
    assert!(text.contains(&format!("tcp:{addr}")), "{text}");
    assert!(text.len() > format!("tcp:{addr}").len() + 10, "{text}");
    let _: &dyn std::error::Error = &e;
}

#[test]
fn unresolvable_address_is_an_error_not_a_panic() {
    for addr in ["no port here", "", "127.0.0.1:99999"] {
        assert!(
            open(&Endpoint::Tcp { addr: addr.into() }).is_err(),
            "{addr:?}"
        );
    }
}
