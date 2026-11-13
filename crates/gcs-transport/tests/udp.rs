use std::io::{ErrorKind, Read, Write};
use std::net::{SocketAddr, UdpSocket};
use std::time::{Duration, Instant};

use gcs_transport::{Endpoint, Link, READ_TIMEOUT, open};

/// A free local port: bind port 0, read it back, release it.
fn free_addr() -> SocketAddr {
    let s = UdpSocket::bind("127.0.0.1:0").unwrap();
    s.local_addr().unwrap()
}

fn peer() -> UdpSocket {
    let s = UdpSocket::bind("127.0.0.1:0").unwrap();
    s.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
    s
}

fn connect() -> (Link, SocketAddr, UdpSocket) {
    let bind = free_addr();
    let link = open(&Endpoint::Udp { bind }).expect("open");
    (link, bind, peer())
}

fn read_n(r: &mut dyn Read, n: usize, chunk: usize) -> Vec<u8> {
    let mut out = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut buf = vec![0u8; chunk];
    while out.len() < n {
        assert!(Instant::now() < deadline, "got {} of {n} bytes", out.len());
        match r.read(&mut buf) {
            Ok(0) => panic!("link down after {} bytes", out.len()),
            Ok(k) => out.extend_from_slice(&buf[..k]),
            Err(e) if e.kind() == ErrorKind::TimedOut => {}
            Err(e) => panic!("{e}"),
        }
    }
    out
}

#[test]
fn a_datagram_is_read_back_byte_for_byte() {
    let (mut link, bind, peer) = connect();
    let data: Vec<u8> = (0..=255).collect();
    peer.send_to(&data, bind).unwrap();
    assert_eq!(read_n(&mut link.reader, 256, 1024), data);
}

#[test]
fn two_datagrams_are_read_in_order() {
    let (mut link, bind, peer) = connect();
    peer.send_to(b"first", bind).unwrap();
    peer.send_to(b"second", bind).unwrap();
    assert_eq!(read_n(&mut link.reader, 11, 1024), b"firstsecond");
}

#[test]
fn a_datagram_larger_than_the_read_buffer_loses_nothing() {
    let (mut link, bind, peer) = connect();
    let data: Vec<u8> = (0..3000u32).map(|i| (i % 251) as u8).collect();
    peer.send_to(&data, bind).unwrap();
    peer.send_to(b"next", bind).unwrap();
    let mut got = read_n(&mut link.reader, 3000, 256);
    got.extend(read_n(&mut link.reader, 4, 256));
    assert_eq!(&got[..3000], &data[..]);
    assert_eq!(&got[3000..], b"next");
}

#[test]
fn the_largest_datagram_fits() {
    let (mut link, bind, peer) = connect();
    let data: Vec<u8> = (0..65_507u32).map(|i| (i % 253) as u8).collect();
    peer.send_to(&data, bind).unwrap();
    assert_eq!(read_n(&mut link.reader, data.len(), 4096), data);
}

#[test]
fn an_empty_datagram_is_skipped_and_never_ends_the_link() {
    let (mut link, bind, peer) = connect();
    peer.send_to(&[], bind).unwrap();
    peer.send_to(&[], bind).unwrap();
    peer.send_to(b"x", bind).unwrap();
    let mut buf = [0u8; 16];
    let n = loop {
        match link.reader.read(&mut buf) {
            Ok(0) => panic!("Ok(0) for an empty datagram means link down"),
            Ok(n) => break n,
            Err(e) if e.kind() == ErrorKind::TimedOut => {}
            Err(e) => panic!("{e}"),
        }
    };
    assert_eq!(&buf[..n], b"x");
    peer.send_to(&[], bind).unwrap();
    let e = link.reader.read(&mut buf).unwrap_err();
    assert_eq!(e.kind(), ErrorKind::TimedOut);
}

#[test]
fn nothing_arriving_gives_timed_out_within_about_the_read_timeout() {
    let (mut link, _bind, _peer) = connect();
    let mut buf = [0u8; 16];
    let t = Instant::now();
    let e = link.reader.read(&mut buf).unwrap_err();
    assert_eq!(e.kind(), ErrorKind::TimedOut);
    assert!(t.elapsed() >= READ_TIMEOUT / 2, "returned too early");
    assert!(t.elapsed() < Duration::from_secs(1), "returned too late");
}

#[test]
fn a_write_before_any_datagram_succeeds_and_goes_nowhere() {
    let (mut link, _bind, peer) = connect();
    assert_eq!(link.writer.write(b"hello").unwrap(), 5);
    link.writer.flush().unwrap();
    peer.set_read_timeout(Some(Duration::from_millis(200)))
        .unwrap();
    let mut buf = [0u8; 16];
    let e = peer.recv_from(&mut buf).unwrap_err();
    assert!(
        matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut),
        "{e}"
    );
}

#[test]
fn after_the_first_datagram_a_write_is_one_datagram_to_the_peer() {
    let (mut link, bind, peer) = connect();
    peer.send_to(b"hi", bind).unwrap();
    read_n(&mut link.reader, 2, 16);
    let data: Vec<u8> = (0..200u8).collect();
    assert_eq!(link.writer.write(&data).unwrap(), data.len());
    let mut buf = [0u8; 1024];
    let (n, from) = peer.recv_from(&mut buf).unwrap();
    assert_eq!(&buf[..n], &data[..]);
    assert_eq!(from, bind);
    link.writer.write_all(b"ab").unwrap();
    let (n, _) = peer.recv_from(&mut buf).unwrap();
    assert_eq!(&buf[..n], b"ab");
}

#[test]
fn the_peer_is_the_latest_sender() {
    let (mut link, bind, first) = connect();
    let second = self::peer();
    first.send_to(b"1", bind).unwrap();
    read_n(&mut link.reader, 1, 16);
    link.writer.write_all(b"to-first").unwrap();
    let mut buf = [0u8; 64];
    let (n, _) = first.recv_from(&mut buf).unwrap();
    assert_eq!(&buf[..n], b"to-first");

    second.send_to(b"2", bind).unwrap();
    read_n(&mut link.reader, 1, 16);
    link.writer.write_all(b"to-second").unwrap();
    let (n, _) = second.recv_from(&mut buf).unwrap();
    assert_eq!(&buf[..n], b"to-second");
    first
        .set_read_timeout(Some(Duration::from_millis(200)))
        .unwrap();
    assert!(
        first.recv_from(&mut buf).is_err(),
        "the old peer got it too"
    );
}

#[test]
fn a_peer_that_went_away_is_not_a_link_down_and_a_new_one_works() {
    let (mut link, bind, gone) = connect();
    gone.send_to(b"hello", bind).unwrap();
    read_n(&mut link.reader, 5, 16);
    drop(gone);
    for _ in 0..5 {
        assert_eq!(link.writer.write(b"into the void").unwrap(), 13);
        let mut buf = [0u8; 16];
        match link.reader.read(&mut buf) {
            Ok(0) => panic!("link down after the peer went away"),
            Ok(n) => panic!("{n} bytes from nowhere"),
            Err(e) => assert_eq!(e.kind(), ErrorKind::TimedOut, "{e}"),
        }
    }
    let back = peer();
    back.send_to(b"again", bind).unwrap();
    assert_eq!(read_n(&mut link.reader, 5, 16), b"again");
    link.writer.write_all(b"pong").unwrap();
    let mut buf = [0u8; 16];
    let (n, _) = back.recv_from(&mut buf).unwrap();
    assert_eq!(&buf[..n], b"pong");
}

#[test]
fn reader_and_writer_work_from_two_threads() {
    let (link, bind, peer) = connect();
    let Link {
        mut reader,
        mut writer,
    } = link;
    peer.send_to(b"0", bind).unwrap();
    read_n(&mut reader, 1, 16);
    let rd = std::thread::spawn(move || read_n(&mut reader, 100, 7));
    let wr = std::thread::spawn(move || {
        for i in 0..100u8 {
            writer.write_all(&[i]).unwrap();
        }
    });
    for i in 0..100u8 {
        peer.send_to(&[i], bind).unwrap();
    }
    wr.join().unwrap();
    assert_eq!(rd.join().unwrap(), (0..100u8).collect::<Vec<_>>());
    let mut buf = [0u8; 4];
    for i in 0..100u8 {
        let (n, _) = peer.recv_from(&mut buf).unwrap();
        assert_eq!(&buf[..n], &[i]);
    }
}

#[test]
fn a_bind_that_fails_is_an_open_error() {
    let taken = UdpSocket::bind("127.0.0.1:0").unwrap();
    let bind = taken.local_addr().unwrap();
    let ep = Endpoint::Udp { bind };
    let e = match open(&ep) {
        Ok(_) => panic!("bound a port that is in use"),
        Err(e) => e,
    };
    assert!(e.to_string().contains(&format!("udp:{bind}")), "{e}");
}

#[test]
fn the_endpoint_text_round_trips_and_bad_forms_are_rejected() {
    for text in ["udp:0.0.0.0:14550", "udp:127.0.0.1:1", "udp:[::1]:14550"] {
        let ep: Endpoint = text.parse().unwrap();
        assert_eq!(ep.to_string(), text);
    }
    for bad in [
        "udp:",
        "udp:0.0.0.0",
        "udp:0.0.0.0:",
        "udp:999.1.1.1:14550",
        "udp:0.0.0.0:0",
        "udp:host:14550",
    ] {
        assert!(bad.parse::<Endpoint>().is_err(), "{bad:?}");
    }
}
