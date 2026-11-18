use std::io;
use std::sync::{Arc, Mutex};

use super::*;
use crate::tlog::LogSink;
use crate::{VehicleId, VehicleState};

/// A writer the test can read back, and make fail.
#[derive(Clone, Default)]
struct Shared {
    bytes: Arc<Mutex<Vec<u8>>>,
    flushes: Arc<Mutex<u32>>,
    /// Fail every write once this many bytes are in.
    fail_after: Option<usize>,
    /// Fail every flush.
    fail_flush: bool,
}

impl Shared {
    fn bytes(&self) -> Vec<u8> {
        self.bytes.lock().unwrap().clone()
    }

    fn flushes(&self) -> u32 {
        *self.flushes.lock().unwrap()
    }
}

impl Write for Shared {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let mut bytes = self.bytes.lock().unwrap();
        if self.fail_after.is_some_and(|limit| bytes.len() >= limit) {
            return Err(io::Error::other("disk full"));
        }
        bytes.extend_from_slice(buf);
        Ok(buf.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        *self.flushes.lock().unwrap() += 1;
        if self.fail_flush {
            return Err(io::Error::other("flush failed"));
        }
        Ok(())
    }
}

/// The bytes of a made-up log: no two neighbouring frames alike.
fn log_bytes(len: usize) -> Vec<u8> {
    (0..len)
        .map(|i| (i.wrapping_mul(31) ^ (i >> 7)) as u8)
        .collect()
}

struct Rig {
    service: Logs,
    state: VehicleState,
    /// Every message sent, with the call's time in ms.
    log: Vec<(u64, Message)>,
    sink: Shared,
}

impl Rig {
    fn new() -> Self {
        let mut state = VehicleState::default();
        state.link.status = LinkStatus::Connected;
        state.link.vehicle = Some(VehicleId {
            system_id: 1,
            component_id: 1,
            mav_type: "MAV_TYPE_GENERIC".into(),
            autopilot: "MAV_AUTOPILOT_GENERIC".into(),
        });
        Self {
            service: Logs::new(),
            state,
            log: Vec::new(),
            sink: Shared::default(),
        }
    }

    fn call(&mut self, now_ms: u64, f: impl FnOnce(&mut Logs, &mut Ctx)) {
        let mut out = Vec::new();
        let mut cx = Ctx {
            now: Duration::from_millis(now_ms),
            wall_time_us: 0,
            state: &mut self.state,
            out: &mut out,
            changed: false,
        };
        f(&mut self.service, &mut cx);
        for message in out {
            self.log.push((now_ms, message));
        }
    }

    fn command(&mut self, command: Command, now_ms: u64) {
        self.call(now_ms, |s, cx| s.on_command(&command, cx));
    }

    fn list(&mut self, now_ms: u64) {
        self.command(Command::ListLogs, now_ms);
    }

    fn get(&mut self, number: u16, size: u32, now_ms: u64) {
        let sink = LogSink::new(self.sink.clone());
        self.command(Command::DownloadLog { number, size, sink }, now_ms);
    }

    fn get_with(&mut self, sink: Shared, number: u16, size: u32, now_ms: u64) {
        self.sink = sink.clone();
        let sink = LogSink::new(sink);
        self.command(Command::DownloadLog { number, size, sink }, now_ms);
    }

    fn tick(&mut self, now_ms: u64) {
        self.call(now_ms, |s, cx| s.on_tick(cx));
    }

    /// Ticks every 50 ms from `from` to `to` inclusive.
    fn tick_range(&mut self, from: u64, to: u64) {
        let mut t = from;
        while t <= to {
            self.tick(t);
            t += 50;
        }
    }

    fn deliver(&mut self, system: u8, message: Message, now_ms: u64) {
        let frame = Frame {
            system_id: system,
            component_id: 1,
            sequence: 0,
            message_id: 0,
            raw: Vec::new(),
            body: Body::Known(message),
        };
        self.call(now_ms, |s, cx| s.on_frame(&frame, cx));
    }

    fn entry(&mut self, id: u16, num_logs: u16, last: u16, size: u32, now_ms: u64) {
        self.deliver(
            1,
            Message::LOG_ENTRY(LOG_ENTRY_DATA {
                time_utc: 0,
                size,
                id,
                num_logs,
                last_log_num: last,
            }),
            now_ms,
        );
    }

    fn data_frame(id: u16, ofs: u32, bytes: &[u8]) -> Message {
        let mut data = [0u8; 90];
        data[..bytes.len()].copy_from_slice(bytes);
        Message::LOG_DATA(LOG_DATA_DATA {
            ofs,
            id,
            count: bytes.len() as u8,
            data,
        })
    }

    /// One LOG_DATA of log `id` at `ofs`, with the bytes `data` has there (at most 90).
    fn frame(&mut self, id: u16, data: &[u8], ofs: u32, now_ms: u64) {
        let end = (ofs as usize + 90).min(data.len());
        let message = Self::data_frame(id, ofs, &data[ofs as usize..end]);
        self.deliver(1, message, now_ms);
    }

    /// What the vehicle sends for a request of `count` bytes at `ofs` of `data`, in order, 90 bytes a
    /// frame, all at `now_ms`; the frames at the offsets in `lose` do not arrive.
    fn serve(&mut self, id: u16, data: &[u8], ofs: u32, count: u32, lose: &[u32], now_ms: u64) {
        let end = (ofs as usize + count as usize).min(data.len());
        let mut at = ofs as usize;
        let mut frames = 0;
        while at < end {
            if !lose.contains(&(at as u32)) {
                self.frame(id, data, at as u32, now_ms);
            }
            at += 90;
            frames += 1;
            assert!(frames <= 1000, "a window has at most 512 frames");
        }
    }

    /// Serves the last LOG_REQUEST_DATA sent, whole.
    fn serve_last(&mut self, data: &[u8], lose: &[u32], now_ms: u64) {
        let (_, id, ofs, count) = *self.requests().last().expect("a request");
        self.serve(id, data, ofs, count, lose, now_ms);
    }

    /// (time, id, ofs, count) of each LOG_REQUEST_DATA sent.
    fn requests(&self) -> Vec<(u64, u16, u32, u32)> {
        self.log
            .iter()
            .filter_map(|(t, m)| match m {
                Message::LOG_REQUEST_DATA(r) => Some((*t, r.id, r.ofs, r.count)),
                _ => None,
            })
            .collect()
    }

    /// Times of the LOG_REQUEST_LIST sent.
    fn list_asks(&self) -> Vec<u64> {
        self.log
            .iter()
            .filter_map(|(t, m)| matches!(m, Message::LOG_REQUEST_LIST(_)).then_some(*t))
            .collect()
    }

    /// Times of the LOG_REQUEST_END sent.
    fn ends(&self) -> Vec<u64> {
        self.log
            .iter()
            .filter_map(|(t, m)| matches!(m, Message::LOG_REQUEST_END(_)).then_some(*t))
            .collect()
    }

    fn download(&self) -> &DownloadState {
        self.state.logs.download.as_ref().expect("a download")
    }

    fn status(&self) -> DownloadStatus {
        self.download().status.clone()
    }
}

fn done_list(rig: &Rig) -> LogList {
    match &rig.state.logs.list {
        ListStatus::Done(list) => list.clone(),
        other => panic!("not done: {other:?}"),
    }
}

fn info(number: u16, size: u32) -> LogInfo {
    LogInfo { number, size }
}

// ---- listing

#[test]
fn a_listing_asks_for_all_logs_and_collects_one_entry_each() {
    let mut rig = Rig::new();
    rig.list(100);
    assert_eq!(rig.state.logs.lists, 1);
    let Message::LOG_REQUEST_LIST(ask) = &rig.log[0].1 else {
        panic!("{:?}", rig.log);
    };
    assert_eq!(
        (ask.target_system, ask.target_component, ask.start, ask.end),
        (1, 1, 0, 0xFFFF)
    );
    assert_eq!(
        rig.state.logs.list,
        ListStatus::Listing {
            received: 0,
            expected: None
        }
    );
    rig.entry(3, 3, 7, 5000, 120);
    assert_eq!(
        rig.state.logs.list,
        ListStatus::Listing {
            received: 1,
            expected: Some(3)
        }
    );
    rig.entry(4, 3, 7, 0, 125);
    rig.entry(7, 3, 7, 123_456, 130);
    assert_eq!(
        done_list(&rig),
        LogList {
            logs: vec![info(3, 5000), info(4, 0), info(7, 123_456)],
            last_log_num: 7
        }
    );
    // Nothing more is asked once it is complete, however long it takes.
    rig.tick_range(150, 5000);
    assert_eq!(rig.list_asks(), vec![100]);
}

#[test]
fn numbers_with_gaps_are_kept_as_they_are_and_sorted() {
    let mut rig = Rig::new();
    rig.list(0);
    rig.entry(500, 3, 500, 10, 10);
    rig.entry(1, 3, 500, 20, 11);
    rig.entry(42, 3, 500, 30, 12);
    assert_eq!(
        done_list(&rig).logs,
        vec![info(1, 20), info(42, 30), info(500, 10)]
    );
}

#[test]
fn no_logs_is_an_empty_list_and_not_an_error() {
    let mut rig = Rig::new();
    rig.list(0);
    rig.entry(0, 0, 0, 0, 10);
    assert_eq!(
        rig.state.logs.list,
        ListStatus::Done(LogList {
            logs: Vec::new(),
            last_log_num: 0
        })
    );
    rig.tick_range(50, 5000);
    assert_eq!(rig.list_asks().len(), 1);
}

#[test]
fn a_lost_entry_is_asked_for_again_after_a_second_of_quiet() {
    let mut rig = Rig::new();
    rig.list(0);
    rig.entry(3, 3, 9, 100, 10);
    rig.entry(9, 3, 9, 300, 15);
    // The entry of log 5 was lost. 1 s after the last new entry the list is asked for again.
    rig.tick_range(50, 1010);
    assert_eq!(rig.list_asks(), vec![0]);
    rig.tick(1015);
    assert_eq!(rig.list_asks(), vec![0, 1015]);
    // The vehicle sends them all again: the ones held are not counted twice.
    rig.entry(3, 3, 9, 100, 1020);
    rig.entry(5, 3, 9, 200, 1025);
    assert_eq!(
        done_list(&rig).logs,
        vec![info(3, 100), info(5, 200), info(9, 300)]
    );
}

#[test]
fn a_listing_gives_up_after_four_asks_without_any_answer() {
    let mut rig = Rig::new();
    rig.list(0);
    rig.tick_range(50, 10_000);
    assert_eq!(rig.list_asks(), vec![0, 1000, 2000, 3000]);
    assert_eq!(rig.state.logs.list, ListStatus::Failed(ListError::NoAnswer));
}

#[test]
fn a_listing_that_stays_short_fails_with_the_counts() {
    let mut rig = Rig::new();
    rig.list(0);
    rig.entry(3, 3, 9, 100, 10);
    rig.entry(9, 3, 9, 300, 15);
    rig.tick_range(50, 10_000);
    assert_eq!(rig.list_asks().len(), 4);
    assert_eq!(
        rig.state.logs.list,
        ListStatus::Failed(ListError::Incomplete {
            received: 2,
            expected: 3
        })
    );
}

#[test]
fn a_listing_without_a_vehicle_sends_nothing() {
    let mut rig = Rig::new();
    rig.state.link.status = LinkStatus::Lost;
    rig.list(0);
    assert!(rig.log.is_empty());
    assert_eq!(
        rig.state.logs.list,
        ListStatus::Failed(ListError::NoVehicle)
    );
}

#[test]
fn an_entry_from_another_system_or_one_nobody_asked_for_changes_nothing() {
    let mut rig = Rig::new();
    rig.entry(3, 1, 3, 100, 5);
    assert_eq!(rig.state.logs.list, ListStatus::Idle);
    rig.list(10);
    rig.deliver(
        2,
        Message::LOG_ENTRY(LOG_ENTRY_DATA {
            time_utc: 0,
            size: 1,
            id: 3,
            num_logs: 1,
            last_log_num: 3,
        }),
        20,
    );
    assert!(matches!(
        rig.state.logs.list,
        ListStatus::Listing { received: 0, .. }
    ));
}

// ---- download

#[test]
fn a_clean_download_is_one_window_and_the_bytes_in_order() {
    let data = log_bytes(1000); // 11 frames and 10 bytes: not a multiple of 90
    let mut rig = Rig::new();
    rig.get(5, 1000, 200);
    assert_eq!(rig.state.logs.downloads, 1);
    assert_eq!(rig.requests(), vec![(200, 5, 0, 1000)]);
    let Message::LOG_REQUEST_DATA(ask) = &rig.log[0].1 else {
        panic!("{:?}", rig.log);
    };
    assert_eq!((ask.target_system, ask.target_component), (1, 1));
    rig.serve_last(&data, &[], 250);
    assert_eq!(rig.sink.bytes(), data);
    let download = rig.download();
    assert_eq!(download.status, DownloadStatus::Done);
    assert_eq!(
        (download.done, download.size, download.requests),
        (1000, 1000, 1)
    );
    assert_eq!(download.ended, Some(Duration::from_millis(250)));
    assert_eq!(rig.sink.flushes(), 1);
    // Done: nothing is sent, now or later, no END either.
    rig.tick_range(300, 8000);
    assert_eq!(rig.requests().len(), 1);
    assert!(rig.ends().is_empty());
}

#[test]
fn progress_follows_the_bytes_handed_on() {
    let data = log_bytes(900);
    let mut rig = Rig::new();
    rig.get(5, 900, 0);
    rig.frame(5, &data, 0, 10);
    assert_eq!(rig.download().done, 90);
    rig.frame(5, &data, 90, 11);
    rig.frame(5, &data, 270, 12); // out of order: not counted yet
    assert_eq!(rig.download().done, 180);
    rig.frame(5, &data, 180, 13); // fills the gap: the held one follows
    assert_eq!(rig.download().done, 360);
    assert_eq!(rig.download().status, DownloadStatus::Running);
}

#[test]
fn a_log_of_less_than_one_frame_downloads() {
    let data = log_bytes(7);
    let mut rig = Rig::new();
    rig.get(2, 7, 0);
    assert_eq!(rig.requests(), vec![(0, 2, 0, 7)]);
    rig.serve_last(&data, &[], 5);
    assert_eq!(rig.sink.bytes(), data);
    assert_eq!(rig.status(), DownloadStatus::Done);
}

#[test]
fn a_log_of_several_windows_is_asked_for_window_by_window() {
    let size = (WINDOW_BYTES * 2 + 500) as usize;
    let data = log_bytes(size);
    let mut rig = Rig::new();
    rig.get(9, size as u32, 0);
    assert_eq!(rig.requests(), vec![(0, 9, 0, WINDOW_BYTES)]);
    rig.serve_last(&data, &[], 640);
    // The window is complete: the next one is asked for at once.
    assert_eq!(rig.requests().len(), 2);
    assert_eq!(rig.requests()[1], (640, 9, WINDOW_BYTES, WINDOW_BYTES));
    rig.serve_last(&data, &[], 1280);
    assert_eq!(rig.requests()[2], (1280, 9, WINDOW_BYTES * 2, 500));
    rig.serve_last(&data, &[], 1300);
    assert_eq!(rig.status(), DownloadStatus::Done);
    assert_eq!(rig.sink.bytes(), data);
    assert_eq!(rig.requests().len(), 3);
}

#[test]
fn a_frame_lost_in_the_middle_of_a_window_is_asked_for_after_a_second() {
    let data = log_bytes(5000);
    let mut rig = Rig::new();
    rig.get(5, 5000, 0);
    rig.serve_last(&data, &[900], 100);
    assert_eq!(rig.download().done, 900);
    assert_eq!(rig.sink.bytes(), data[..900]);
    // 1 s without the first missing byte moving, counted from the last time it moved.
    rig.tick_range(150, 1050);
    assert_eq!(rig.requests().len(), 1);
    rig.tick(1100);
    assert_eq!(rig.requests(), vec![(0, 5, 0, 5000), (1100, 5, 900, 4100)]);
    // The vehicle sends from the first missing offset; what was held beyond it is not needed twice.
    rig.serve_last(&data, &[], 1150);
    assert_eq!(rig.status(), DownloadStatus::Done);
    assert_eq!(rig.sink.bytes(), data);
    assert_eq!(rig.requests().len(), 2);
}

#[test]
fn a_frame_lost_at_the_end_of_a_window_does_not_start_the_next_one() {
    let size = WINDOW_BYTES as usize + 1000;
    let data = log_bytes(size);
    let last_of_window = WINDOW_BYTES - 90;
    let mut rig = Rig::new();
    rig.get(5, size as u32, 0);
    rig.serve_last(&data, &[last_of_window], 600);
    assert_eq!(rig.download().done, last_of_window);
    assert_eq!(rig.requests().len(), 1);
    rig.tick_range(650, 1550);
    assert_eq!(rig.requests().len(), 1);
    rig.tick(1600);
    assert_eq!(
        rig.requests()[1],
        (1600, 5, last_of_window, 90 + 1000),
        "from the first missing byte, to the end of that window"
    );
    rig.serve_last(&data, &[], 1650);
    assert_eq!(rig.status(), DownloadStatus::Done);
    assert_eq!(rig.sink.bytes(), data);
}

#[test]
fn the_last_frame_of_the_log_lost_is_asked_for_again() {
    let data = log_bytes(1000);
    let mut rig = Rig::new();
    rig.get(5, 1000, 0);
    rig.serve_last(&data, &[990], 50);
    assert_eq!(rig.download().done, 990);
    rig.tick_range(100, 1100);
    assert_eq!(rig.requests().last(), Some(&(1050, 5, 990, 10)));
    rig.serve_last(&data, &[], 1100);
    assert_eq!(rig.sink.bytes(), data);
    assert_eq!(rig.status(), DownloadStatus::Done);
}

#[test]
fn duplicates_out_of_order_frames_and_strangers_are_tolerated() {
    let size = WINDOW_BYTES as usize + 1000;
    let data = log_bytes(size);
    let garbage = [0xEEu8; 90];
    let mut rig = Rig::new();
    rig.get(5, size as u32, 0);
    // The first frames, in reverse order, each twice.
    for ofs in [270, 180, 90, 0] {
        rig.frame(5, &data, ofs, 10);
        rig.frame(5, &data, ofs, 11);
    }
    assert_eq!(rig.download().done, 360);
    // A frame of another log at an offset that is wanted, and one at the end of the window and past it.
    rig.deliver(1, Rig::data_frame(6, 360, &garbage), 12);
    rig.deliver(1, Rig::data_frame(5, WINDOW_BYTES, &garbage), 12);
    rig.deliver(1, Rig::data_frame(5, WINDOW_BYTES + 90, &garbage), 12);
    assert_eq!(rig.download().done, 360);
    // The end marker (count 0) of the log completes nothing.
    rig.deliver(1, Rig::data_frame(5, 360, &[]), 13);
    assert_eq!(rig.download().done, 360);
    // A frame from another system.
    rig.deliver(2, Rig::data_frame(5, 360, &garbage), 13);
    assert_eq!(rig.download().done, 360);
    // The rest of the window.
    rig.serve(5, &data, 360, WINDOW_BYTES - 360, &[], 20);
    assert_eq!(rig.requests().len(), 2);
    rig.serve_last(&data, &[], 30);
    assert_eq!(rig.status(), DownloadStatus::Done);
    assert_eq!(rig.sink.bytes(), data);
}

#[test]
fn a_frame_that_overlaps_what_is_in_adds_only_its_new_part() {
    let data = log_bytes(500);
    let mut rig = Rig::new();
    rig.get(5, 500, 0);
    rig.frame(5, &data, 0, 1);
    // Offsets that do not fall on the frame grid: 60..150, then 120..210 (30 bytes are old), then
    // one beyond that is held and joined later.
    rig.deliver(1, Rig::data_frame(5, 90, &data[90..150]), 2);
    rig.deliver(1, Rig::data_frame(5, 120, &data[120..210]), 3);
    assert_eq!(rig.download().done, 210);
    rig.deliver(1, Rig::data_frame(5, 300, &data[300..390]), 4);
    rig.deliver(1, Rig::data_frame(5, 330, &data[330..420]), 5);
    rig.deliver(1, Rig::data_frame(5, 210, &data[210..300]), 6);
    assert_eq!(rig.download().done, 420);
    rig.deliver(1, Rig::data_frame(5, 420, &data[420..500]), 7);
    assert_eq!(rig.status(), DownloadStatus::Done);
    assert_eq!(rig.sink.bytes(), data);
}

#[test]
fn a_zero_length_log_is_done_at_once_and_asks_for_nothing() {
    let mut rig = Rig::new();
    rig.get(5, 0, 100);
    assert!(rig.log.is_empty(), "{:?}", rig.log);
    assert_eq!(rig.status(), DownloadStatus::Done);
    assert_eq!(rig.download().done, 0);
    assert!(rig.sink.bytes().is_empty());
    assert_eq!(rig.sink.flushes(), 1);
}

#[test]
fn a_log_that_grew_after_the_listing_ends_at_the_size_it_was_started_with() {
    let listed = 1000usize;
    let data = log_bytes(1500);
    let mut rig = Rig::new();
    rig.get(5, listed as u32, 0);
    assert_eq!(rig.requests(), vec![(0, 5, 0, 1000)]);
    // The vehicle's last frame of the window carries 90 bytes, 80 of them past the size, and it goes
    // on with frames beyond the window.
    rig.serve(5, &data, 0, 1500, &[], 50);
    assert_eq!(rig.status(), DownloadStatus::Done);
    assert_eq!(rig.sink.bytes(), data[..listed]);
    assert_eq!(rig.download().size, 1000);
    assert!(rig.ends().is_empty());
}

#[test]
fn a_silent_vehicle_ends_after_five_fruitless_retries_with_the_distinct_error() {
    let mut rig = Rig::new();
    rig.get(5, 5000, 0);
    rig.tick_range(50, 10_000);
    let times: Vec<u64> = rig.requests().iter().map(|r| r.0).collect();
    assert_eq!(
        times,
        vec![0, 1000, 2000, 3000, 4000, 5000],
        "1 + 5 retries"
    );
    assert!(
        rig.requests()
            .iter()
            .all(|r| (r.1, r.2, r.3) == (5, 0, 5000))
    );
    assert_eq!(
        rig.status(),
        DownloadStatus::Failed(DownloadError::Stalled { offset: 0 })
    );
    assert_eq!(rig.download().ended, Some(Duration::from_millis(6000)));
    // It tells the vehicle to stop, once.
    assert_eq!(rig.ends(), vec![6000]);
}

#[test]
fn a_closed_link_ends_a_running_download_at_once_with_the_bytes_so_far() {
    let data = log_bytes(5000);
    let mut rig = Rig::new();
    rig.get(5, 5000, 0);
    rig.frame(5, &data, 0, 10);
    rig.frame(5, &data, 90, 20);
    rig.call(30, |s, cx| s.on_close(cx));
    assert_eq!(
        rig.status(),
        DownloadStatus::Failed(DownloadError::LinkLost { offset: 180 })
    );
    assert_eq!(rig.download().ended, Some(Duration::from_millis(30)));
    assert_eq!(rig.download().done, 180);
    // Nothing more is asked for: the transfer is over.
    rig.tick_range(50, 10_000);
    assert_eq!(rig.requests().len(), 1);
}

#[test]
fn a_closed_link_leaves_a_finished_or_absent_download_alone() {
    let mut rig = Rig::new();
    rig.call(0, |s, cx| s.on_close(cx));
    assert!(rig.state.logs.download.is_none());
    let data = log_bytes(90);
    rig.get(5, 90, 10);
    rig.frame(5, &data, 0, 20);
    rig.call(30, |s, cx| s.on_close(cx));
    assert_eq!(rig.status(), DownloadStatus::Done);
}

#[test]
fn a_log_that_does_not_exist_is_the_same_error_and_an_end_marker_does_not_finish_it() {
    // The vehicle answers a request for a log it does not have with nothing at all. A log that is
    // shorter than listed answers with the zero-count marker: that does not complete it either.
    let data = log_bytes(180);
    let mut rig = Rig::new();
    rig.get(77, 5000, 0);
    rig.serve_last(&data, &[], 10);
    assert_eq!(rig.download().done, 180);
    rig.deliver(1, Rig::data_frame(77, 5000, &[]), 20);
    rig.tick_range(50, 10_000);
    assert_eq!(
        rig.status(),
        DownloadStatus::Failed(DownloadError::Stalled { offset: 180 })
    );
    assert_eq!(rig.requests().len(), 6, "the first, then five retries");
    assert_eq!(rig.ends().len(), 1);
}

#[test]
fn progress_resets_the_retries() {
    let data = log_bytes(5000);
    let mut rig = Rig::new();
    rig.get(5, 5000, 0);
    // Eight rounds in which the vehicle sends one more frame each time and then goes quiet: more
    // re-requests than the five, none of them of the same offset.
    let mut base = 0;
    for round in 0..8u32 {
        rig.frame(5, &data, round * 90, base + 10);
        rig.tick_range(base + 50, base + 1100);
        base += 1100;
        assert_eq!(rig.status(), DownloadStatus::Running, "round {round}");
    }
    assert_eq!(rig.download().done, 8 * 90);
    assert!(rig.requests().len() >= 9, "one retry in each round");
    assert_eq!(rig.requests().last().map(|r| r.2), Some(8 * 90));
    // Then silence: five more retries of that offset and it ends.
    rig.tick_range(base + 50, base + 8000);
    assert_eq!(
        rig.status(),
        DownloadStatus::Failed(DownloadError::Stalled { offset: 8 * 90 })
    );
}

#[test]
fn bytes_that_arrive_beyond_a_gap_do_not_count_as_progress() {
    let data = log_bytes(5000);
    let mut rig = Rig::new();
    rig.get(5, 5000, 0);
    // Frame 0 is lost every time, the rest arrives every time.
    let mut t = 0;
    for _ in 0..6 {
        rig.serve_last(&data, &[0], t + 100);
        t += 1000;
        rig.tick_range(t - 850, t);
    }
    assert_eq!(rig.download().done, 0);
    assert_eq!(
        rig.status(),
        DownloadStatus::Failed(DownloadError::Stalled { offset: 0 })
    );
}

#[test]
fn a_cancel_tells_the_vehicle_to_stop_and_later_frames_are_ignored() {
    let data = log_bytes(5000);
    let mut rig = Rig::new();
    rig.get(5, 5000, 0);
    rig.frame(5, &data, 0, 10);
    rig.command(Command::CancelLogDownload, 20);
    assert_eq!(rig.status(), DownloadStatus::Cancelled);
    assert_eq!(rig.ends(), vec![20]);
    rig.frame(5, &data, 90, 30);
    rig.tick_range(50, 8000);
    assert_eq!(rig.download().done, 90);
    assert_eq!(rig.sink.bytes(), data[..90]);
    assert_eq!(rig.requests().len(), 1);
    // A second cancel has nothing to cancel.
    rig.command(Command::CancelLogDownload, 9000);
    assert_eq!(rig.ends(), vec![20]);
}

#[test]
fn a_writer_that_fails_ends_the_download_and_stops_the_vehicle() {
    let data = log_bytes(5000);
    let mut rig = Rig::new();
    let sink = Shared {
        fail_after: Some(180),
        ..Shared::default()
    };
    rig.get_with(sink, 5, 5000, 0);
    rig.serve_last(&data, &[], 10);
    assert_eq!(
        rig.status(),
        DownloadStatus::Failed(DownloadError::Sink("disk full".into()))
    );
    assert_eq!(rig.sink.bytes().len(), 180);
    assert_eq!(rig.ends(), vec![10]);
    rig.tick_range(50, 8000);
    assert_eq!(rig.requests().len(), 1);
}

#[test]
fn only_one_thing_runs_at_a_time() {
    let mut rig = Rig::new();
    rig.get(5, 5000, 0);
    rig.list(10);
    rig.get(6, 100, 20);
    assert_eq!(rig.log.len(), 1, "{:?}", rig.log);
    assert_eq!(rig.state.logs.refused, 2);
    assert_eq!(rig.download().number, 5);
    assert_eq!(rig.state.logs.list, ListStatus::Idle);

    let mut rig = Rig::new();
    rig.list(0);
    rig.get(5, 5000, 10);
    assert_eq!(rig.log.len(), 1);
    assert_eq!(rig.state.logs.refused, 1);
    assert!(rig.state.logs.download.is_none());
}

#[test]
fn a_download_without_a_vehicle_sends_nothing() {
    let mut rig = Rig::new();
    rig.state.link.status = LinkStatus::Lost;
    rig.get(5, 5000, 0);
    assert!(rig.log.is_empty());
    assert_eq!(
        rig.status(),
        DownloadStatus::Failed(DownloadError::NoVehicle)
    );
}

#[test]
fn a_download_can_follow_a_download() {
    let mut rig = Rig::new();
    let first = log_bytes(300);
    rig.get(5, 300, 0);
    rig.serve_last(&first, &[], 5);
    assert_eq!(rig.status(), DownloadStatus::Done);
    let second = log_bytes(180);
    rig.get_with(Shared::default(), 6, 180, 100);
    rig.serve_last(&second, &[], 105);
    assert_eq!(rig.download().number, 6);
    assert_eq!(rig.status(), DownloadStatus::Done);
    assert_eq!(rig.sink.bytes(), second);
}

#[test]
fn a_final_flush_that_fails_ends_the_download_and_stops_the_vehicle() {
    let data = log_bytes(300);
    let mut rig = Rig::new();
    let sink = Shared {
        fail_flush: true,
        ..Shared::default()
    };
    rig.get_with(sink, 5, 300, 0);
    rig.serve_last(&data, &[], 10);
    assert_eq!(
        rig.status(),
        DownloadStatus::Failed(DownloadError::Sink("flush failed".into()))
    );
    assert_eq!(rig.ends(), vec![10]);
    rig.tick_range(50, 8000);
    assert_eq!(rig.requests().len(), 1);
    assert_eq!(rig.ends().len(), 1);
}
