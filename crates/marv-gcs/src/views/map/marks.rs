//! Pins dropped from the vehicle: the INFO text `MARK <n> <lat_e7> <lon_e7> <hacc_cm>` (or
//! `MARK <n> NOFIX`) the vehicle sends when its mark switch is toggled. `n` counts from 1 per vehicle boot.

use std::collections::HashSet;

use gcs_mavlink::services::texts::{Severity, TextLog};

/// What a MARK text says.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Mark {
    /// Where the vehicle was, in degrees, with its horizontal accuracy in metres.
    Pin {
        n: u32,
        lat: f64,
        lon: f64,
        accuracy_m: f64,
    },
    NoFix {
        n: u32,
    },
}

impl Mark {
    pub fn n(&self) -> u32 {
        match *self {
            Mark::Pin { n, .. } | Mark::NoFix { n } => n,
        }
    }
}

/// The mark in `text`, or `None` when it is not a well-formed MARK text (out of range coordinates included).
pub fn parse_mark(text: &str) -> Option<Mark> {
    let mut words = text.split_whitespace();
    if words.next()? != "MARK" {
        return None;
    }
    let n: u32 = words.next()?.parse().ok().filter(|&n| n >= 1)?;
    let first = words.next()?;
    let mark = if first == "NOFIX" {
        Mark::NoFix { n }
    } else {
        let lat_e7: i32 = first.parse().ok()?;
        let lon_e7: i32 = words.next()?.parse().ok()?;
        let hacc_cm: u32 = words.next()?.parse().ok()?;
        if !(-900_000_000..=900_000_000).contains(&lat_e7)
            || !(-1_800_000_000..=1_800_000_000).contains(&lon_e7)
        {
            return None;
        }
        Mark::Pin {
            n,
            lat: f64::from(lat_e7) / 1e7,
            lon: f64::from(lon_e7) / 1e7,
            accuracy_m: f64::from(hacc_cm) / 100.0,
        }
    };
    words.next().is_none().then_some(mark)
}

/// The words of the visible notice for `mark`.
pub fn notice(mark: &Mark) -> String {
    match *mark {
        Mark::Pin { n, accuracy_m, .. } => {
            format!("Pin {n} dropped from the vehicle, ±{accuracy_m:.1} m")
        }
        Mark::NoFix { n } => format!("Mark {n}: no GPS fix, nothing dropped"),
    }
}

/// Reads the vehicle's texts and hands out each mark once.
#[derive(Debug, Default)]
pub struct Marks {
    /// How many texts of the log were looked at.
    cursor: u64,
    /// The `n` taken since the vehicle's boot.
    seen: HashSet<u32>,
    highest: u32,
}

impl Marks {
    /// A new session: nothing seen.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// The marks in texts that arrived since the last call, oldest first, each `n` once. An `n` of 1 after higher
    /// ones is a vehicle that started again (it counts from 1 per boot): the count starts anew.
    pub fn take(&mut self, texts: &TextLog) -> Vec<Mark> {
        if texts.total < self.cursor {
            self.reset();
        }
        let fresh = usize::try_from(texts.total - self.cursor)
            .unwrap_or(usize::MAX)
            .min(texts.entries.len());
        self.cursor = texts.total;
        let mut out = Vec::new();
        for entry in texts.entries.iter().skip(texts.entries.len() - fresh) {
            if entry.severity != Severity::Info {
                continue;
            }
            let Some(mark) = parse_mark(&entry.text) else {
                continue;
            };
            let n = mark.n();
            if n == 1 && self.highest > 1 {
                self.seen.clear();
                self.highest = 0;
            }
            if self.seen.insert(n) {
                self.highest = self.highest.max(n);
                out.push(mark);
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use gcs_mavlink::services::texts::TextEntry;

    use super::*;

    #[test]
    fn a_mark_with_a_position_is_read_in_degrees_and_metres() {
        assert_eq!(
            parse_mark("MARK 3 261886799 -981234567 120"),
            Some(Mark::Pin {
                n: 3,
                lat: 26.1886799,
                lon: -98.1234567,
                accuracy_m: 1.2
            })
        );
    }

    #[test]
    fn negative_latitudes_and_the_limits_are_read() {
        let Some(Mark::Pin { lat, lon, .. }) = parse_mark("MARK 1 -900000000 1800000000 0") else {
            panic!("not a pin");
        };
        assert_eq!((lat, lon), (-90.0, 180.0));
    }

    #[test]
    fn nofix_has_no_position() {
        assert_eq!(parse_mark("MARK 7 NOFIX"), Some(Mark::NoFix { n: 7 }));
    }

    #[test]
    fn extra_spaces_around_and_between_the_words_are_fine() {
        assert_eq!(parse_mark("  MARK   2  NOFIX  "), Some(Mark::NoFix { n: 2 }));
        assert_eq!(
            parse_mark("MARK  2   10000000  20000000   50 "),
            Some(Mark::Pin {
                n: 2,
                lat: 1.0,
                lon: 2.0,
                accuracy_m: 0.5
            })
        );
    }

    #[test]
    fn anything_else_is_not_a_mark() {
        for text in [
            "",
            "MARK",
            "MARK 3",
            "MARK 0 NOFIX",
            "MARK x NOFIX",
            "MARK -1 NOFIX",
            "MARK 3 261886799 -981234567",
            "MARK 3 261886799 -981234567 120 9",
            "MARK 3 261886799 -981234567 -5",
            "MARK 3 261886799 abc 120",
            "MARK 3 2.6 -9.8 120",
            "MARK 3 NOFIX extra",
            "MARK 3 900000001 0 10",
            "MARK 3 0 1800000001 10",
            "mark 3 NOFIX",
            "MARKS 3 NOFIX",
            "Armed MARK 3 NOFIX",
        ] {
            assert_eq!(parse_mark(text), None, "{text:?}");
        }
    }

    #[test]
    fn the_notice_says_the_accuracy_or_that_nothing_was_dropped() {
        assert_eq!(
            notice(&parse_mark("MARK 3 261886799 -981234567 120").unwrap()),
            "Pin 3 dropped from the vehicle, ±1.2 m"
        );
        assert_eq!(
            notice(&Mark::NoFix { n: 3 }),
            "Mark 3: no GPS fix, nothing dropped"
        );
    }

    fn push(log: &mut TextLog, severity: Severity, text: &str) {
        log.entries.push_back(TextEntry {
            time: Duration::ZERO,
            severity,
            text: text.to_string(),
            system_id: 1,
            component_id: 1,
        });
        log.total += 1;
    }

    #[test]
    fn each_mark_is_handed_out_once_and_only_info_texts_count() {
        let mut log = TextLog::default();
        let mut marks = Marks::default();
        push(&mut log, Severity::Info, "MARK 1 NOFIX");
        push(&mut log, Severity::Warning, "MARK 2 NOFIX");
        push(&mut log, Severity::Info, "hello");
        push(&mut log, Severity::Info, "MARK 3 10000000 20000000 100");
        let got = marks.take(&log);
        assert_eq!(got.iter().map(Mark::n).collect::<Vec<_>>(), [1, 3]);
        assert!(marks.take(&log).is_empty());
        push(&mut log, Severity::Info, "MARK 3 10000000 20000000 100");
        push(&mut log, Severity::Info, "MARK 3 10000000 20000000 100");
        assert!(marks.take(&log).is_empty(), "repeats are not new");
    }

    #[test]
    fn a_vehicle_that_counts_from_1_again_drops_again() {
        let mut log = TextLog::default();
        let mut marks = Marks::default();
        push(&mut log, Severity::Info, "MARK 1 NOFIX");
        push(&mut log, Severity::Info, "MARK 2 NOFIX");
        assert_eq!(marks.take(&log).len(), 2);
        push(&mut log, Severity::Info, "MARK 1 10000000 20000000 100");
        let got = marks.take(&log);
        assert_eq!(got.iter().map(Mark::n).collect::<Vec<_>>(), [1]);
        push(&mut log, Severity::Info, "MARK 2 10000000 20000000 100");
        assert_eq!(marks.take(&log).len(), 1);
    }

    #[test]
    fn a_log_that_dropped_old_entries_still_gives_the_new_ones() {
        let mut log = TextLog::default();
        let mut marks = Marks::default();
        push(&mut log, Severity::Info, "MARK 1 NOFIX");
        marks.take(&log);
        log.entries.clear();
        push(&mut log, Severity::Info, "MARK 2 NOFIX");
        assert_eq!(marks.take(&log).iter().map(Mark::n).collect::<Vec<_>>(), [2]);
    }

    #[test]
    fn a_new_log_starts_the_count_again() {
        let mut log = TextLog::default();
        let mut marks = Marks::default();
        push(&mut log, Severity::Info, "MARK 1 NOFIX");
        push(&mut log, Severity::Info, "MARK 2 NOFIX");
        marks.take(&log);
        let mut log = TextLog::default();
        push(&mut log, Severity::Info, "MARK 2 NOFIX");
        assert_eq!(marks.take(&log).len(), 1);
    }
}
