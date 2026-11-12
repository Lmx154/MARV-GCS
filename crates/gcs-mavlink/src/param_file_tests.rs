use super::*;
use crate::services::params::{Param, ParamEncoding, ParamType};

fn param(name: &str, index: u16, kind: ParamType, raw: [u8; 4]) -> Param {
    Param {
        name: name.to_owned(),
        index,
        kind,
        raw,
    }
}

/// Three parameters, byte-wise: an I32 (-3), a U8 (200) and an F32 (1.5).
fn table() -> ParamTable {
    let mut table = ParamTable {
        encoding: ParamEncoding::Bytewise,
        encoding_known: true,
        ..ParamTable::default()
    };
    for p in [
        param("RATE_MAX", 2, ParamType::F32, [0x00, 0x00, 0xC0, 0x3F]),
        param("GAIN_OFFSET", 0, ParamType::I32, [0xFD, 0xFF, 0xFF, 0xFF]),
        param("MODE", 1, ParamType::U8, [0xC8, 0, 0, 0]),
    ] {
        table.params.insert(p.name.clone(), p);
    }
    table
}

fn error(text: &str) -> ParamFileError {
    parse(text).unwrap_err()
}

#[test]
fn to_text_is_the_literal_expected_file() {
    assert_eq!(
        to_text(&table()),
        "# MARV GCS parameters\nGAIN_OFFSET,-3\nMODE,200\nRATE_MAX,1.5\n"
    );
}

#[test]
fn to_text_decodes_with_the_table_encoding() {
    let mut table = table();
    table.encoding = ParamEncoding::CCast;
    table.params.clear();
    let p = param("A", 0, ParamType::I32, (-3.0f32).to_le_bytes());
    table.params.insert("A".to_owned(), p);
    assert_eq!(to_text(&table), "# MARV GCS parameters\nA,-3\n");
}

#[test]
fn to_text_of_an_empty_table_is_the_header_only() {
    assert_eq!(to_text(&ParamTable::default()), "# MARV GCS parameters\n");
}

#[test]
fn text_parse_changes_round_trip_is_empty() {
    let table = table();
    let wanted = parse(&to_text(&table)).unwrap();
    assert_eq!(wanted.len(), 3);
    let (changed, problems) = changes(&table, &wanted);
    assert!(changed.is_empty(), "{changed:?}");
    assert!(problems.is_empty(), "{problems:?}");
}

#[test]
fn round_trip_keeps_awkward_floats() {
    let mut table = ParamTable::default();
    for (i, v) in [0.1f32, 1.0e-10, 3.4028235e38, -123456.78]
        .into_iter()
        .enumerate()
    {
        let name = format!("F{i}");
        let p = param(&name, i as u16, ParamType::F32, v.to_le_bytes());
        table.params.insert(name, p);
    }
    let wanted = parse(&to_text(&table)).unwrap();
    let (changed, problems) = changes(&table, &wanted);
    assert!(
        changed.is_empty() && problems.is_empty(),
        "{changed:?} {problems:?}"
    );
}

#[test]
fn an_edited_line_is_one_change_in_the_parameters_own_type() {
    let table = table();
    let text = to_text(&table).replace("MODE,200", "MODE,7");
    let (changed, problems) = changes(&table, &parse(&text).unwrap());
    assert_eq!(changed, vec![("MODE".to_owned(), ParamValue::U8(7))]);
    assert!(problems.is_empty());

    let text = to_text(&table).replace("RATE_MAX,1.5", "RATE_MAX,2");
    let (changed, _) = changes(&table, &parse(&text).unwrap());
    assert_eq!(changed, vec![("RATE_MAX".to_owned(), ParamValue::F32(2.0))]);
}

#[test]
fn an_unknown_name_and_a_number_that_does_not_fit_give_one_problem_line_each() {
    let table = table();
    let wanted = vec![
        ("NOPE".to_owned(), 1.0),
        ("MODE".to_owned(), 300.0),
        ("GAIN_OFFSET".to_owned(), 1.5),
    ];
    let (changed, problems) = changes(&table, &wanted);
    assert!(changed.is_empty());
    assert_eq!(problems.len(), 3, "{problems:?}");
    assert!(problems[0].starts_with("NOPE"), "{problems:?}");
    assert!(
        problems[1].starts_with("MODE") && problems[1].contains("u8"),
        "{problems:?}"
    );
    assert!(
        problems[2].starts_with("GAIN_OFFSET") && problems[2].contains("i32"),
        "{problems:?}"
    );
}

#[test]
fn problems_do_not_hide_valid_changes() {
    let wanted = vec![("NOPE".to_owned(), 1.0), ("MODE".to_owned(), 1.0)];
    let (changed, problems) = changes(&table(), &wanted);
    assert_eq!(changed, vec![("MODE".to_owned(), ParamValue::U8(1))]);
    assert_eq!(problems.len(), 1);
}

#[test]
fn parse_ignores_blank_lines_and_comments_and_accepts_both_separators() {
    let text = "\n# a comment\n   \nA,1\nB 2\nC\t3\n  D ,  4  \nE , 5.5\n   # indented comment\n";
    assert_eq!(
        parse(text).unwrap(),
        vec![
            ("A".to_owned(), 1.0),
            ("B".to_owned(), 2.0),
            ("C".to_owned(), 3.0),
            ("D".to_owned(), 4.0),
            ("E".to_owned(), 5.5),
        ]
    );
}

#[test]
fn parse_accepts_windows_line_ends_and_negative_and_exponent_numbers() {
    assert_eq!(
        parse("A,-2\r\nB,1e3\r\n").unwrap(),
        vec![("A".to_owned(), -2.0), ("B".to_owned(), 1000.0)]
    );
}

#[test]
fn parse_accepts_a_name_of_exactly_16_characters() {
    let wanted = parse("ABCDEFGHIJKLMNOP,1").unwrap();
    assert_eq!(wanted, vec![("ABCDEFGHIJKLMNOP".to_owned(), 1.0)]);
}

#[test]
fn parse_errors_carry_the_one_based_line_number() {
    let e = error("# header\nA,1\nB\n");
    assert_eq!(e.line, 3);
    let e = error("A,1\n\nB,\n");
    assert_eq!(e.line, 3);
    let e = error("A 1\nB abc\n");
    assert_eq!(e.line, 2);
    let e = error("A,1\nB,1\nABCDEFGHIJKLMNOPQ,1\n");
    assert_eq!(e.line, 3);
    assert!(e.reason.contains("ABCDEFGHIJKLMNOPQ"), "{}", e.reason);
    assert!(e.to_string().starts_with("line 3: "));
}

#[test]
fn parse_refuses_non_numbers_and_non_finite_numbers() {
    for bad in ["A,x", "A,1,2", "A,NaN", "A,inf", "A,1.2.3", "A 0x10"] {
        assert_eq!(error(bad).line, 1, "{bad}");
    }
}

#[test]
fn parse_refuses_a_line_without_a_name() {
    assert_eq!(error("A,1\n,5\n").line, 2);
}

#[test]
fn parse_ignores_a_byte_order_mark_at_the_start() {
    assert_eq!(
        parse("\u{feff}A,1\nB,2\n").unwrap(),
        vec![("A".to_owned(), 1.0), ("B".to_owned(), 2.0)]
    );
    assert_eq!(
        parse("\u{feff}# MARV GCS parameters\nA,1\n").unwrap(),
        vec![("A".to_owned(), 1.0)]
    );
    assert_eq!(parse("\u{feff}").unwrap(), vec![]);
}

#[test]
fn the_text_of_an_f32_holding_f32_max_reads_back_as_no_change() {
    let mut table = ParamTable::default();
    let p = param("BIG", 0, ParamType::F32, f32::MAX.to_le_bytes());
    table.params.insert("BIG".to_owned(), p);
    let text = to_text(&table);
    assert_eq!(
        text,
        "# MARV GCS parameters\nBIG,340282350000000000000000000000000000000\n"
    );
    let wanted = parse(&text).unwrap();
    let (changed, problems) = changes(&table, &wanted);
    assert!(changed.is_empty(), "{changed:?}");
    assert!(problems.is_empty(), "{problems:?}");
}
