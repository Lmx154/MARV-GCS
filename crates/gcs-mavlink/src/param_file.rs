//! Parameter files: one `NAME,VALUE` per line, `#` starts a comment (the layout Mission Planner reads).

use std::fmt;

use crate::services::params::{ParamTable, ParamValue};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParamFileError {
    /// 1-based line number.
    pub line: usize,
    pub reason: String,
}

impl fmt::Display for ParamFileError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "line {}: {}", self.line, self.reason)
    }
}

impl std::error::Error for ParamFileError {}

const HEADER: &str = "# MARV GCS parameters";
const MAX_NAME_LEN: usize = 16;

/// The whole table as file text, sorted by name.
pub fn to_text(table: &ParamTable) -> String {
    let mut text = String::from(HEADER);
    text.push('\n');
    let mut names: Vec<&String> = table.params.keys().collect();
    names.sort();
    for name in names {
        let param = &table.params[name];
        text.push_str(&format!("{name},{}\n", param.value(table.encoding)));
    }
    text
}

/// The names and numbers in a file.
pub fn parse(text: &str) -> Result<Vec<(String, f64)>, ParamFileError> {
    let mut entries = Vec::new();
    let text = text.strip_prefix('\u{feff}').unwrap_or(text);
    for (number, line) in text.lines().enumerate() {
        let line_number = number + 1;
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let error = |reason: String| ParamFileError {
            line: line_number,
            reason,
        };
        let name_end = line
            .find(|c: char| c == ',' || c.is_whitespace())
            .unwrap_or(line.len());
        let (name, rest) = line.split_at(name_end);
        if name.is_empty() {
            return Err(error("no parameter name".to_owned()));
        }
        if name.len() > MAX_NAME_LEN {
            return Err(error(format!(
                "name {name} is longer than {MAX_NAME_LEN} characters"
            )));
        }
        let rest = rest.trim_start();
        let rest = rest.strip_prefix(',').unwrap_or(rest).trim();
        if rest.is_empty() {
            return Err(error(format!("no value for {name}")));
        }
        match rest.parse::<f64>() {
            Ok(value) if value.is_finite() => entries.push((name.to_owned(), value)),
            _ => return Err(error(format!("{rest} is not a number"))),
        }
    }
    Ok(entries)
}

/// What a file would change: the name and the new value, in the parameter's own type, for every
/// value that differs from the table; and one line of text per problem (a name the vehicle does not
/// have, a number that does not fit the parameter's type).
pub fn changes(
    table: &ParamTable,
    wanted: &[(String, f64)],
) -> (Vec<(String, ParamValue)>, Vec<String>) {
    let mut changed = Vec::new();
    let mut problems = Vec::new();
    for (name, number) in wanted {
        let Some(param) = table.params.get(name) else {
            problems.push(format!("{name}: the vehicle has no such parameter"));
            continue;
        };
        let Some(value) = param.kind.value_from(*number) else {
            problems.push(format!(
                "{name}: {number} does not fit the type {}",
                param.kind.name()
            ));
            continue;
        };
        if value != param.value(table.encoding) {
            changed.push((name.clone(), value));
        }
    }
    (changed, problems)
}

#[cfg(test)]
#[path = "param_file_tests.rs"]
mod tests;
