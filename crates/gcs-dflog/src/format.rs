//! Format characters and the decoding of one field.

use std::borrow::Cow;

/// What a format character decodes (ArduPilot `LogStructure.h`; scaling as pymavlink's `FORMAT_TO_STRUCT`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    I8,
    U8,
    I16,
    U16,
    I32,
    U32,
    I64,
    U64,
    F16,
    F32,
    F64,
    /// `c`: i16 x 0.01.
    CentiI16,
    /// `C`: u16 x 0.01.
    CentiU16,
    /// `e`: i32 x 0.01.
    CentiI32,
    /// `E`: u32 x 0.01.
    CentiU32,
    /// `L`: i32 x 1e-7 (degrees).
    Degrees,
    Text4,
    Text16,
    Text64,
    /// `a`: i16[32].
    Array32,
}

impl Kind {
    fn from_char(c: u8) -> Option<Kind> {
        Some(match c {
            b'b' => Kind::I8,
            b'B' | b'M' => Kind::U8,
            b'h' => Kind::I16,
            b'H' => Kind::U16,
            b'i' => Kind::I32,
            b'I' => Kind::U32,
            b'q' => Kind::I64,
            b'Q' => Kind::U64,
            b'g' => Kind::F16,
            b'f' => Kind::F32,
            b'd' => Kind::F64,
            b'c' => Kind::CentiI16,
            b'C' => Kind::CentiU16,
            b'e' => Kind::CentiI32,
            b'E' => Kind::CentiU32,
            b'L' => Kind::Degrees,
            b'n' => Kind::Text4,
            b'N' => Kind::Text16,
            b'Z' => Kind::Text64,
            b'a' => Kind::Array32,
            _ => return None,
        })
    }

    fn size(self) -> usize {
        match self {
            Kind::I8 | Kind::U8 => 1,
            Kind::I16 | Kind::U16 | Kind::F16 | Kind::CentiI16 | Kind::CentiU16 => 2,
            Kind::I32
            | Kind::U32
            | Kind::F32
            | Kind::CentiI32
            | Kind::CentiU32
            | Kind::Degrees
            | Kind::Text4 => 4,
            Kind::I64 | Kind::U64 | Kind::F64 => 8,
            Kind::Text16 => 16,
            Kind::Text64 => 64,
            Kind::Array32 => 64,
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct Field {
    kind: Kind,
    offset: usize,
}

/// One decoded field.
#[derive(Clone, Debug, PartialEq)]
pub enum Value<'a> {
    /// Any numeric format character, after scaling.
    Number(f64),
    /// `n`, `N`, `Z`: the bytes up to the first NUL.
    Text(Cow<'a, str>),
    /// `a`: 32 numbers.
    Array([i16; 32]),
}

/// A record type as defined by a FMT record.
#[derive(Clone, Debug)]
pub struct Format {
    /// The type number that starts a record of this type.
    pub type_id: u8,
    /// The length of the whole record, header included.
    pub length: u8,
    pub name: String,
    /// The format characters, one per column.
    pub format: String,
    /// The column names, split at commas.
    pub columns: Vec<String>,
    fields: Vec<Field>,
    time_index: Option<usize>,
}

fn c_string(bytes: &[u8]) -> Cow<'_, str> {
    let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    String::from_utf8_lossy(&bytes[..end])
}

fn le<const N: usize>(bytes: &[u8]) -> [u8; N] {
    let mut out = [0u8; N];
    out.copy_from_slice(&bytes[..N]);
    out
}

fn half_to_f64(h: u16) -> f64 {
    let sign = if h & 0x8000 != 0 { -1.0 } else { 1.0 };
    let exp = i32::from((h >> 10) & 0x1f);
    let frac = f64::from(h & 0x3ff);
    match exp {
        0 => sign * frac * 2f64.powi(-24),
        31 if frac == 0.0 => sign * f64::INFINITY,
        31 => f64::NAN,
        _ => sign * (1.0 + frac / 1024.0) * 2f64.powi(exp - 15),
    }
}

impl Format {
    /// The FMT record's own definition, which is built in.
    pub(crate) fn fmt_of_fmt() -> Format {
        Format::new(128, 89, "FMT", "BBnNZ", "Type,Length,Name,Format,Columns")
            .expect("the built-in FMT definition is valid")
    }

    /// `None` when a format character is unknown or `length` is not 3 plus the size of the fields.
    pub(crate) fn new(
        type_id: u8,
        length: u8,
        name: &str,
        format: &str,
        columns: &str,
    ) -> Option<Format> {
        let mut fields = Vec::with_capacity(format.len());
        let mut offset = 0;
        for c in format.bytes() {
            let kind = Kind::from_char(c)?;
            fields.push(Field { kind, offset });
            offset += kind.size();
        }
        if usize::from(length) != offset + 3 {
            return None;
        }
        let columns: Vec<String> = if columns.is_empty() {
            Vec::new()
        } else {
            columns.split(',').map(str::to_owned).collect()
        };
        let time_index = columns
            .iter()
            .position(|c| c == "TimeUS")
            .filter(|&i| i < fields.len() && !matches!(fields[i].kind, Kind::Text4 | Kind::Text16 | Kind::Text64 | Kind::Array32));
        Some(Format {
            type_id,
            length,
            name: name.to_owned(),
            format: format.to_owned(),
            columns,
            fields,
            time_index,
        })
    }

    /// Parses the payload of a FMT record (86 bytes); `None` if it defines nothing usable.
    pub(crate) fn from_fmt_payload(p: &[u8]) -> Option<Format> {
        if p.len() < 86 {
            return None;
        }
        Format::new(
            p[0],
            p[1],
            &c_string(&p[2..6]),
            &c_string(&p[6..22]),
            &c_string(&p[22..86]),
        )
    }

    /// The number of fields (one per format character).
    pub fn field_count(&self) -> usize {
        self.fields.len()
    }

    /// The index of the column called `name`, if the format has one.
    pub fn column_index(&self, name: &str) -> Option<usize> {
        self.columns
            .iter()
            .position(|c| c == name)
            .filter(|&i| i < self.fields.len())
    }

    /// The index of the `TimeUS` column, if it is numeric.
    pub fn time_index(&self) -> Option<usize> {
        self.time_index
    }

    pub(crate) fn number(&self, payload: &[u8], index: usize) -> Option<f64> {
        let f = self.fields.get(index)?;
        let b = payload.get(f.offset..f.offset + f.kind.size())?;
        Some(match f.kind {
            Kind::I8 => f64::from(b[0] as i8),
            Kind::U8 => f64::from(b[0]),
            Kind::I16 => f64::from(i16::from_le_bytes(le(b))),
            Kind::U16 => f64::from(u16::from_le_bytes(le(b))),
            Kind::I32 => f64::from(i32::from_le_bytes(le(b))),
            Kind::U32 => f64::from(u32::from_le_bytes(le(b))),
            Kind::I64 => i64::from_le_bytes(le(b)) as f64,
            Kind::U64 => u64::from_le_bytes(le(b)) as f64,
            Kind::F16 => half_to_f64(u16::from_le_bytes(le(b))),
            Kind::F32 => f64::from(f32::from_le_bytes(le(b))),
            Kind::F64 => f64::from_le_bytes(le(b)),
            Kind::CentiI16 => f64::from(i16::from_le_bytes(le(b))) * 0.01,
            Kind::CentiU16 => f64::from(u16::from_le_bytes(le(b))) * 0.01,
            Kind::CentiI32 => f64::from(i32::from_le_bytes(le(b))) * 0.01,
            Kind::CentiU32 => f64::from(u32::from_le_bytes(le(b))) * 0.01,
            Kind::Degrees => f64::from(i32::from_le_bytes(le(b))) * 1.0e-7,
            Kind::Text4 | Kind::Text16 | Kind::Text64 | Kind::Array32 => return None,
        })
    }

    pub(crate) fn value<'a>(&self, payload: &'a [u8], index: usize) -> Option<Value<'a>> {
        let f = self.fields.get(index)?;
        let b = payload.get(f.offset..f.offset + f.kind.size())?;
        match f.kind {
            Kind::Text4 | Kind::Text16 | Kind::Text64 => Some(Value::Text(c_string(b))),
            Kind::Array32 => {
                let mut a = [0i16; 32];
                for (slot, pair) in a.iter_mut().zip(b.chunks_exact(2)) {
                    *slot = i16::from_le_bytes(le(pair));
                }
                Some(Value::Array(a))
            }
            _ => self.number(payload, index).map(Value::Number),
        }
    }
}
