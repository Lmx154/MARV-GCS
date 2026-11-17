//! The condition search: a small expression language over the fields of a log's records, and one pass that
//! returns the intervals where the condition held. The semantics are pymavlink's `recv_match(condition=...)`
//! (`DFReader.py`, `mavutil.evaluate_condition`): after each record of a type the condition names, it is
//! evaluated over the LATEST record of each named type, and it is false until every named type has appeared.
//!
//! # Syntax
//!
//! ```text
//! condition  = or
//! or         = and { "or" and }
//! and        = not { "and" not }
//! not        = "not" not | comparison
//! comparison = sum [ ("<" | "<=" | ">" | ">=" | "==" | "!=") sum ]
//! sum        = product { ("+" | "-") product }
//! product    = unary { ("*" | "/") unary }
//! unary      = "-" unary | number | "abs" "(" or ")" | reference | "(" or ")"
//! reference  = TYPE "." Field | TYPE "[" n "]" "." Field
//! ```
//!
//! Names are those of the log's FMT records (case matters); numbers are integers, decimals and exponents
//! (`15`, `14.8`, `1e-3`). A comparison is not chained (`a < b < c` is an error), and the two sides of a
//! comparison, the operands of `+ - * /` and of `abs` are numbers while those of `and`, `or` and `not` are
//! comparisons, so `BAT.Volt` alone, or `not BAT.Volt`, is an error (reported with its position).
//!
//! Examples:
//!
//! - `BAT.Volt < 14.8`: the battery voltage is low.
//! - `IMU[0].AccZ < -15`: the first IMU's vertical acceleration is below -15 (instance 0 only).
//! - `DEV[7].Errs > 100 and GPS.Status < 3`: the radio receiver's error counter is above 100 and the GPS has
//!   no 3D fix.
//!
//! `TYPE[n]` is the latest record of `TYPE` whose instance field equals `n`; the instance field is the first
//! column after `TimeUS` named `I`, `Inst`, `Instance` or `Id` (a type without one cannot be used with `[n]`);
//! plain `TYPE.Field` is the latest record of any instance.
//!
//! Division by zero is NaN, and every comparison with a NaN (`!=` included) is false; `not` of that false
//! comparison is true. Binding ([`Condition::bind`]) reports before the pass an unknown type or field, a text
//! or array field used as a number, a `[n]` on a type without an instance field and a type without `TimeUS`.

use std::fmt;
use std::io::{self, Read};

use crate::format::Format;
use crate::reader::{Control, Progress, Reader, Record, scan};

/// At most this many intervals are kept; the count of the further ones is in [`Found::not_kept`].
pub const MAX_INTERVALS: usize = 10_000;

const INSTANCE_COLUMNS: [&str; 4] = ["I", "Inst", "Instance", "Id"];

/// A syntax or binding error: where (a character offset into the text) and why.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ConditionError {
    pub position: usize,
    pub reason: String,
}

impl fmt::Display for ConditionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "at character {}: {}", self.position, self.reason)
    }
}

impl std::error::Error for ConditionError {}

fn error<T>(position: usize, reason: impl Into<String>) -> Result<T, ConditionError> {
    Err(ConditionError {
        position,
        reason: reason.into(),
    })
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Arith {
    Add,
    Sub,
    Mul,
    Div,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Compare {
    Lt,
    Le,
    Gt,
    Ge,
    Eq,
    Ne,
}

/// A numeric expression; `R` is what a reference to a field is (a name before binding, a slot after).
#[derive(Clone, Debug)]
enum Num<R> {
    Lit(f64),
    Ref(R),
    Neg(Box<Num<R>>),
    Abs(Box<Num<R>>),
    Bin(Arith, Box<Num<R>>, Box<Num<R>>),
}

#[derive(Clone, Debug)]
enum Cond<R> {
    Cmp(Compare, Num<R>, Num<R>),
    Not(Box<Cond<R>>),
    And(Box<Cond<R>>, Box<Cond<R>>),
    Or(Box<Cond<R>>, Box<Cond<R>>),
}

impl<R> Num<R> {
    fn try_map<S, E>(&self, f: &mut impl FnMut(&R) -> Result<S, E>) -> Result<Num<S>, E> {
        Ok(match self {
            Num::Lit(x) => Num::Lit(*x),
            Num::Ref(r) => Num::Ref(f(r)?),
            Num::Neg(a) => Num::Neg(Box::new(a.try_map(f)?)),
            Num::Abs(a) => Num::Abs(Box::new(a.try_map(f)?)),
            Num::Bin(op, a, b) => {
                let a = a.try_map(f)?;
                Num::Bin(*op, Box::new(a), Box::new(b.try_map(f)?))
            }
        })
    }
}

impl<R> Cond<R> {
    fn try_map<S, E>(&self, f: &mut impl FnMut(&R) -> Result<S, E>) -> Result<Cond<S>, E> {
        Ok(match self {
            Cond::Cmp(op, a, b) => {
                let a = a.try_map(f)?;
                Cond::Cmp(*op, a, b.try_map(f)?)
            }
            Cond::Not(a) => Cond::Not(Box::new(a.try_map(f)?)),
            Cond::And(a, b) => {
                let a = a.try_map(f)?;
                Cond::And(Box::new(a), Box::new(b.try_map(f)?))
            }
            Cond::Or(a, b) => {
                let a = a.try_map(f)?;
                Cond::Or(Box::new(a), Box::new(b.try_map(f)?))
            }
        })
    }
}

impl Num<usize> {
    fn eval(&self, values: &[f64]) -> f64 {
        match self {
            Num::Lit(x) => *x,
            Num::Ref(slot) => values[*slot],
            Num::Neg(a) => -a.eval(values),
            Num::Abs(a) => a.eval(values).abs(),
            Num::Bin(op, a, b) => {
                let (a, b) = (a.eval(values), b.eval(values));
                match op {
                    Arith::Add => a + b,
                    Arith::Sub => a - b,
                    Arith::Mul => a * b,
                    Arith::Div if b == 0.0 => f64::NAN,
                    Arith::Div => a / b,
                }
            }
        }
    }
}

impl Cond<usize> {
    fn eval(&self, values: &[f64]) -> bool {
        match self {
            Cond::Cmp(op, a, b) => {
                let (a, b) = (a.eval(values), b.eval(values));
                if a.is_nan() || b.is_nan() {
                    return false;
                }
                match op {
                    Compare::Lt => a < b,
                    Compare::Le => a <= b,
                    Compare::Gt => a > b,
                    Compare::Ge => a >= b,
                    Compare::Eq => a == b,
                    Compare::Ne => a != b,
                }
            }
            Cond::Not(a) => !a.eval(values),
            Cond::And(a, b) => a.eval(values) && b.eval(values),
            Cond::Or(a, b) => a.eval(values) || b.eval(values),
        }
    }
}

/// A reference to a field as written: `TYPE.Field` or `TYPE[n].Field`.
#[derive(Clone, Debug)]
struct RefAst {
    type_name: String,
    type_pos: usize,
    /// The instance and the position of its `[`.
    instance: Option<(u32, usize)>,
    field: String,
    field_pos: usize,
}

// ---- lexer ----

#[derive(Clone, Debug, PartialEq)]
enum Tok {
    Num(String),
    Ident(String),
    Plus,
    Minus,
    Star,
    Slash,
    Lt,
    Le,
    Gt,
    Ge,
    EqEq,
    Ne,
    LParen,
    RParen,
    LBracket,
    RBracket,
    Dot,
}

impl Tok {
    fn describe(&self) -> String {
        match self {
            Tok::Num(s) => format!("the number {s}"),
            Tok::Ident(s) => format!("'{s}'"),
            Tok::Plus => "'+'".into(),
            Tok::Minus => "'-'".into(),
            Tok::Star => "'*'".into(),
            Tok::Slash => "'/'".into(),
            Tok::Lt => "'<'".into(),
            Tok::Le => "'<='".into(),
            Tok::Gt => "'>'".into(),
            Tok::Ge => "'>='".into(),
            Tok::EqEq => "'=='".into(),
            Tok::Ne => "'!='".into(),
            Tok::LParen => "'('".into(),
            Tok::RParen => "')'".into(),
            Tok::LBracket => "'['".into(),
            Tok::RBracket => "']'".into(),
            Tok::Dot => "'.'".into(),
        }
    }
}

struct Token {
    tok: Tok,
    pos: usize,
}

fn lex(chars: &[char]) -> Result<Vec<Token>, ConditionError> {
    let mut out = Vec::new();
    let mut i = 0;
    while i < chars.len() {
        let c = chars[i];
        let pos = i;
        let next = chars.get(i + 1).copied();
        let tok = if c.is_whitespace() {
            i += 1;
            continue;
        } else if c.is_ascii_digit() || (c == '.' && next.is_some_and(|n| n.is_ascii_digit())) {
            while i < chars.len() && chars[i].is_ascii_digit() {
                i += 1;
            }
            if i < chars.len() && chars[i] == '.' {
                i += 1;
                while i < chars.len() && chars[i].is_ascii_digit() {
                    i += 1;
                }
            }
            if i < chars.len() && (chars[i] == 'e' || chars[i] == 'E') {
                let mut j = i + 1;
                if j < chars.len() && (chars[j] == '+' || chars[j] == '-') {
                    j += 1;
                }
                if j >= chars.len() || !chars[j].is_ascii_digit() {
                    return error(i, "the exponent of a number has no digits");
                }
                while j < chars.len() && chars[j].is_ascii_digit() {
                    j += 1;
                }
                i = j;
            }
            Tok::Num(chars[pos..i].iter().collect())
        } else if c.is_ascii_alphabetic() || c == '_' {
            while i < chars.len() && (chars[i].is_ascii_alphanumeric() || chars[i] == '_') {
                i += 1;
            }
            Tok::Ident(chars[pos..i].iter().collect())
        } else {
            i += 1;
            match (c, next) {
                ('+', _) => Tok::Plus,
                ('-', _) => Tok::Minus,
                ('*', _) => Tok::Star,
                ('/', _) => Tok::Slash,
                ('(', _) => Tok::LParen,
                (')', _) => Tok::RParen,
                ('[', _) => Tok::LBracket,
                (']', _) => Tok::RBracket,
                ('.', _) => Tok::Dot,
                ('<', Some('=')) => {
                    i += 1;
                    Tok::Le
                }
                ('<', _) => Tok::Lt,
                ('>', Some('=')) => {
                    i += 1;
                    Tok::Ge
                }
                ('>', _) => Tok::Gt,
                ('=', Some('=')) => {
                    i += 1;
                    Tok::EqEq
                }
                ('=', _) => return error(pos, "'=' is not an operator; write '==' to compare"),
                ('!', Some('=')) => {
                    i += 1;
                    Tok::Ne
                }
                ('!', _) => return error(pos, "'!' is not an operator; write 'not' or '!='"),
                (c, _) => return error(pos, format!("unexpected character '{c}'")),
            }
        };
        out.push(Token { tok, pos });
    }
    Ok(out)
}

// ---- parser ----

enum Item {
    N(Num<RefAst>),
    C(Cond<RefAst>),
}

fn want_num(item: Item, start: usize) -> Result<Num<RefAst>, ConditionError> {
    match item {
        Item::N(n) => Ok(n),
        Item::C(_) => error(start, "a number is expected here, not a comparison"),
    }
}

fn want_cond(item: Item, start: usize) -> Result<Cond<RefAst>, ConditionError> {
    match item {
        Item::C(c) => Ok(c),
        Item::N(_) => error(
            start,
            "a comparison is expected here (compare with < <= > >= == !=), not a number",
        ),
    }
}

struct Parser {
    toks: Vec<Token>,
    i: usize,
    /// The length of the text in characters: where "the end" is.
    end: usize,
}

impl Parser {
    fn peek(&self) -> Option<&Tok> {
        self.toks.get(self.i).map(|t| &t.tok)
    }

    fn pos(&self) -> usize {
        self.toks.get(self.i).map_or(self.end, |t| t.pos)
    }

    fn at_keyword(&self, word: &str) -> bool {
        matches!(self.peek(), Some(Tok::Ident(s)) if s == word)
    }

    fn unexpected<T>(&self) -> Result<T, ConditionError> {
        match self.peek() {
            Some(t) => error(self.pos(), format!("unexpected {}", t.describe())),
            None => error(self.end, "the condition ends too early"),
        }
    }

    fn expect(&mut self, tok: &Tok, what: &str) -> Result<(), ConditionError> {
        if self.peek() == Some(tok) {
            self.i += 1;
            Ok(())
        } else if self.peek().is_none() {
            error(self.end, format!("{what} is expected, the condition ends"))
        } else {
            error(
                self.pos(),
                format!(
                    "{what} is expected, found {}",
                    self.peek().map(Tok::describe).unwrap_or_default()
                ),
            )
        }
    }

    fn or(&mut self) -> Result<(Item, usize), ConditionError> {
        let (mut left, start) = self.and()?;
        while self.at_keyword("or") {
            self.i += 1;
            let l = want_cond(left, start)?;
            let rstart = self.pos();
            let (right, _) = self.and()?;
            left = Item::C(Cond::Or(Box::new(l), Box::new(want_cond(right, rstart)?)));
        }
        Ok((left, start))
    }

    fn and(&mut self) -> Result<(Item, usize), ConditionError> {
        let (mut left, start) = self.not()?;
        while self.at_keyword("and") {
            self.i += 1;
            let l = want_cond(left, start)?;
            let rstart = self.pos();
            let (right, _) = self.not()?;
            left = Item::C(Cond::And(Box::new(l), Box::new(want_cond(right, rstart)?)));
        }
        Ok((left, start))
    }

    fn not(&mut self) -> Result<(Item, usize), ConditionError> {
        let start = self.pos();
        if self.at_keyword("not") {
            self.i += 1;
            let ostart = self.pos();
            let (inner, _) = self.not()?;
            return Ok((
                Item::C(Cond::Not(Box::new(want_cond(inner, ostart)?))),
                start,
            ));
        }
        self.comparison()
    }

    fn comparison(&mut self) -> Result<(Item, usize), ConditionError> {
        let (left, start) = self.sum()?;
        let op = match self.peek() {
            Some(Tok::Lt) => Compare::Lt,
            Some(Tok::Le) => Compare::Le,
            Some(Tok::Gt) => Compare::Gt,
            Some(Tok::Ge) => Compare::Ge,
            Some(Tok::EqEq) => Compare::Eq,
            Some(Tok::Ne) => Compare::Ne,
            _ => return Ok((left, start)),
        };
        self.i += 1;
        let l = want_num(left, start)?;
        let rstart = self.pos();
        let (right, _) = self.sum()?;
        let r = want_num(right, rstart)?;
        if matches!(
            self.peek(),
            Some(Tok::Lt | Tok::Le | Tok::Gt | Tok::Ge | Tok::EqEq | Tok::Ne)
        ) {
            return error(
                self.pos(),
                "comparisons cannot be chained; join them with 'and'",
            );
        }
        Ok((Item::C(Cond::Cmp(op, l, r)), start))
    }

    fn sum(&mut self) -> Result<(Item, usize), ConditionError> {
        let (mut left, start) = self.product()?;
        while let Some(op) = match self.peek() {
            Some(Tok::Plus) => Some(Arith::Add),
            Some(Tok::Minus) => Some(Arith::Sub),
            _ => None,
        } {
            self.i += 1;
            let l = want_num(left, start)?;
            let rstart = self.pos();
            let (right, _) = self.product()?;
            left = Item::N(Num::Bin(
                op,
                Box::new(l),
                Box::new(want_num(right, rstart)?),
            ));
        }
        Ok((left, start))
    }

    fn product(&mut self) -> Result<(Item, usize), ConditionError> {
        let (mut left, start) = self.unary()?;
        while let Some(op) = match self.peek() {
            Some(Tok::Star) => Some(Arith::Mul),
            Some(Tok::Slash) => Some(Arith::Div),
            _ => None,
        } {
            self.i += 1;
            let l = want_num(left, start)?;
            let rstart = self.pos();
            let (right, _) = self.unary()?;
            left = Item::N(Num::Bin(
                op,
                Box::new(l),
                Box::new(want_num(right, rstart)?),
            ));
        }
        Ok((left, start))
    }

    fn unary(&mut self) -> Result<(Item, usize), ConditionError> {
        let start = self.pos();
        if self.peek() == Some(&Tok::Minus) {
            self.i += 1;
            let ostart = self.pos();
            let (inner, _) = self.unary()?;
            return Ok((Item::N(Num::Neg(Box::new(want_num(inner, ostart)?))), start));
        }
        self.primary()
    }

    fn primary(&mut self) -> Result<(Item, usize), ConditionError> {
        let start = self.pos();
        let Some(tok) = self.peek().cloned() else {
            return error(self.end, "the condition ends too early");
        };
        match tok {
            Tok::Num(text) => {
                self.i += 1;
                match text.parse::<f64>() {
                    Ok(x) => Ok((Item::N(Num::Lit(x)), start)),
                    Err(_) => error(start, format!("'{text}' is not a number")),
                }
            }
            Tok::LParen => {
                self.i += 1;
                let (inner, istart) = self.or()?;
                self.expect(&Tok::RParen, "')'")?;
                Ok((inner, istart.min(start)))
            }
            Tok::Ident(word) if matches!(word.as_str(), "and" | "or" | "not") => {
                error(start, format!("unexpected '{word}'"))
            }
            Tok::Ident(word) => {
                self.i += 1;
                if self.peek() == Some(&Tok::LParen) {
                    if word != "abs" {
                        return error(
                            start,
                            format!("unknown function '{word}' (only abs is available)"),
                        );
                    }
                    self.i += 1;
                    let istart = self.pos();
                    let (inner, _) = self.or()?;
                    let inner = want_num(inner, istart)?;
                    self.expect(&Tok::RParen, "')'")?;
                    return Ok((Item::N(Num::Abs(Box::new(inner))), start));
                }
                let instance = if self.peek() == Some(&Tok::LBracket) {
                    let bracket = self.pos();
                    self.i += 1;
                    let npos = self.pos();
                    let n = match self.peek() {
                        Some(Tok::Num(t)) if t.bytes().all(|b| b.is_ascii_digit()) => t
                            .parse::<u32>()
                            .or_else(|_| error(npos, "the instance number is too large"))?,
                        _ => return error(npos, "a whole number is expected as the instance"),
                    };
                    self.i += 1;
                    self.expect(&Tok::RBracket, "']'")?;
                    Some((n, bracket))
                } else {
                    None
                };
                if self.peek() != Some(&Tok::Dot) {
                    return error(
                        self.pos(),
                        format!("'.' and a field name are expected after the record type '{word}'"),
                    );
                }
                self.i += 1;
                let field_pos = self.pos();
                let field = match self.peek() {
                    Some(Tok::Ident(f)) => f.clone(),
                    _ => return error(field_pos, "a field name is expected after '.'"),
                };
                self.i += 1;
                Ok((
                    Item::N(Num::Ref(RefAst {
                        type_name: word,
                        type_pos: start,
                        instance,
                        field,
                        field_pos,
                    })),
                    start,
                ))
            }
            _ => self.unexpected(),
        }
    }
}

/// A condition that parsed: not yet tied to a log's record types.
#[derive(Clone, Debug)]
pub struct Condition {
    cond: Cond<RefAst>,
}

impl Condition {
    /// Parses `text`; an error carries the character offset where the text goes wrong.
    pub fn parse(text: &str) -> Result<Condition, ConditionError> {
        let chars: Vec<char> = text.chars().collect();
        let toks = lex(&chars)?;
        if toks.is_empty() {
            return error(0, "the condition is empty");
        }
        let mut p = Parser {
            toks,
            i: 0,
            end: chars.len(),
        };
        let (item, start) = p.or()?;
        if p.i < p.toks.len() {
            return p.unexpected();
        }
        Ok(Condition {
            cond: want_cond(item, start)?,
        })
    }

    /// Ties the condition to the record types of a log (`Summary::formats`): an unknown type or field, a
    /// text or array field used as a number, `[n]` on a type without an instance field, and a type without a
    /// numeric `TimeUS` are errors, reported here, before any pass.
    pub fn bind(&self, formats: &[Format]) -> Result<BoundCondition, ConditionError> {
        let mut binder = Binder {
            formats,
            sources: Vec::new(),
            slots: Vec::new(),
        };
        let cond = self.cond.try_map(&mut |r| binder.slot(r))?;
        let mut by_type = vec![Vec::new(); 256];
        for (i, s) in binder.sources.iter().enumerate() {
            by_type[usize::from(s.type_id)].push(i);
        }
        Ok(BoundCondition {
            cond,
            sources: binder.sources,
            slots: binder.slots,
            by_type,
        })
    }
}

fn is_number_char(format: &Format, index: usize) -> bool {
    !matches!(
        format.format.as_bytes().get(index),
        Some(b'n' | b'N' | b'Z' | b'a') | None
    )
}

struct SourceSpec {
    type_id: u8,
    type_name: String,
    /// The instance column and the value it must have.
    instance: Option<(usize, f64)>,
    slots: Vec<usize>,
}

struct SlotSpec {
    column: usize,
    label: String,
}

struct Binder<'a> {
    formats: &'a [Format],
    sources: Vec<SourceSpec>,
    slots: Vec<SlotSpec>,
}

impl Binder<'_> {
    fn slot(&mut self, r: &RefAst) -> Result<usize, ConditionError> {
        let Some(format) = self.formats.iter().find(|f| f.name == r.type_name) else {
            return error(
                r.type_pos,
                format!("the log has no record type '{}'", r.type_name),
            );
        };
        if format.time_index().is_none() {
            return error(
                r.type_pos,
                format!("the record type '{}' has no TimeUS", r.type_name),
            );
        }
        let Some(column) = format.column_index(&r.field) else {
            return error(
                r.field_pos,
                format!("'{}' has no field '{}'", r.type_name, r.field),
            );
        };
        if !is_number_char(format, column) {
            return error(
                r.field_pos,
                format!(
                    "the field '{}' of '{}' is text, not a number",
                    r.field, r.type_name
                ),
            );
        }
        let instance = match r.instance {
            None => None,
            Some((n, pos)) => {
                let time = format.time_index().unwrap_or(0);
                let col = (time + 1..format.field_count()).find(|&i| {
                    INSTANCE_COLUMNS.contains(&format.columns[i].as_str())
                        && is_number_char(format, i)
                });
                match col {
                    Some(c) => Some((c, f64::from(n))),
                    None => {
                        return error(
                            pos,
                            format!(
                                "'{}' has no instance field (I, Inst, Instance or Id), so [n] cannot be used",
                                r.type_name
                            ),
                        );
                    }
                }
            }
        };
        let source = match self
            .sources
            .iter()
            .position(|s| s.type_id == format.type_id && s.instance == instance)
        {
            Some(i) => i,
            None => {
                self.sources.push(SourceSpec {
                    type_id: format.type_id,
                    type_name: format.name.clone(),
                    instance,
                    slots: Vec::new(),
                });
                self.sources.len() - 1
            }
        };
        if let Some(&slot) = self.sources[source]
            .slots
            .iter()
            .find(|&&s| self.slots[s].column == column)
        {
            return Ok(slot);
        }
        let label = match r.instance {
            Some((n, _)) => format!("{}[{n}].{}", r.type_name, r.field),
            None => format!("{}.{}", r.type_name, r.field),
        };
        self.slots.push(SlotSpec { column, label });
        let slot = self.slots.len() - 1;
        self.sources[source].slots.push(slot);
        Ok(slot)
    }
}

/// A condition bound to a log's record types, ready for [`find_with`].
pub struct BoundCondition {
    cond: Cond<usize>,
    sources: Vec<SourceSpec>,
    slots: Vec<SlotSpec>,
    by_type: Vec<Vec<usize>>,
}

impl BoundCondition {
    /// The fields the condition names, as `TYPE.Field` or `TYPE[n].Field`, in order of first appearance.
    pub fn fields(&self) -> impl Iterator<Item = &str> {
        self.slots.iter().map(|s| s.label.as_str())
    }
}

/// The smallest and largest value one field took while the condition held.
#[derive(Clone, Debug, PartialEq)]
pub struct FieldRange {
    /// `TYPE.Field` or `TYPE[n].Field` as written in the condition.
    pub field: String,
    /// NaN if the field was never a number during the interval.
    pub min: f64,
    pub max: f64,
}

/// One stretch where the condition held.
#[derive(Clone, Debug, PartialEq)]
pub struct Interval {
    /// `TimeUS` of the record that made the condition true.
    pub start_us: u64,
    /// `TimeUS` of the record that made it false; `None` if it still held at the log's end.
    pub end_us: Option<u64>,
    /// End minus start; for an open interval, the log's last `TimeUS` minus start.
    pub duration_us: u64,
    /// One entry per field the condition names. Values of the record that ended the interval are not in them.
    pub ranges: Vec<FieldRange>,
}

/// What a pass found.
#[derive(Clone, Debug, PartialEq)]
pub struct Found {
    /// The first [`MAX_INTERVALS`] intervals, in time order.
    pub intervals: Vec<Interval>,
    /// How many further intervals there were (not kept).
    pub not_kept: u64,
}

impl Found {
    pub fn total(&self) -> u64 {
        self.intervals.len() as u64 + self.not_kept
    }
}

struct Open {
    start: u64,
    min: Vec<f64>,
    max: Vec<f64>,
}

struct Search<'a> {
    c: &'a BoundCondition,
    values: Vec<f64>,
    seen: Vec<bool>,
    unseen: usize,
    open: Option<Open>,
    found: Found,
    last_time: Option<u64>,
}

impl Search<'_> {
    fn fold(&mut self) {
        if let Some(open) = &mut self.open {
            for (i, &v) in self.values.iter().enumerate() {
                if v.is_nan() {
                    continue;
                }
                if open.min[i].is_nan() || v < open.min[i] {
                    open.min[i] = v;
                }
                if open.max[i].is_nan() || v > open.max[i] {
                    open.max[i] = v;
                }
            }
        }
    }

    fn close(&mut self, end: Option<u64>) {
        let Some(open) = self.open.take() else { return };
        if self.found.intervals.len() >= MAX_INTERVALS {
            self.found.not_kept += 1;
            return;
        }
        let stop = end.or(self.last_time).unwrap_or(open.start);
        let ranges = self
            .c
            .slots
            .iter()
            .enumerate()
            .map(|(i, s)| FieldRange {
                field: s.label.clone(),
                min: open.min[i],
                max: open.max[i],
            })
            .collect();
        self.found.intervals.push(Interval {
            start_us: open.start,
            end_us: end,
            duration_us: stop.saturating_sub(open.start),
            ranges,
        });
    }

    fn feed(&mut self, r: &Record<'_>) {
        let Some(t) = r.time_us() else { return };
        self.last_time = Some(t);
        let mut touched = false;
        for &s in &self.c.by_type[usize::from(r.type_id())] {
            let spec = &self.c.sources[s];
            if spec.type_name != r.type_name() {
                continue;
            }
            if let Some((column, n)) = spec.instance
                && r.number(column) != Some(n)
            {
                continue;
            }
            for &slot in &spec.slots {
                self.values[slot] = r.number(self.c.slots[slot].column).unwrap_or(f64::NAN);
            }
            if !self.seen[s] {
                self.seen[s] = true;
                self.unseen -= 1;
            }
            touched = true;
        }
        if !touched {
            return;
        }
        let holds = self.unseen == 0 && self.c.cond.eval(&self.values);
        match (holds, self.open.is_some()) {
            (true, false) => {
                let n = self.values.len();
                self.open = Some(Open {
                    start: t,
                    min: vec![f64::NAN; n],
                    max: vec![f64::NAN; n],
                });
                self.fold();
            }
            (true, true) => self.fold(),
            (false, true) => self.close(Some(t)),
            (false, false) => {}
        }
    }
}

/// One pass over `reader` with progress and cancellation; `None` if `on_progress` answered `Stop`. `cond`
/// must have been bound to the formats of the same log.
pub fn find_with<R: Read>(
    reader: &mut Reader<R>,
    cond: &BoundCondition,
    on_progress: impl FnMut(Progress) -> Control,
) -> io::Result<Option<Found>> {
    let mut search = Search {
        c: cond,
        values: vec![f64::NAN; cond.slots.len()],
        seen: vec![false; cond.sources.len()],
        unseen: cond.sources.len(),
        open: None,
        found: Found {
            intervals: Vec::new(),
            not_kept: 0,
        },
        last_time: None,
    };
    let done = scan(reader, |r| search.feed(r), on_progress)?;
    if !done {
        return Ok(None);
    }
    search.close(None);
    Ok(Some(search.found))
}
