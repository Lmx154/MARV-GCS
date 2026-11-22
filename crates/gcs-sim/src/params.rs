//! The parameters of the simulated vehicle and how their values go into the `param_value` field
//! of PARAM_VALUE and PARAM_SET (MAVLink parameter protocol).

use mavlink::dialects::common::MavParamType;

/// A parameter value and its real type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ParamValue {
    Uint8(u8),
    Int8(i8),
    Uint16(u16),
    Int16(i16),
    Uint32(u32),
    Int32(i32),
    Real32(f32),
}

/// How a value is carried in the 4-byte float field of PARAM_VALUE and PARAM_SET.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ParamEncoding {
    /// The value's own bytes are copied into the low bytes of the field (little-endian, the rest
    /// zero).
    #[default]
    Bytewise,
    /// The number is converted to a `float` (exact only up to 24 bits for integers).
    CCast,
}

/// One parameter of the simulated vehicle. Names are at most 16 ASCII bytes.
#[derive(Debug, Clone, PartialEq)]
pub struct SimParam {
    pub name: String,
    pub value: ParamValue,
    pub read_only: bool,
}

impl ParamValue {
    pub(crate) fn param_type(self) -> MavParamType {
        match self {
            Self::Uint8(_) => MavParamType::MAV_PARAM_TYPE_UINT8,
            Self::Int8(_) => MavParamType::MAV_PARAM_TYPE_INT8,
            Self::Uint16(_) => MavParamType::MAV_PARAM_TYPE_UINT16,
            Self::Int16(_) => MavParamType::MAV_PARAM_TYPE_INT16,
            Self::Uint32(_) => MavParamType::MAV_PARAM_TYPE_UINT32,
            Self::Int32(_) => MavParamType::MAV_PARAM_TYPE_INT32,
            Self::Real32(_) => MavParamType::MAV_PARAM_TYPE_REAL32,
        }
    }

    /// The content of the `param_value` field for this value.
    pub(crate) fn to_field(self, encoding: ParamEncoding) -> f32 {
        match encoding {
            ParamEncoding::CCast => match self {
                Self::Uint8(v) => f32::from(v),
                Self::Int8(v) => f32::from(v),
                Self::Uint16(v) => f32::from(v),
                Self::Int16(v) => f32::from(v),
                Self::Uint32(v) => v as f32,
                Self::Int32(v) => v as f32,
                Self::Real32(v) => v,
            },
            ParamEncoding::Bytewise => {
                let mut bytes = [0u8; 4];
                match self {
                    Self::Uint8(v) => bytes[0] = v,
                    Self::Int8(v) => bytes[..1].copy_from_slice(&v.to_le_bytes()),
                    Self::Uint16(v) => bytes[..2].copy_from_slice(&v.to_le_bytes()),
                    Self::Int16(v) => bytes[..2].copy_from_slice(&v.to_le_bytes()),
                    Self::Uint32(v) => bytes = v.to_le_bytes(),
                    Self::Int32(v) => bytes = v.to_le_bytes(),
                    Self::Real32(v) => bytes = v.to_le_bytes(),
                }
                f32::from_le_bytes(bytes)
            }
        }
    }

    /// A value of the same type as `self`, read from a `param_value` field.
    pub(crate) fn same_type_from_field(self, field: f32, encoding: ParamEncoding) -> Self {
        match encoding {
            ParamEncoding::CCast => match self {
                Self::Uint8(_) => Self::Uint8(field as u8),
                Self::Int8(_) => Self::Int8(field as i8),
                Self::Uint16(_) => Self::Uint16(field as u16),
                Self::Int16(_) => Self::Int16(field as i16),
                Self::Uint32(_) => Self::Uint32(field as u32),
                Self::Int32(_) => Self::Int32(field as i32),
                Self::Real32(_) => Self::Real32(field),
            },
            ParamEncoding::Bytewise => {
                let b = field.to_le_bytes();
                match self {
                    Self::Uint8(_) => Self::Uint8(b[0]),
                    Self::Int8(_) => Self::Int8(i8::from_le_bytes([b[0]])),
                    Self::Uint16(_) => Self::Uint16(u16::from_le_bytes([b[0], b[1]])),
                    Self::Int16(_) => Self::Int16(i16::from_le_bytes([b[0], b[1]])),
                    Self::Uint32(_) => Self::Uint32(u32::from_le_bytes(b)),
                    Self::Int32(_) => Self::Int32(i32::from_le_bytes(b)),
                    Self::Real32(_) => Self::Real32(field),
                }
            }
        }
    }
}

/// The parameters of a default [`SimConfig`](crate::SimConfig): the link contract allows only
/// INT32 and REAL32, so every integer parameter (a bool or an enum too) is an `Int32`. There is one
/// read-only parameter and one name of exactly 16 characters. The 44 parameters of the RC block
/// (`docs/link-contract.md` section 2) follow, then the sensor block (`crate::presets`), then the 23
/// parameters of indices 70 to 92 (`crate::fwparams`: `GPS_RATE_MS` is index 80, the compass block 81 to 91,
/// `GPS_GNSS` 92; 93 in all), so no other parameter changes its index.
pub fn default_params() -> Vec<SimParam> {
    let p = |name: &str, value, read_only| SimParam {
        name: name.to_string(),
        value,
        read_only,
    };
    let mut params = vec![
        p("IMU_RATE_HZ", ParamValue::Int32(800), false),
        p("IMU_ACC_RANGE", ParamValue::Int32(16), false),
        p("IMU_GYR_LPF_HZ", ParamValue::Real32(98.0), false),
        p("HIGHG_RANGE", ParamValue::Int32(320), false),
        p("BARO_RATE_HZ", ParamValue::Int32(50), false),
        p("GPS_RATE_HZ", ParamValue::Int32(10), false),
        p("LOG_ENABLE", ParamValue::Int32(1), false),
        p("LOG_RATE_HZ", ParamValue::Int32(100), false),
        p("BAT_V_SCALE", ParamValue::Real32(11.0), false),
        p("BAT_LOW_V", ParamValue::Real32(14.0), false),
        p("SYS_TRIM", ParamValue::Int32(-3), false),
        p("SYS_UID", ParamValue::Int32(1_592_460_852), true),
        p("NAV_TEST_16CHARS", ParamValue::Real32(1.5), false),
    ];
    params.extend(crate::rcmap::rc_params());
    params.extend(crate::presets::preset_params());
    params.extend(crate::fwparams::fw_params());
    params
}

/// The parameters [`SimConfig::with_every_param_type`](crate::SimConfig::with_every_param_type)
/// appends: one of each type that the contract does not use, for testing general decoding.
pub(crate) fn other_type_params() -> Vec<SimParam> {
    let p = |name: &str, value| SimParam {
        name: name.to_string(),
        value,
        read_only: false,
    };
    vec![
        p("TYPE_U8", ParamValue::Uint8(200)),
        p("TYPE_I8", ParamValue::Int8(-3)),
        p("TYPE_U16", ParamValue::Uint16(50_000)),
        p("TYPE_I16", ParamValue::Int16(-320)),
        p("TYPE_U32", ParamValue::Uint32(3_735_928_559)),
    ]
}
