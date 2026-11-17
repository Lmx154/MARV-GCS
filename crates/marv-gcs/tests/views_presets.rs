//! Headless test of the "Sensor presets" section of the Devices tab with a hand-built parameter table
//! (`docs/link-contract.md` section 13).

use std::cell::{Cell, RefCell};
use std::rc::Rc;
use std::time::Duration;

use eframe::egui::accesskit::Role;
use eframe::egui::{self, Color32, Shape, pos2};
use egui_kittest::Harness;
use egui_kittest::kittest::{NodeT, Queryable};
use gcs_mavlink::dialect::MavCmd;
use gcs_mavlink::services::params::{FetchStatus, Param, ParamTable, ParamType, ParamValue};
use gcs_mavlink::{Access, Command, CommandRecord, CommandStatus, VehicleState};
use marv_gcs::views::presets::{
    self, CUSTOM_TEXT, IMU_TEXT, MISMATCH_TEXT, NOT_LOADED_TEXT, SKEW_TEXT,
};
use marv_gcs::views::{self, ViewCtx};
use marv_presets::{BARO, IMU1, MAG, SENSORS, Sensor, Value, preset};

const NOT_SAVED: &str = "Applied in RAM; not saved to flash";

fn put(table: &mut ParamTable, name: &str, value: Value) {
    let (kind, raw) = match value {
        Value::I32(v) => (ParamType::I32, v.to_le_bytes()),
        Value::F32(v) => (ParamType::F32, v.to_le_bytes()),
    };
    let index = table.params.len() as u16;
    table.params.insert(
        name.to_string(),
        Param {
            name: name.to_string(),
            index,
            kind,
            raw,
        },
    );
}

/// The selector of `sensor` at `id` and its fields at the values of preset `id`.
fn set_sensor(table: &mut ParamTable, sensor: &Sensor, id: i32) {
    put(table, sensor.selector, Value::I32(id));
    if let Some(p) = preset(sensor, id) {
        for (field, value) in sensor.fields.iter().zip(p.values) {
            put(table, field, *value);
        }
    }
}

/// A complete list with every sensor at preset 1.
fn loaded() -> ParamTable {
    let mut table = ParamTable {
        fetch: FetchStatus::Complete,
        encoding_known: true,
        ..ParamTable::default()
    };
    for sensor in SENSORS {
        set_sensor(&mut table, sensor, 1);
    }
    table
}

type Shared<T> = Rc<RefCell<T>>;

struct Rig {
    harness: Harness<'static>,
    state: Shared<VehicleState>,
    now: Rc<Cell<Duration>>,
    commands: Shared<Vec<Command>>,
}

impl Rig {
    fn new(access: Access, params: ParamTable) -> Rig {
        let vehicle = VehicleState {
            params,
            ..VehicleState::default()
        };
        let state = Rc::new(RefCell::new(vehicle));
        let now = Rc::new(Cell::new(Duration::from_secs(10)));
        let commands: Shared<Vec<Command>> = Rc::default();
        let mut view = views::all()
            .into_iter()
            .find(|v| v.name() == "Devices")
            .expect("Devices view is registered");
        let (s, n, c) = (state.clone(), now.clone(), commands.clone());
        let mut harness = Harness::builder()
            .with_size(egui::vec2(1500.0, 1800.0))
            .build_ui(move |ui| {
                let state = s.borrow();
                let mut out = c.borrow_mut();
                let mut cx = ViewCtx {
                    state: &state,
                    commands: &mut out,
                    now: Some(n.get()),
                    access,
                };
                view.ui(ui, &mut cx);
            });
        harness.run();
        Rig {
            harness,
            state,
            now,
            commands,
        }
    }

    fn has(&self, text: &str) -> bool {
        self.harness
            .query_all_by_label_contains(text)
            .next()
            .is_some()
    }

    fn count(&self, text: &str) -> usize {
        self.harness.query_all_by_label_contains(text).count()
    }

    /// The `n`th node with this exact label (cards in order IMU1, IMU2, BARO, MAG).
    fn nth<'a>(&'a self, label: &'a str, n: usize) -> egui_kittest::Node<'a> {
        self.harness
            .get_all_by_label(label)
            .nth(n)
            .unwrap_or_else(|| panic!("no {n}th {label}"))
    }

    fn click_nth(&mut self, label: &str, n: usize) {
        self.nth(label, n).click();
        self.harness.run();
    }

    fn enabled_nth(&self, label: &str, n: usize) -> bool {
        !self.nth(label, n).accesskit_node().is_disabled()
    }

    fn combo(&self, card: usize) -> egui_kittest::Node<'_> {
        self.harness
            .get_all_by_role(Role::ComboBox)
            .nth(card)
            .expect("a drop-down per card")
    }

    /// Choose `name` in the drop-down of card `card`.
    fn pick(&mut self, card: usize, name: &str) {
        self.combo(card).click();
        self.harness.run();
        self.harness
            .get_all_by_role_and_label(Role::Button, name)
            .next()
            .unwrap_or_else(|| panic!("no entry {name}"))
            .click();
        self.harness.run();
    }

    fn shown(&self, card: usize) -> String {
        self.combo(card).value().unwrap_or_default()
    }

    fn sent(&self) -> Vec<String> {
        self.commands
            .borrow()
            .iter()
            .map(|c| format!("{c:?}"))
            .collect()
    }

    fn update(&mut self, change: impl FnOnce(&mut VehicleState)) {
        change(&mut self.state.borrow_mut());
        self.harness.run();
    }
}

fn name_of(sensor: &Sensor, id: i32) -> &'static str {
    preset(sensor, id).expect("preset").name
}

const BARO_CARD: usize = 2;

#[test]
fn four_cards_show_the_crates_names_summaries_and_physical_values() {
    let rig = Rig::new(Access::Configure, loaded());
    for (i, sensor) in SENSORS.iter().enumerate() {
        assert_eq!(rig.shown(i), name_of(sensor, 1), "card {i}");
        assert!(rig.has(preset(sensor, 1).unwrap().summary), "card {i}");
    }
    for title in ["IMU1 preset", "IMU2 preset", "BARO preset", "MAG preset"] {
        assert!(rig.has(title), "{title}");
    }
    assert_eq!(rig.harness.get_all_by_role(Role::ComboBox).count(), 4);
    // IMU: the accelerometer's second value and the model tag; BARO: the altitude.
    assert!(rig.has("Low noise: 0.078 dps, 1.61 mg (model)"));
    assert!(rig.has("Accel range: 16 g"));
    assert!(rig.has("Light load: 14400 B/s"));
    assert!(rig.has("Update rate: 50 Hz"));
    assert!(rig.has("(bound)"), "the MAG delay is a bound");
}

#[test]
fn the_open_picker_offers_the_crates_presets_and_never_custom() {
    let mut rig = Rig::new(Access::Configure, loaded());
    rig.combo(BARO_CARD).click();
    rig.harness.run();
    for p in BARO.presets {
        assert!(
            rig.harness
                .query_all_by_role_and_label(Role::Button, p.name)
                .next()
                .is_some(),
            "{}",
            p.name
        );
    }
    assert!(rig.harness.query_by_label(CUSTOM_TEXT).is_none());
}

#[test]
fn baro_apply_sends_one_set_and_five_reads_and_applied_waits_for_the_read_back() {
    let mut rig = Rig::new(Access::Configure, loaded());
    rig.pick(BARO_CARD, "Smooth");
    assert!(rig.commands.borrow().is_empty(), "choosing sends nothing");
    assert!(rig.has("Smooth: not applied"));
    rig.click_nth("Apply", BARO_CARD);
    let sent = rig.sent();
    assert_eq!(sent.len(), 6, "{sent:?}");
    assert!(matches!(
        &rig.commands.borrow()[0],
        Command::SetParam { name, value: ParamValue::I32(3) } if name == "BARO_PRESET"
    ));
    let reads: Vec<String> = rig.commands.borrow()[1..]
        .iter()
        .map(|c| match c {
            Command::ReadParam { name } => name.clone(),
            other => panic!("{other:?}"),
        })
        .collect();
    assert_eq!(reads, BARO.fields);
    assert_eq!(reads.len(), 5);
    // The echo carries only the selector: the fields are still the old preset's.
    rig.update(|s| put(&mut s.params, "BARO_PRESET", Value::I32(3)));
    assert!(!rig.has(NOT_SAVED), "applied only after the read-back");
    assert!(rig.has("reading back"));
    // Three of the five reads arrived.
    rig.update(|s| {
        let p = preset(&BARO, 3).unwrap();
        for (f, v) in BARO.fields.iter().zip(p.values).take(3) {
            put(&mut s.params, f, *v);
        }
    });
    assert!(!rig.has(NOT_SAVED));
    rig.update(|s| set_sensor(&mut s.params, &BARO, 3));
    assert!(rig.has(NOT_SAVED));
    assert!(rig.enabled_nth("Save to flash", 0));
    assert_eq!(rig.sent().len(), 6, "nothing else was sent");
}

fn storage_record(started: Duration, status: CommandStatus) -> CommandRecord {
    CommandRecord {
        command: 245,
        name: None,
        params: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        status,
        sends: 1,
        started,
        ended: None,
        refused: None,
        cancel: None,
    }
}

fn applied_baro(rig: &mut Rig) {
    rig.pick(BARO_CARD, "Smooth");
    rig.click_nth("Apply", BARO_CARD);
    rig.update(|s| set_sensor(&mut s.params, &BARO, 3));
    assert!(rig.has(NOT_SAVED));
    rig.commands.borrow_mut().clear();
}

#[test]
fn save_sends_one_command_245_and_the_line_clears_only_on_an_accepted_save_after_the_apply() {
    let mut rig = Rig::new(Access::Configure, loaded());
    // An accepted save from before the Apply does not count.
    rig.update(|s| {
        s.command = Some(storage_record(
            Duration::from_secs(1),
            CommandStatus::Finished {
                text: "accepted",
                accepted: true,
            },
        ));
    });
    applied_baro(&mut rig);
    rig.click_nth("Save to flash", 0);
    let sent = rig.sent();
    assert_eq!(sent.len(), 1, "{sent:?}");
    assert!(sent[0].contains(&format!("{:?}", MavCmd::MAV_CMD_PREFLIGHT_STORAGE)));
    assert!(sent[0].contains("[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"));
    let started = rig.now.get();
    rig.update(|s| s.command = Some(storage_record(started, CommandStatus::Waiting)));
    assert!(rig.has(NOT_SAVED), "waiting is not saved");
    assert!(!rig.enabled_nth("Save to flash", 0), "a command is running");
    rig.update(|s| {
        s.command = Some(storage_record(
            started,
            CommandStatus::Finished {
                text: "denied",
                accepted: false,
            },
        ));
    });
    assert!(rig.has(NOT_SAVED), "a refused save is not saved");
    rig.update(|s| {
        s.command = Some(storage_record(
            started,
            CommandStatus::Finished {
                text: "accepted",
                accepted: true,
            },
        ));
    });
    assert!(!rig.has(NOT_SAVED));
    assert!(rig.harness.query_by_label("Save to flash").is_none());
}

#[test]
fn a_read_back_that_differs_says_so_and_offers_no_save() {
    let mut rig = Rig::new(Access::Configure, loaded());
    rig.pick(BARO_CARD, "Smooth");
    rig.click_nth("Apply", BARO_CARD);
    // The selector was accepted, one field never took the preset's value.
    rig.update(|s| {
        set_sensor(&mut s.params, &BARO, 3);
        put(&mut s.params, "BARO_OSR_P", Value::I32(1));
    });
    assert!(rig.has("reading back"));
    rig.now.set(rig.now.get() + Duration::from_secs(4));
    rig.update(|_| {});
    assert!(rig.has(MISMATCH_TEXT));
    assert!(!rig.has(NOT_SAVED));
    assert!(rig.harness.query_by_label("Save to flash").is_none());
    assert_eq!(rig.sent().len(), 6);
}

#[test]
fn mag_apply_reads_its_two_fields_and_compares_the_real_value_numerically() {
    let mut rig = Rig::new(Access::Configure, loaded());
    rig.pick(3, name_of(&MAG, 4));
    rig.click_nth("Apply", 3);
    assert_eq!(rig.sent().len(), 3, "{:?}", rig.sent());
    rig.update(|s| set_sensor(&mut s.params, &MAG, 4));
    assert!(rig.has(NOT_SAVED));
    assert!(matches!(preset(&MAG, 4).unwrap().values[0], Value::F32(_)));
}

#[test]
fn an_operate_session_shows_the_cards_with_apply_disabled_and_sends_nothing() {
    let mut rig = Rig::new(Access::Operate, loaded());
    assert_eq!(rig.shown(BARO_CARD), name_of(&BARO, 1));
    assert!(rig.has("Configure needs a serial or TCP link"));
    assert!(!rig.enabled_nth("Apply", BARO_CARD));
    rig.pick(BARO_CARD, "Smooth");
    rig.click_nth("Apply", BARO_CARD);
    assert!(rig.commands.borrow().is_empty());
}

#[test]
fn a_running_calibration_disables_apply() {
    let mut rig = Rig::new(Access::Configure, loaded());
    assert!(rig.enabled_nth("Apply", BARO_CARD));
    let started = rig.now.get();
    rig.update(|s| {
        s.command = Some(CommandRecord {
            command: 241,
            params: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ..storage_record(started, CommandStatus::Waiting)
        });
    });
    assert!(!rig.enabled_nth("Apply", BARO_CARD));
    assert!(rig.has("a command is running"));
}

#[test]
fn imu_cards_have_a_picker_and_a_disabled_apply() {
    let mut rig = Rig::new(Access::Configure, loaded());
    assert_eq!(rig.count(IMU_TEXT), 2, "one per IMU card");
    assert!(!rig.enabled_nth("Apply", 0));
    assert!(!rig.enabled_nth("Apply", 1));
    assert!(rig.enabled_nth("Apply", 2));
    rig.pick(0, name_of(&IMU1, 4));
    assert_eq!(rig.shown(0), name_of(&IMU1, 4));
    rig.click_nth("Apply", 0);
    assert!(rig.commands.borrow().is_empty());
}

#[test]
fn an_unknown_selector_value_is_named_and_the_picker_is_off() {
    let mut params = loaded();
    put(&mut params, "BARO_PRESET", Value::I32(9));
    let rig = Rig::new(Access::Configure, params);
    assert!(rig.has("unknown preset 9 (station older than firmware)"));
    assert!(rig.combo(BARO_CARD).accesskit_node().is_disabled());
    assert!(!rig.combo(0).accesskit_node().is_disabled());
}

#[test]
fn a_known_preset_with_other_field_values_is_a_table_mismatch_and_the_picker_is_off() {
    let mut params = loaded();
    put(&mut params, "BARO_OSR_P", Value::I32(64));
    let rig = Rig::new(Access::Configure, params);
    assert!(rig.has(SKEW_TEXT));
    assert_eq!(rig.count(SKEW_TEXT), 1);
    assert!(rig.combo(BARO_CARD).accesskit_node().is_disabled());
    assert!(!rig.combo(3).accesskit_node().is_disabled());
}

#[test]
fn custom_is_shown_read_only_and_an_unloaded_list_says_so() {
    let mut params = loaded();
    put(&mut params, "MAG_PRESET", Value::I32(0));
    let rig = Rig::new(Access::Configure, params);
    assert_eq!(rig.shown(3), CUSTOM_TEXT);
    assert!(rig.combo(3).accesskit_node().is_disabled());
    drop(rig);
    let rig = Rig::new(Access::Configure, ParamTable::default());
    assert_eq!(rig.count(NOT_LOADED_TEXT), 4);
    assert!(rig.harness.query_by_label("Apply").is_none());
}

// The chart geometry.

/// Five axes, radius 100 about (200, 150); the score set 100, 20, 100, 100, 100 is not convex.
/// Spoke i points at angle -90 deg + 72 deg * i from the +x axis (y down): the unit vectors are
/// (0, -1), (0.95106, -0.30902), (0.58779, 0.80902), (-0.58779, 0.80902), (-0.95106, -0.30902).
/// Vertices: (200, 50); 20 % of the second spoke: (200 + 19.021, 150 - 6.180); (258.779, 230.902);
/// (141.221, 230.902); (104.894, 119.098).
/// The polygon is the sum of five triangles from the centre, each 0.5 * r_i * r_(i+1) * sin 72 deg
/// (sin 72 deg = 0.95106): 0.5 * 0.95106 * (2000 + 2000 + 10000 + 10000 + 10000) = 16167.96.
fn pentagon() -> Vec<egui::Pos2> {
    presets::radar_vertices(&[100, 20, 100, 100, 100], pos2(200.0, 150.0), 100.0)
}

fn near(a: egui::Pos2, x: f32, y: f32) -> bool {
    (a.x - x).abs() < 0.01 && (a.y - y).abs() < 0.01
}

#[test]
fn the_radar_polygon_has_one_vertex_per_spoke_at_its_score() {
    let v = pentagon();
    assert_eq!(v.len(), 5);
    assert!(near(v[0], 200.0, 50.0), "{:?}", v[0]);
    assert!(near(v[1], 219.021, 143.820), "{:?}", v[1]);
    assert!(near(v[2], 258.779, 230.902), "{:?}", v[2]);
    assert!(near(v[3], 141.221, 230.902), "{:?}", v[3]);
    assert!(near(v[4], 104.894, 119.098), "{:?}", v[4]);
    // A diamond: right, down, left of a top vertex at score 0 is the centre itself.
    let d = presets::radar_vertices(&[100, 50, 0, 100], pos2(0.0, 0.0), 80.0);
    assert!(near(d[0], 0.0, -80.0) && near(d[1], 40.0, 0.0));
    assert!(near(d[2], 0.0, 0.0) && near(d[3], -80.0, 0.0));
}

fn mesh_area(shape: &Shape) -> (usize, f32) {
    let Shape::Mesh(mesh) = shape else {
        panic!("the fill is a Mesh (a fan of triangles), not {shape:?}");
    };
    let mut area = 0.0;
    for t in mesh.indices.chunks(3) {
        let [a, b, c] = [t[0], t[1], t[2]].map(|i| mesh.vertices[i as usize].pos);
        area += 0.5 * ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)).abs();
    }
    (mesh.indices.len() / 3, area)
}

#[test]
fn a_non_convex_polygon_is_filled_as_a_fan_of_one_triangle_per_edge() {
    let v = pentagon();
    let shape = presets::fill_shape(&v, pos2(200.0, 150.0), Color32::ORANGE);
    let (triangles, area) = mesh_area(&shape);
    assert_eq!(triangles, 5);
    assert!((area - 16167.96).abs() < 1.0, "area {area}");
}

#[test]
fn every_preset_of_every_sensor_has_one_vertex_per_axis_and_a_fan_of_that_many_triangles() {
    for sensor in SENSORS {
        for p in sensor.presets {
            let v = presets::radar_vertices(p.scores, pos2(0.0, 0.0), 50.0);
            assert_eq!(v.len(), sensor.axes.len());
            let (triangles, _) = mesh_area(&presets::fill_shape(&v, pos2(0.0, 0.0), Color32::RED));
            assert_eq!(triangles, sensor.axes.len());
        }
    }
}

#[test]
fn numbers_are_shown_with_a_few_digits_and_no_trailing_zeros() {
    assert_eq!(presets::number_text(16.0), "16");
    assert_eq!(presets::number_text(14400.0), "14400");
    assert_eq!(presets::number_text(0.078), "0.078");
    assert_eq!(presets::number_text(1.61), "1.61");
    assert_eq!(presets::number_text(12.34), "12.3");
    assert_eq!(presets::number_text(0.0), "0");
}
