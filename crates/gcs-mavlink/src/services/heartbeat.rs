//! Heartbeat service: sends the ground station's HEARTBEAT and tracks the vehicle's.

use std::time::Duration;

use crate::dialect::{HEARTBEAT_DATA, MavAutopilot, MavModeFlag, MavState, MavType};
use crate::flight::Home;
use crate::session::SessionConfig;
use crate::{Body, Frame, LinkStatus, Message, VehicleId};

use super::{Ctx, Service};

const INTERVAL: Duration = Duration::from_millis(1000);

pub struct Heartbeat {
    system_id: u8,
    timeout: Duration,
    /// Session time of the next heartbeat to send; `None` until the first tick.
    next_send: Option<Duration>,
}

impl Heartbeat {
    pub fn new(config: &SessionConfig) -> Self {
        Self {
            system_id: config.system_id,
            timeout: config.heartbeat_timeout,
            next_send: None,
        }
    }
}

impl Service for Heartbeat {
    fn on_frame(&mut self, frame: &Frame, cx: &mut Ctx) {
        if let Body::Known(Message::HOME_POSITION(home)) = &frame.body {
            let from_vehicle = frame.system_id != self.system_id
                && cx
                    .state
                    .link
                    .vehicle
                    .as_ref()
                    .is_some_and(|v| v.system_id == frame.system_id);
            if from_vehicle {
                cx.state.flight.home = Some(Home {
                    lat_e7: home.latitude,
                    lon_e7: home.longitude,
                    alt_mm: home.altitude,
                });
                cx.changed = true;
            }
            return;
        }
        let Body::Known(Message::HEARTBEAT(heartbeat)) = &frame.body else {
            return;
        };
        let link = &mut cx.state.link;
        if matches!(link.status, LinkStatus::Closed(_)) {
            return;
        }
        if heartbeat.mavtype == MavType::MAV_TYPE_GCS || frame.system_id == self.system_id {
            return;
        }
        match &link.vehicle {
            Some(vehicle) if vehicle.system_id != frame.system_id => return,
            Some(_) => {}
            None => {
                link.vehicle = Some(VehicleId {
                    system_id: frame.system_id,
                    component_id: frame.component_id,
                    mav_type: format!("{:?}", heartbeat.mavtype),
                    autopilot: format!("{:?}", heartbeat.autopilot),
                });
            }
        }
        link.last_heartbeat = Some(cx.now);
        link.status = LinkStatus::Connected;
        cx.state.flight.heartbeat(heartbeat.custom_mode);
        cx.changed = true;
    }

    fn on_tick(&mut self, cx: &mut Ctx) {
        let due = self.next_send.is_none_or(|next| cx.now >= next);
        if due {
            cx.out.push(ground_station_heartbeat());
            let scheduled = self.next_send.unwrap_or(cx.now) + INTERVAL;
            self.next_send = Some(if scheduled > cx.now {
                scheduled
            } else {
                cx.now + INTERVAL
            });
        }
        let link = &mut cx.state.link;
        if link.status == LinkStatus::Connected
            && let Some(last) = link.last_heartbeat
            && cx.now.saturating_sub(last) > self.timeout
        {
            link.status = LinkStatus::Lost;
            cx.changed = true;
        }
    }
}

fn ground_station_heartbeat() -> Message {
    Message::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_GCS,
        autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
        base_mode: MavModeFlag::empty(),
        system_status: MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
    })
}

#[cfg(test)]
#[path = "heartbeat_tests.rs"]
mod tests;
