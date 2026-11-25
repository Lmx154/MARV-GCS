# Link contract: mission timers, verdict and worked examples

Part of [MARV-GCS](../README.md) · [link-contract](link-contract.md), section 17 · rules in [link-missions](link-missions.md)

The station's side of the mission protocol (what it sends, how it reacts, its timers and the verdict it shows), the
vehicle's reply pacing, and worked examples with a test vector. S is the station, V the vehicle.

## Sequences

| Operation | Steps |
|---|---|
| Upload of n waypoints (n + 1 items, `count` n + 1) | 1. S sends `MISSION_COUNT`. 2. V checks it and sends `MISSION_REQUEST_INT` 0. 3. S sends `MISSION_ITEM_INT` 0. 4. V stores it in staging and requests 1; repeat up to item n (the return item). 5. V commits and sends ACK ACCEPTED with the new plan id. 6. S takes the ACK and reads the list back |
| Clear (`Command::ClearMission`) | S sends `MISSION_CLEAR_ALL`; V empties the list and sends ACK ACCEPTED with `opaque_id` 0; S reads the list back. An `UploadMission` of no waypoints is not a clear: it sends the return item alone, `count` 1 |
| Download | S sends `MISSION_REQUEST_LIST`; V sends `MISSION_COUNT` with the plan id; S requests `seq` 0 (n at least 1), V sends it, repeat; S, with every item in (with n = 0 at once), sends ACK ACCEPTED; V ends on that ACK |

## What the station sends (checked before anything goes out)

- An upload is `Command::UploadMission { items, return_height }`: the waypoints, then the return item, so `count` =
  waypoints + 1. More than 16 waypoints, or a return height that is not finite, is refused before sending.
- Every waypoint is checked: latitude and longitude finite and, converted, inside the vehicle's ranges; altitude finite. Any
  failure refuses the whole operation: nothing is sent, no read-back is made, and the reason names the row (the test is
  `Waypoint::is_valid` in `crates/marv-gcs/src/missions.rs`). The height limits are the vehicle's to check.

## What the station does with the vehicle's messages

- **Upload.** It answers every `MISSION_REQUEST_INT` with `seq` below the item count at once with that item, every time it
  is asked. A request at or above the count: it sends ACK INVALID_SEQUENCE and the upload fails. An ACK ACCEPTED ends the
  upload only after the station has sent the last item and only when its `opaque_id` equals the plan id the station computed;
  any other ACCEPTED is ignored (a late copy). An ACK of any other type ends the upload as refused with that type.
- **Download.** It takes `MISSION_ITEM_INT` `seq` k when it asked for k; another `seq` is dropped and request k is sent again
  at once. A second `MISSION_COUNT` is ignored. An ACK ACCEPTED is ignored; any other type ends the download as failed. The
  station sets no cap on the vehicle's `count`. It treats the last item as the return height and the others as waypoints; a
  list whose last item is not command 20 is an error, not a mission.
- **Clear.** ACK ACCEPTED with `opaque_id` 0 ends it accepted; an ACCEPTED with another `opaque_id` is ignored; any other
  type ends it refused.
- Every mission message that belongs to no operation is ignored. One mission operation at a time; a second command meanwhile
  is refused and nothing is sent.

## Timeouts and retries

1500 ms and 5 retries (MAVLink's 250 ms for plan items is not used: 1500 ms holds for every message). The vehicle hands a
message to its link's reply slot; the station sends at once. Every timer starts at the hand-over, whatever then happens to
the message. A retry is the same message handed over again when 1500 ms have passed since its last hand-over without its
response, at most 5 retries (6 hand-overs); 1500 ms after the sixth the sender gives up. A new message (every message sent in
answer to a request) starts with its own count and timer: the station's last item sent again because the vehicle asked again
starts a new count of 6 hand-overs (9000 ms in all). A re-request caused by a wrong item restarts that request's timer and
leaves its count as it was.

| Who | Retries | Response that stops it | On giving up |
|---|---|---|---|
| S | `MISSION_COUNT` | a `MISSION_REQUEST_INT` or a `MISSION_ACK` | sends ACK OPERATION_CANCELLED; failed, "no answer" |
| S | `MISSION_CLEAR_ALL` | a `MISSION_ACK` | the same |
| S | `MISSION_ITEM_INT` of the last item only | a `MISSION_REQUEST_INT` or a `MISSION_ACK` | the same |
| S | `MISSION_REQUEST_LIST` | `MISSION_COUNT`, or an ACK other than ACCEPTED | the same |
| S | `MISSION_REQUEST_INT` k | `MISSION_ITEM_INT` k, or an ACK other than ACCEPTED | the same |
| V | `MISSION_REQUEST_INT` k (upload) | `MISSION_ITEM_INT` k from the initiator | sends ACK OPERATION_CANCELLED to the initiator; the upload ends, the list unchanged |

- An ACK the station ignores is no response: it stops no timer. Not retried by its sender: an item other than the last (the
  vehicle asks again), `MISSION_ACK`, the vehicle's `MISSION_COUNT` and `MISSION_ITEM_INT` (the station asks again).
- After sending an item other than the last, the station waits 9000 ms (6 x 1500 ms, the vehicle's whole retry span) for a
  request or an ACK it takes, then gives up as in the table.
- In a download the vehicle asks for nothing: when 9000 ms pass after the last mission message from the initiator that passed the
  addressing check, it ends the download and sends ACK OPERATION_CANCELLED to the initiator. Its own sends do not count.
- A frame dropped by the addressing check refreshes no timer. A side that receives ACK OPERATION_CANCELLED ends the operation.

## Reply pacing

The vehicle sends its mission messages through one reply slot per link, with an interval of 40 ms on the radio link and 5 ms
on USB. On the radio the slot is a stream of the radio node, so the radio's flow control lengthens its interval by at most
2040 ms; the timers do not change for it. Two mission messages on one link go out at least one interval apart, and a
handed-over message that is not replaced or dropped goes out no later than one interval after its hand-over or after the
previous mission message on that link, whichever is later; sending at once when the slot is idle and sending at the first tick
of a fixed grid are both legal, and no side may depend on the choice. A slot holds one waiting message: a new message to the
same destination replaces it; one to another destination is dropped while one waits (the receiver's retry recovers it). The
station sends at once, without pacing.

## Link down

Vehicle: when the initiator's link goes down (USB down; radio: the port leaves its MAVLink lock, where the radio node's
`on_link_down` runs), the transaction ends at once as cancelled: staging discarded, the list unchanged, nothing sent; the
message waiting in that link's slot is dropped and the last upload of an initiator on that link is forgotten. A transaction owned
on the other link goes on. Station: when the session's link closes, a running mission operation ends at once as failed
("link lost"); nothing is sent and no read-back is made.

## The station's verdict

After an upload or a clear ends in any way other than "link lost" or the station's own refusal before sending (accepted,
refused, or given up) the station reads the list back with one download at once. It is done only when all hold: the vehicle
accepted it (ACK ACCEPTED with the plan id the station computed, 0 for a clear); the read-back completed; its `count` equals
the number sent; its items equal what was sent `seq` by `seq` (`frame` 3, `command` 16, or 20 for the return item; `x`, `y`;
`z` as 4 bytes); and its `opaque_id` equals the station's plan id. Otherwise it failed, and the read-back list, when one
completed, is shown as what the vehicle holds. The reason shown is the first that applies: 1. "link lost" (the read-back
included); 2. the upload or clear failed (the vehicle's refusal type, "no answer", or the station's own INVALID_SEQUENCE);
3. the read-back failed; 4. the count differs ("the vehicle holds m waypoints, n were sent"); 5. the first `seq` whose item
differs; 6. the plan id differs. A download asked for by itself shows the vehicle's list and plan id: complete or failed. The
window shows the number of waypoints, never counting the return item.

The station's commands are `Command::UploadMission`, `DownloadMission` and `ClearMission` (Operate-class); `marv-cli mission
upload`, `download` and `clear` use them. `gcs-sim` serves this section as the vehicle does, from this text, with a reply slot
of 5 ms; it runs disarmed, so its armed refusals are exercised only by tests calling its library; its configuration can drop
chosen mission frames in either direction.

## Worked example 1: an upload with one item lost, then the read-back

Radio link, the vehicle's list empty before (plan id 0). Times in ms, approximate, and one legal timing among others (the
vehicle's slot on a fixed grid at multiples of 40 ms, the link's own delay left out). Made-up points in the open ocean, then
the return item:

| `seq` | `command` | latitude | longitude | `x` | `y` | `z` (m) |
|---|---|---|---|---|---|---|
| 0 | 16 | 10.0000000 | -30.0000000 | 100000000 | -300000000 | 10.0 |
| 1 | 16 | 10.0010000 | -30.0000000 | 100010000 | -300000000 | 12.5 |
| 2 | 16 | 10.0010000 | -29.9990000 | 100010000 | -299990000 | 15.0 |
| 3 | 20 | | | 0 | 0 | 20.0 |

1. t 0, S -> V: `MISSION_COUNT` `count` 4, `mission_type` 0, `opaque_id` 0; target 1, 1.
2. t 40, V -> S: `MISSION_REQUEST_INT` `seq` 0; target 255, 190.
3. S -> V: `MISSION_ITEM_INT` `seq` 0, `frame` 3, `command` 16, `current` 0, `autocontinue` 1, `param1`-`param4` 0.0, `x`
   100000000, `y` -300000000, `z` 10.0, `mission_type` 0.
4. t 80, V -> S: `MISSION_REQUEST_INT` `seq` 1 (handed to the slot when item 0 arrived, about t 45: the vehicle's timer starts then).
5. S -> V: item `seq` 1: lost on the link. The station does not resend it (not the last item); its 9000 ms wait runs.
6. t 1560, V -> S: `MISSION_REQUEST_INT` `seq` 1, retry 1 of 5.
7. S -> V: item `seq` 1, as in step 5. 8. t 1600, V -> S: request `seq` 2. 9. S -> V: item `seq` 2.
10. t 1640, V -> S: request `seq` 3. 11. S -> V: item `seq` 3, `command` 20, `x` 0, `y` 0, `z` 20.0; the station's timer for the
    last item starts.
12. The vehicle commits the four items; plan id 0x1862F4A2. t 1680, V -> S: `MISSION_ACK` ACCEPTED, `mission_type` 0,
    `opaque_id` 409138338 (on the wire A2 F4 62 18). It equals the station's own plan id, so the upload ends accepted.
13. S -> V: `MISSION_REQUEST_LIST`. 14. t 1720, V -> S: `MISSION_COUNT` `count` 4, `opaque_id` 409138338.
15. Request and item for `seq` 0 to 3, one per slot (t 1760 to 1880), each item with `frame` 3, `current` 0, `autocontinue`
    1, `param1`-`param4` 0.0 and the values above (`command` 20 for `seq` 3).
16. S -> V: ACK ACCEPTED, `opaque_id` 0. Verdict: done (count, items and plan id equal).

**Test vector** (Python's `zlib.crc32` and a bitwise copy of the algorithm agree): 48 bytes covered, `seq` 0 to 3, each `x`,
`y`, `z` little-endian:

```
00 E1 F5 05  00 5D 1E EE  00 00 20 41
10 08 F6 05  00 5D 1E EE  00 00 48 41
10 08 F6 05  10 84 1E EE  00 00 70 41
00 00 00 00  00 00 00 00  00 00 A0 41
```

CRC-32 = 0x1862F4A2 = plan id (409138338). The empty list: 0x00000000. The same three waypoints alone, without a return
item, give 0x99C3BD49 (a list of an earlier format; a vehicle that takes return items refuses it with INVALID). With a return
`z` of 4.0 instead the upload ends at `seq` 3 with INVALID_PARAM7 (below the 5 m limit) and the vehicle's list is unchanged.

## Worked example 2: an upload the vehicle refuses

The vehicle holds the list of example 1 (plan id 0x1862F4A2). Another station sends `MISSION_COUNT` `count` 18,
`mission_type` 0, `opaque_id` 0 (this station refuses more than 16 waypoints itself). The vehicle answers `MISSION_ACK`
NO_SPACE (4), `mission_type` 0, `opaque_id` 0; no item is asked for, no staging is used, no transaction stays open. The
upload ends refused ("the vehicle holds at most 16 waypoints"); a read-back (`MISSION_REQUEST_LIST`, `MISSION_COUNT` `count`
4 with `opaque_id` 409138338, the four items, ACK ACCEPTED) gives verdict failed, and the vehicle's list is unchanged.

## Related

[link-missions](link-missions.md) · [link-flight](link-flight.md) · [ui-map](ui-map.md) · [marv-cli](marv-cli.md)
