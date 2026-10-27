# BIOBUZZ: two hive-tip attempts, then park

`BiobuzzHiveAuto` implements the requested sequence using Pedro + Pinpoint for position and
Limelight for same-tag high-to-low transition evidence. It targets **your own alliance's hive**,
not both alliances' hives. It is a **commissioning draft listed on the Driver Station**, not a
match-ready routine. Listing does not bypass the calibration gates: the default configuration
still prevents movement and shooting.

The expanded sequence uses **four separate four-pollen shooting batches**:

**Shoot 4 preloads → flower 1: collect 4 → shoot 4 / confirm first tip → flower 2: collect 4 →
opposite cell: shoot 4 → own garden: collect 4 → shoot 4 / confirm second tip → park.**

Eight pollen per tip is the team's requested tuning assumption, not a guaranteed manual threshold.
The first batch at each cell deliberately does not require a tip. The refill batch does.
Only your alliance's two flowers and garden are visited; no opponent-side pickup is programmed (G402).

![Nominal red and blue autonomous routes](BIOBUZZ_HIVE_AUTO.svg)

## Route and start

The coordinate frame is the existing [BIOBUZZ field map](BIOBUZZ_FIELD_LOCATIONS.md):
robot center in inches; origin audience-left; +X right, +Y away from audience.
The nominal hive frame footprint is about 49.46 x 38.95 inches centered on the field.
Shooting poses are team-chosen estimates, NOT FIRST-prescribed positions or verified trajectories.

| Stage | Red pose (X, Y, heading degrees) | Blue pose |
| --- | --- | --- |
| Required wall-contact start | (36, half robot length, 90) | (108, 144 − half robot length, 270) |
| Clear wall | (36, 21, 90) | (108, 123, 270) |
| First approach | (36, 32, 90) | (108, 112, 270) |
| Shooting batches 1 and 2: initially raised cell | (59.25, 32, 90) | (84.75, 112, 270) |
| Flower 1 intake approach → pickup | (23, 48, 180) → (15, 48, 180) | (121, 96, 0) → (129, 96, 0) |
| Flower 2 intake approach → pickup | (48, 121, 90) → (48, 129, 90) | (96, 23, 270) → (96, 15, 270) |
| Shooting batches 3 and 4: opposite cell | (59.25, 112, 270) | (84.75, 32, 90) |
| Own garden intake approach → pickup | (12, 20, 270) → (12, 12, 270) | (132, 124, 90) → (132, 132, 90) |
| Outside transit corridor | X = 28 | X = 116 |
| Nominal loading-zone park | (12, 108, 180) | (132, 36, 0) |

`HiveAutoRoute.approach(Stage)` contains every intermediate transit waypoint. Each pickup's
last eight inches runs at reduced drive power with the intake on, followed by a bounded dwell.
The route backs away from wall pickups before rotating, and retreats to the outside corridor
before changing hive sides. Flower centers are nominally (0,48)/(48,144) for red and
(144,96)/(96,0) for blue; pickup poses above refer to the robot center, not the flower itself.
The garden approach assumes a sufficiently wide, floor-level intake can collect its four staged pollen.
Intake reach, lower-opening access, contact clearance and garden sweep require physical verification.

Parking requires the robot to be **at least partially in its loading zone**, not entirely inside it
(TU06 section 10.5.4, page 90). These park poses overlap the nominal loading-zone rectangles with
an 18-inch footprint. Measure the taped boundaries and actual footprint before enabling.
Only these two starts are currently supported; INIT d-pad left/right selects red/blue and a fresh
A confirms the corresponding physical placement. After Play, no gamepad commands are read;
use Driver Station STOP to abort and enable its 30-second AUTO timer (G401).

## Shooting and confirmation

1. Arrive and hold the configured shooting pose using Pinpoint.
2. Require three distinct Limelight frames showing the SAME tag high for at least 0.25 seconds.
3. Spin `Shoot1` / `Shoot2` to their configured RPMs; require both within 150 RPM for 0.25 seconds.
4. Pulse the explicitly selected feeder four times per batch, with flywheel readiness checks and pauses.
   A complete pulse counts as one **expected** pollen only under verified timing calibration.
5. Stop feeding immediately if low-height evidence appears, visibility is lost, RPM drops, or pose drifts.
6. After batches 1/3, allow a completed four-pulse batch without a tip so the robot can collect the refill.
   After batches 2/4, require the same tag low in three distinct frames over at least 0.35 seconds.
   The same detector/tag identity is retained between the two batches for each cell.

Unexpected first-batch tipping stops the planned scoring sequence rather than firing into a low cell.
An interrupted pulse or pickup makes expected inventory UNKNOWN and blocks all further pickups.
The software refuses to start any pickup unless expected inventory is empty, respecting G407's
four-element carrying limit. This is not a sensor-backed guarantee: timing alone cannot prove the
robot holds four pollen, has released four pollen, or has scored them successfully.

Expected tag clusters from Figure 9-17 (page 77): red first 34–37 then 30–33;
blue first 42–45 then 38–41. Initial raised sides follow the staging diagram (page 83).
Tag disappearance, a different tag, repeated polling of one frame, or elapsed shooting time
does NOT count as a tip. The tag-height signal is a **vision proxy**, not proof of official scoring:
section 10.5.1 requires the hive's opposite damper to contact the frame.

Drive/heading hold, Limelight polling and shooting are updated in one nonblocking loop. There are
no `sleep()` calls, so waiting for flywheel speed or a hive transition does not pause movement updates.
Limelight never changes the Pinpoint position. There is no automatic ballistic solver or vision steering;
the shooting pose, heading, flywheel direction/RPM and feeder must be physically calibrated.

## Commissioning gates — do not skip

In `config/HiveAutoConfig.java`:

- Keep `DRY_RUN = true` initially. No shooter, feeder, intake or Limelight is initialized in dry-run mode;
  pickups/shots are simulated in the inventory model and no tip successes are claimed. The route
  still drives to pickup poses, so physical route/clearance verification remains mandatory.
- Check all paths, actual robot dimensions, extended mechanisms, teammate clearance, frame/CELL sweep,
  flowers, wall departure and parking. Set `ROUTE_VERIFIED = true` only after verification.
- Build and deploy to the Robot Controller to see `BIOBUZZ: Two Hive Tips + Park` in the AUTO list.
  It has no `@Disabled` annotation; route verification remains required before even a dry-run move.
- Determine whether `FEEDER` is `INTAKE` or `SERVO`; default `NONE` deliberately blocks live operation.
  Only the selected feeder is used during shooting. The `intake` motor is independently commanded
  during collection; if it also feeds the shooter, both roles share the same hardware instance.
  Shooter and feeder stay off during pickups. Confirm signed powers and `FEED_PULSE_SECONDS` safely.
- Calibrate `PICKUP_INTAKE_POWER`, `PICKUP_DRIVE_POWER`, `PICKUP_HOLD_SECONDS`, all pickup positions
  and intake reach. Set `PICKUPS_VERIFIED = true` only after testing both flowers and garden.
- No pollen-counting sensor is configured. Default `TIMED_POLLEN_COUNTING_VERIFIED = false` blocks
  live operation. Enable only after establishing one pollen per feed pulse and four per timed pickup.
  For reliable measured counts and missed-shot detection, add appropriate sensors rather than assuming
  a completed timer proves collection/scoring. Expected count and UNKNOWN status appear in telemetry.
- Tune both `SHOOT_RPM_*` and flywheel direction to launch INTO an **upward-facing opening**.
  G417 prohibits intentionally tipping by hitting exterior surfaces or physically manipulating the hive.
  The AprilTags themselves are not the shooting target.
- The lens is currently 14.8 inches above the floor. Configure its measured translation AND mounting
  pitch in Limelight. `ROBOT_ORIGIN_HEIGHT_INCHES` is the field height of the configured robot-space
  origin, NOT necessarily lens height. Calibrate the provisional LOW/HIGH height bands in both states.
- Configure Limelight pipeline 0 for 36h11 tags of 3.25 inches. Confirm robot-space ranges, full-tag
  visibility and same-tag transition detection at BOTH shooting poses. Low-face occlusion can prevent
  confirmation; do not replace lost visibility with assumed success.
- Four pollen are preloaded per robot (page 84). Each own-side flower starts with four pollen and
  the own garden with four (page 83): 4 + 4 + 4 + 4 = 16 available for this routine. Coordinate with
  your teammate so these supplies are not removed before the programmed pickups.
- Only then set `SHOOTING_AND_CAMERA_VERIFIED = true` and `DRY_RUN = false` for live testing.

## Failure and time handling

Each drive leg has a 5-second timeout. Each four-shot batch has a 6-second timeout.
Pickup dwell defaults to 1.5 seconds after the slow intake creep.
Scoring is interrupted at 21 seconds to reserve 7 seconds of the 28-second software budget for parking.
A failed batch, interrupted pickup or missing required tip skips remaining scoring and tries parking
via the outside corridor. Lack of a tip after batches 1/3 is intentional and is NOT a failure.
A Pinpoint fault, movement timeout or STOP stops the routine rather than attempting blind recovery.
All initialized mechanisms and drive motors stop on every exit, including exceptions.
Time reserve does not guarantee parking: measured speed and total route duration must fit the budget.
This expanded route is substantially longer, especially the garden round trip. It may park before
completing the full requested sequence at the current conservative speed; do not extend AUTO past
30 seconds. Optimize/test motion and mechanism timing before using it in a match.
Endpoint bounds checks are not obstacle avoidance; there is no dynamic collision or opponent detection.

## Files and tests

- `opmodes/BiobuzzHiveAuto.java`: hardware orchestration, INIT selection, motion and parking.
- `navigation/HiveAutoRoute.java`: nominal mirrored red/blue waypoints and tag clusters.
- `localization/HiveTipDetector.java`: fresh-frame same-tag transition evidence.
- `control/HiveShotController.java`: bounded nonblocking flywheel/feed/wait states.
- `control/PollenBatchInventory.java`: expected four-element inventory and interruption handling.
- `config/HiveAutoConfig.java`: explicit commissioning gates and provisional tunables.
- `localization/AprilTagTracker.java`: exposes the SDK Limelight frame timestamp.

Unit tests cover mirrored routes, endpoints, outer-corridor recovery, missing/wrong/repeated tags,
high-to-low confirmation, RPM/pose/connection interlocks, four-pulse batches with/without tip requirements,
16-pollen sequencing, refill limits, interrupted inventory, pickup geometry and failure timeouts.
Sampled nominal segments are checked against field bounds, own-half footprint and a conservative
expanded frame rectangle, not actual field CAD or moving hive geometry.
They do not simulate trajectories, pollen physics, damper contact or physical field collisions.
