# Blue and Red: preloaded hive autonomous

Select **BIOBUZZ: Blue 3 Batches + Park** or **BIOBUZZ: Red 3 Batches + Park** on
the Driver Station. Their alliance is locked by the selected OpMode. Place the robot
at the displayed start; pressing **Play confirms placement and starts the route**.
No A-button confirmation is required, and A has no effect during INIT in these entries.
There are no controller commands after Play; Driver Station STOP aborts.

Only the first hive side starts with three nectars. Under the team's scoring
assumption, four additional nectars tip that side. The opposite side needs eight,
supplied by two separate four-nectar flower pickups. The software attempts:

1. Start with four robot preloads; approach hive side 1 and optionally reconcile
   Pinpoint X/Y with the configured Limelight tag map.
2. Shoot four at side 1 and require a confirmed tag high-to-low transition.
3. Visit the first flower to the robot's left, collect four, and use the outer
   corridor to reach the opposite hive side (side 2).
4. Shoot four at side 2; this batch does **not** require a tip.
5. Visit the next alliance flower, collect four, and return to **the same side 2**.
6. Shoot four more at side 2 and require its tag high-to-low transition.
7. Retreat to the outer corridor and park partly in the alliance loading zone.

The same side-2 tag detector is retained across batches 2 and 3. A tip after only
batch 2 stops further scoring and attempts parking rather than firing at a low cell.
Any interrupted feed or pickup makes inventory unknown and blocks another pickup.
There is no garden pickup, no Shoot2 dependency, and no official score guarantee.

## Starts and endpoints

Coordinates are robot-center inches in the existing field-center map. Blue is Red
rotated 180 degrees. With an 18-inch starting length:

| Location | Red (X, Y, heading) | Blue (X, Y, heading) |
| --- | --- | --- |
| Start | (-36, -63, 90°), RED_B1_AUDIENCE | (36, 63, 270°), BLUE_E6_FAR |
| Batch 1, side 1 | (-12.75, -40, 90°) | (12.75, 40, 270°) |
| Flower 1 pickup | (-54, -24, 180°) | (54, 24, 0°) |
| Batches 2 and 3, side 2 | (-12.75, 46, 270°) | (12.75, -46, 90°) |
| Flower 2 pickup | (-24, 58, 90°) | (24, -58, 270°) |
| Park center: nearest inner corner | (-61, 48, 180°) | (61, -48, 0°) |

Pickup poses are provisional robot-center positions, not flower centers. The last
11 inches (flower 1) or 15 inches (flower 2) are an intake creep at reduced drive power.
Flower depth remains six
inches; the approach stays six inches back from the original approach, but the
pickup target now adds three inches of extra reach toward the flower. The net
target standoff is three inches for flower 1. Flower 2 adds another four inches
(`FLOWER_2_ADDITIONAL_REACH_INCHES = 4`), placing its target one inch farther toward
the flower than the original pickup position; its approach remains unchanged.
`FLOWER_PICKUP_EXTRA_REACH_INCHES = 3` adjusts this reach independently of depth.
Pickup arrival requires **one-inch** position error or less, so the general
three-inch waypoint tolerance no longer cancels the added reach. Flower 2 now
turns to face the flower during the lateral approach and keeps that heading for
the final approach/creep, instead of turning 180 degrees on a short segment.
A bounded dwell follows
in live mode. The opposite shooting pose is six inches farther back along the
robot's heading; both refill batches use it. These adjustments live in
`PreloadedHiveAutoConfig.FLOWER_PICKUP_DEPTH_INCHES`, `FLOWER_PICKUP_EXTRA_REACH_INCHES`,
`FLOWER_2_ADDITIONAL_REACH_INCHES`, `PICKUP_POSITION_TOLERANCE_INCHES`, and
`OPPOSITE_SHOT_BACKOFF_INCHES`.

The park center is at the nearest inner corner of the existing nominal loading-zone
map: Red X=-61, Y=48; Blue X=61, Y=-48 (`RED_PARK_CORNER_X/Y`, mirrored for Blue).
Measure the actual taped zone and adjust those coordinates if needed. Wall-side
corners were not used, because placing the robot center on the wall would put half
the robot outside the field.

Verify real intake/flower clearance and reach slowly; ideal pose checks do not
account for overshoot or extended mechanisms. The three-inch
tolerance does not guarantee wall clearance or that the park center lands exactly
on the taped corner.

## Live mechanism operation

Both Driver Station entries now enable real intake pickups, Shoot1 and gateMotor.
`PreloadedHiveAutoConfig.MECHANISMS_ENABLED = true` records the operator's
validation. Shoot1 is set to **2900 RPM, reverse**; gateMotor uses **60 RPM, forward**.
TeleTourney and the older autonomous entries retain their existing defaults.

The intake runs at the shared pickup power during each final flower approach and
the 1.5-second pickup dwell. At the hive, the gate and intake feed only during the
four-pulse batch. The intake pauses while the gate is stopped, reversing, or below
its movement threshold. Feeding resumes after jam recovery. A persistent gate fault
aborts the batch, stops the mechanisms, invalidates expected inventory and attempts
parking if the drivetrain is healthy. The robot does not attempt another pickup
with unknown inventory.

Each 0.15-second feed pulse is credited from encoder travel at the selected gate
speed, rather than elapsed wall time during a jam. Reverse travel must be regained
before additional feed progress is credited. Four feeds/four picked-up nectars remain
**estimates**, without a counting sensor. STOP stops the mechanisms; none spin in INIT.

The camera servo holds **center 0.20** during these autos. The known center mount
is configured on pipeline 0 in INIT; fresh matching frames after servo settling
are required before robot-space observations are published. This avoids the
uncalibrated moving-pan mode, which withholds tag-height measurements. Live shots
still require visible high-tag evidence, a settled robot pose and Shoot1 RPM readiness.
Low-tag evidence stops feeding immediately. First and final batches require a
confirmed high-to-low transition; the middle batch does not.

The shot controller allows 20 seconds of batch-control time for visibility and
flywheel spin-up. Gate stalls/reversal do not advance a feed pulse. An individual
feed with no sufficient progress aborts after 10 seconds of real time; jam recovery
also retains its three-retry limit. These are mechanism fault guards, separate from
the disabled movement deadlines.

## Files for quick changes

- `opmodes/BluePreloadedHiveAuto.java` and `RedPreloadedHiveAuto.java`: locked alliance,
  Play placement confirmation, live-mechanism and single-shooter selection.
- `config/PreloadedHiveAutoConfig.java`: mechanism enable, Shoot1 RPM/direction,
  gate RPM/direction, route reach/backoff and arrival tolerances.
- `opmodes/BiobuzzHiveAuto.java`: shared executor, centered camera setup,
  pickup/feed coordination and cleanup.
- `control/GateFeedProgress.java`: encoder-based pulse progress and reverse repayment.
- `subsystems/GateMotorSubsystem.java` and `control/GateJamRecoveryController.java`:
  gate motor commands and nonblocking jam recovery.
- `config/HiveAutoConfig.java`: shared feed/pickup power, pulse/dwell durations and
  tag height bands. Its `DRY_RUN = true` still applies to the older entries;
  Red/Blue use their separate mechanism-enable setting.
- `navigation/PreloadedHiveAutoRoute.java`: mirrored route and stage sequence.

## Untimed field testing

The existing `UNLIMITED_DRIVING_TEST = true` setting retains its legacy name but
now keeps these live Red/Blue routes untimed as requested. There is no overall
budget, parking-reserve cutoff, movement-leg timer or Pedro path-end correction
timer. Set it false to restore the shared 28-second budget, seven-second parking
reserve and five-second movement-leg timer. Mechanism enable is independent.

General arrival remains **3 inches / 5 degrees**, settled for 0.1 second; flower
pickup arrival and dwell use **1 inch / 5 degrees**. All previous flower reach,
six-inch opposite-side shooting backoff and corner parking adjustments are shared
by Red and Blue. STOP and finite/READY Pinpoint checks remain active.

Disable the Driver Station AUTO timer for an untimed field test; the app cannot
override a Driver Station stop. Use timed operation for matches.

Build and deploy the app to run these changes. The offline movement simulation
checks mirrored geometry and illustrative timing; controller unit tests check
gated batches and jam behavior. These do not reproduce physical nectar flow.
