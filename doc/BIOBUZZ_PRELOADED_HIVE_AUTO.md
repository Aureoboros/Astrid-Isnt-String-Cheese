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

## Files and commissioning

- `opmodes/BluePreloadedHiveAuto.java` and `RedPreloadedHiveAuto.java`: separate
  Driver Station entries, locked starts, and selection of Shoot1-only control.
- `navigation/PreloadedHiveAutoRoute.java`: ordered stages, transit coordinates,
  target tag clusters, and which batches require a tip. The old seven-stage route
  remains available in `BiobuzzHiveAuto`; `FortyEight` is unchanged.
- `opmodes/BiobuzzHiveAuto.java`: shared nonblocking Pedro/Pinpoint executor,
  Limelight position correction, vision tip checks, pickup/feed loop, and cleanup.
- `config/PreloadedHiveAutoConfig.java`: new route approval and Shoot1 settings.
  Defaults are **4800 RPM, reverse**, matching TeleTourney. No Shoot2 is initialized.
- `config/HiveAutoConfig.java`: shared vision/timing, live-mechanism verification,
  feeder selection, and timeouts. `DRY_RUN = true` means real driving but simulated
  shots/pickups. `servoMotor` remains camera pan, not a feeder.

**The operator enabled the new route (`PreloadedHiveAutoConfig.ROUTE_VERIFIED = true`).**
With the current shared `DRY_RUN = true`, Play starts actual driving and simulates
shots/pickups. Check the displayed start, physical clearances and route before Play.
The original `BiobuzzHiveAuto` retains its A-button placement confirmation.
Do not enable live shooting until the shared camera/shooting, pickups, and timed
counting flags are commissioned, `FEEDER = INTAKE` is verified, and `DRY_RUN` is false.
The gate separator is not automatically armed by these routines.

Four feed pulses/four collected nectars are **timing estimates**, not sensor counts.
Low-tag evidence immediately stops feeding, even before four pulses complete;
an early tip does not justify assuming the robot is empty and collecting more.
Measure high/low tag heights, signed intake feed power, pulse duration, and flywheel
settings before live use. Confirm both hive tips physically; vision is only a proxy.

## Slow driving practice: no overall timer

The new Blue/Red entries currently use
`PreloadedHiveAutoConfig.UNLIMITED_DRIVING_TEST = true` while `DRY_RUN = true`.
They have **no overall time limit or parking-reserve cutoff**: Play starts the
complete route and it runs to the park endpoint unless stopped or faulted.
There is also **no movement-leg timeout or Pedro path-end correction timeout**
in this driving-only practice mode. STOP, Pinpoint readiness and finite-pose
checks remain active. INIT telemetry displays the unlimited-practice warning.
If a target cannot be reached, the robot may keep applying power indefinitely:
press STOP if it stalls. Position and heading checks are not bypassed, so removing
timers is not a fix for poor heading control or low-power/friction problems.
Read-only camera relocalization attempts remain bounded so missing tags cannot
hang the route. Live shooting/controller safety timeouts remain unchanged.

Driving-only Blue/Red tests now accept **up to 3 inches of radial position error**
at waypoints (`PRACTICE_POSITION_TOLERANCE_INCHES`). Heading must still be within
5 degrees, with a 0.1-second settling check. Telemetry shows the active tolerance.
Parking/transit legs may finish up to three inches short; pickup legs require
one-inch error or less. The pickup limit is used during arrival and live pickup hold.
Verify actual nectar reach, loading-zone overlap, and clearance on the field.
Switching to live mode restores the 1.5-inch position tolerance; old autos remain
at 1.5 inches; the new pickup check remains at one inch in both modes.
The offline movement simulation still settles at exact endpoints
and does not certify obstacle clearance with three inches of real tracking error.

If flower 2 still stops short, record `Status`, `Stage`, `Move position error / allowed`,
and `Move heading error / allowed` from telemetry. Movement-timeout details remain
available in timed modes; driving-only practice has no movement timers. STOP/fault
checks remain; the current driving-only mode still runs no intake/shooter.

**Disable the Driver Station AUTO timer manually for this practice run.** The app
cannot override a Driver Station stop. This mode is not for competition matches.
Set `UNLIMITED_DRIVING_TEST = false` to restore normal timing. Switching
`DRY_RUN = false` also restores the shared 28-second budget, seven-second parking
reserve and five-second leg timeout; unlimited timing never enables live mechanisms.
The older autonomous routines keep their existing timing behavior.

With live timing, the executor may interrupt scoring and attempt parking; it does
not guarantee all three batches fit. Measure actual mechanism/transit durations.
Build/deploy the app to see these entries; no hub deployment is performed by editing source.

See [Red/Blue offline movement simulation](BIOBUZZ_MOVEMENT_SIMULATION.md) for
repeatable geometry/timing tests and their limitations. At illustrative nominal
speed the driving-only route completes; slow unlimited practice also completes
beyond 30 seconds. Adding shooting/pickup timing in the timed baseline reaches
the scoring cutoff early. A dry-run pass is not proof that live scoring fits.
