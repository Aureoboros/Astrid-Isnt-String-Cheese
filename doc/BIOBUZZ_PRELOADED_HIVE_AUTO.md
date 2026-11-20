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
| Flower 1 pickup | (-57, -24, 180°) | (57, 24, 0°) |
| Batches 2 and 3, side 2 | (-12.75, 40, 270°) | (12.75, -40, 90°) |
| Flower 2 pickup | (-24, 57, 90°) | (24, -57, 270°) |
| Park | (-60, 36, 180°) | (60, -36, 0°) |

Pickup poses are provisional robot-center positions, not flower centers. The last
eight inches are an intake creep at reduced drive power, followed by a bounded
dwell in live mode. Confirm intake reach and real field clearances.

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

The shared 28-second budget reserves seven seconds for parking. If time runs short,
the executor interrupts scoring and attempts parking; it does not guarantee all
three batches fit. Measure actual route duration and mechanism timing on the field.
Build/deploy the app to see these entries; no hub deployment is performed by editing source.

See [Red/Blue offline movement simulation](BIOBUZZ_MOVEMENT_SIMULATION.md) for
repeatable geometry/timing tests and their limitations. At illustrative nominal
speed the driving-only route completes; adding shooting/pickup timing reaches
the scoring cutoff early. A dry-run pass is not proof that live scoring fits.
