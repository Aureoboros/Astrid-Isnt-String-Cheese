# Red / Blue movement simulation

`HiveRouteMovementSimulationTest` runs the current Blue/Red three-batch plan using
the production route coordinates, configuration, inventory model, and shot state
machine. The test-only `HiveRouteMovementSimulator` supplies idealized movement
and synthetic vision/RPM. No robot hardware is commanded and no runtime settings
are modified.

This is **not** a simulation of the Android OpMode, Pedro PID/follower dynamics,
motor directions, Pinpoint, floor friction, acceleration, camera calibration, or
real nectar counts. Speed/power scaling is assumed linear, not measured. Position
and heading interpolate concurrently; each leg settles at its exact endpoint for
at least 0.1 seconds. This is slightly more conservative than the executor's
current 3-inch/5-degree practice endpoint tolerance (1.5 inches in live mode).
The default unmeasured tag map skips the
position correction, just as the executor does. A measured-map correction and its
latency are not modeled.

## Results under illustrative assumptions

The nominal profile assumes 100 in/s translation and 360 deg/s rotation at full
power: 35 in/s and 126 deg/s at the configured 0.35 drive-power cap. Pickup creeps
use the configured 0.15 power. The slower profile halves both assumed full-power
rates. These are example rates, **not estimates of this robot's actual speed**.

| Scenario (both alliances) | Modeled elapsed time | Result |
| --- | --- | --- |
| Nominal driving-only, timed baseline | 16.38 s | All five stages complete; 319.5-inch route; parked |
| Slower driving-only, timed baseline | 24.96 s | Three stages complete; scoring interrupted; parked |
| Stalled translation, timed baseline | 5.02 s | Leg timeout; no blind parking attempt |
| Nominal motion plus synthetic live timing, timed baseline | 23.06 s | Three stages complete; interrupted approaching flower 2; parked |
| Very slow driving-only, current unlimited practice | 71.42 s | All five stages complete; 319.5-inch route; parked |

The unlimited-practice scenario uses illustrative full-power rates of 20 in/s and
72 deg/s, no overall deadline, and no movement-leg timeout. Separate tests confirm
an individual leg can exceed 30 seconds and the full route still completes, and
a stalled simulation waits past the old deadline until a synthetic Driver Station
STOP at 40 seconds. The simulator's synthetic STOP is not a runtime time limit.

These results include six-inch flower depth with three inches of extra reach for
both flowers and another four inches for flower 2,
the earlier flower-2 heading turn, six-inch opposite-shot
backoff, and corner parking. Red ends at **(-61, 48)** and Blue at **(61, -48)**.
Sampled trajectories are
180-degree mirrors. Tests check the starting-size footprint stays inside the
field/own half and outside the nominal hive frame. Flower openings, hive cell
sweep, obstacles, teammates, and actual intake reach are not represented.
The pickup approaches stop six inches short of the original approach; targets
stop three inches short of the original pickup position for flower 1 and at the
one inch beyond the original position for flower 2, making 11-inch and 15-inch creeps respectively.
Pickup arrival uses a one-inch tolerance in the OpMode; simulation still settles
at exact endpoints. Exact-endpoint simulation does not
validate intake reach, clearance with real tracking error, or overshoot. Test the
pickup approaches cautiously on the physical field.

The synthetic live scenario calls `HiveShotController` with ideal RPM readiness,
high-tag stability after 0.3 seconds, and a required tip 0.4 seconds after pulse
four completes. It includes the configured 1.5-second pickup dwells. Actual delays
may differ substantially; the scenario does not bypass live commissioning gates.

The timed baseline/live software budget remains **28 seconds**, reserving seven
for parking: scoring stops at **21 seconds**. The new Blue/Red driving-only entries
now use unlimited practice timing, so those overall cutoffs do not apply to them.
Disable the Driver Station timer separately for practice; the app cannot prevent
the Driver Station from stopping an OpMode. A driving-only pass does not establish that
three real four-nectar batches fit. Measure transit, turn, pickup, spin-up, and tip
confirmation times before live operation. Unlimited practice is restricted to
`DRY_RUN`; switching to live mode restores the timed limits.

## Build and test status

The seven simulation tests pass for both alliances. The earlier full-suite run had 248 tests:
245 passed, and three existing `MecanumDriveTest` assertions failed. Those assert
old forward/strafe/rotation power patterns that differ from the current configured
mixing. The simulation additions do not change drive mappings, and no production
motor-direction changes were made to force these assertions to pass. Autonomous,
route, start-policy, shot, inventory, vision, and Pedro-layout tests passed.

## Run again

From the project root:

```sh
./gradlew -Dorg.gradle.java.home='/Applications/Android Studio.app/Contents/jbr/Contents/Home' :TeamCode:testDebugUnitTest --tests '*HiveRouteMovementSimulationTest' --no-daemon
```

Detailed stage timestamps are in the simulation test's standard output in
`TeamCode/build/test-results/testDebugUnitTest/TEST-org.firstinspires.ftc.teamcode.navigation.HiveRouteMovementSimulationTest.xml`.
The HTML test report is `TeamCode/build/reports/tests/testDebugUnitTest/index.html`.
Change the illustrative profiles in the test to match measured on-robot rates;
these changes affect only offline simulation, not drivetrain configuration.
