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
1.5-inch/5-degree endpoint tolerance. The default unmeasured tag map skips the
position correction, just as the executor does. A measured-map correction and its
latency are not modeled.

## Results under illustrative assumptions

The nominal profile assumes 100 in/s translation and 360 deg/s rotation at full
power: 35 in/s and 126 deg/s at the configured 0.35 drive-power cap. Pickup creeps
use the configured 0.15 power. The slower profile halves both assumed full-power
rates. These are example rates, **not estimates of this robot's actual speed**.

| Scenario (both alliances) | Modeled elapsed time | Result |
| --- | --- | --- |
| Nominal, current driving-only mode | 15.78 s | All five stages complete; 322.5-inch route; parked |
| Slower driving-only profile | 25.06 s | Four stages complete; scoring interrupted; parked |
| Stalled translation | 5.02 s | Leg timeout; no blind parking attempt |
| Nominal motion plus synthetic live shot/pickup timing | 22.82 s | Three stages complete; interrupted approaching flower 2; parked |

Red ends at **(-60, 36)** and Blue at **(60, -36)**. Sampled trajectories are
180-degree mirrors. Tests check the starting-size footprint stays inside the
field/own half and outside the nominal hive frame. Flower openings, hive cell
sweep, obstacles, teammates, and actual intake reach are not represented.

The synthetic live scenario calls `HiveShotController` with ideal RPM readiness,
high-tag stability after 0.3 seconds, and a required tip 0.4 seconds after pulse
four completes. It includes the configured 1.5-second pickup dwells. Actual delays
may differ substantially; the scenario does not bypass live commissioning gates.

The software budget remains **28 seconds**, reserving seven for parking: scoring
stops at **21 seconds**. A driving-only pass therefore does not establish that
three real four-nectar batches fit. Measure transit, turn, pickup, spin-up, and tip
confirmation times before considering route or timing changes. No timeout or
safety-gate changes were made for these tests.

## Build and test status

The four simulation tests pass for both alliances. The full suite ran 248 tests:
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
