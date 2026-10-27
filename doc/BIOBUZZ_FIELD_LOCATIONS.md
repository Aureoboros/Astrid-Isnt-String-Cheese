# BIOBUZZ field locations and autonomous start selection

This implementation follows the supplied **2026–27 BIOBUZZ Competition Manual TU06**:
section 9.2 (p. 64), section 9.4 / Figures 9-4 and 9-5 (p. 67), G304 (p. 104),
Figure 11-1 (p. 105), and R102 (p. 121).

The manual defines legal starting **regions**, not mandatory robot-center coordinates. G304
requires a robot wholly on its own alliance half, touching a perimeter wall, outside loading
zones, and not contacting a flower or its scoring volume. Other starting requirements, including
four POLLEN preloads and a stationary starting configuration, still apply. The outside alliance
areas are for drive teams, not robot starting positions.

## Coordinate frame and location matrix

`BiobuzzFieldMap` uses the nominal 144-by-144-inch field and 24-inch tiles. Looking into the field
from the audience, `(0, 0)` is the audience-left inside-wall corner. X increases rightward from
column A to F; Y increases away from the audience from row 1 to 6. Heading 0° faces right / +X,
90° faces away / +Y, 180° faces left, and 270° faces the audience. All poses describe the **robot
center**. Pedro accepts these field poses in inches with headings converted to radians.

```text
Far wall / away from audience (+Y)
 A6 B6 C6 | D6 E6 F6
 A5 B5 C5 | D5 E5 F5
 A4 B4 C4 | D4 E4 F4
 A3 B3 C3 | D3 E3 F3
 A2 B2 C2 | D2 E2 F2
 A1 B1 C1 | D1 E1 F1
   RED   |   BLUE
Audience: origin at lower left, +X toward the right
```

`getLocationMatrix()` returns `[row - 1][column - 'A']` tile-center waypoints. A1 is `(12,12)`,
F6 is `(132,132)`. `getLocation("B2")` resolves the nominal tile center `(36,36,0°)`;
`tileCenter('B', 2, 90)` creates the same position facing 90°. `tileAt(x,y)` identifies the
robot-center tile and reports OUTSIDE FIELD / INVALID POSE for invalid coordinates.

This is **not a free-space, collision, or scoring map**. For example, the central C3/D3/C4/D4
tiles contain the hive structure. Loading zones, flowers, game pieces, other robots, and robot
footprint must be considered when programming any route. Field dimensions are nominal; use
measured field/CAD dimensions and the manual's tolerances when calibrating positions.

## Team starting presets

These are wall-aligned rectangular-robot placement templates chosen from G304's permitted regions,
not exact coordinates prescribed by Figure 11-1. Defaults assume an **18-inch length × 18-inch
width** starting footprint. Set `RobotConfig.Autonomous.START_LENGTH_INCHES` and
`START_WIDTH_INCHES` to the actual bumper-to-bumper starting dimensions, then rebuild. Length is
front-to-back; width is side-to-side. Wall-normal center offsets are half the configured length.

| Preset | Center X / Y (in), with 18-in length | Heading | Contact wall |
| --- | --- | --- | --- |
| `RED_A3_WEST` | 9 / 64 | 0° | Red/left wall |
| `RED_A6_WEST` | 9 / 132 | 0° | Red/left wall |
| `RED_B1_AUDIENCE` | 36 / 9 | 90° | Audience wall |
| `RED_B6_FAR` | 32 / 135 | 270° | Far wall |
| `BLUE_F4_EAST` | 135 / 80 | 180° | Blue/right wall |
| `BLUE_F1_EAST` | 135 / 12 | 180° | Blue/right wall |
| `BLUE_E6_FAR` | 108 / 135 | 270° | Far wall |
| `BLUE_E1_AUDIENCE` | 112 / 9 | 90° | Audience wall |

Each blue preset is the 180° field rotation of the corresponding red preset. The deliberate
along-wall offsets provide separation from nearby flowers; verify actual structures, tape,
preloads, and bumper placement on the real field. Software cannot certify physical legality or
discover the robot's start automatically. Use an alignment jig if needed. Measured pod offsets
must reference the same robot center used by this map.

## Run the autonomous OpMode

Select **BIOBUZZ: Field Location Auto** in Competition, enable the Driver Station's **30-second
AUTO timer** (G305), and press INIT. Keep the robot still during Pinpoint calibration.

| Input during INIT (either controller) | Action |
| --- | --- |
| D-pad left / right | Select red / blue alliance |
| D-pad up / down | Cycle that alliance's four start presets |
| Left / right bumper | Cycle programmed autonomous plans |
| A | Confirm robot physically matches the displayed wall position and heading |

Every selection change invalidates confirmation; release A and press it separately after placing
the robot. INIT reads sensors and assigns the field origin but does not drive the motors. Keep
the robot motionless after confirmation. At Play, the selected start and plan are frozen and the
Pinpoint field pose is anchored to that start without recalibrating the gyro.

`POSE_ONLY` is the default and commands no movement. `LEAVE_WALL_12_INCHES` is an opt-in
demonstration: it resolves `<START_NAME>_CLEAR_WALL` and drives 12 estimated inches inward,
preserving the starting heading. Verify clearance before enabling it. B during movement aborts;
Driver Station Stop ends the OpMode. No Limelight or mechanisms are initialized.

The movement plan uses Pedro with Pinpoint as the only pose source, 25% power, a 15-second
per-leg timeout, a 28-second software AUTO budget, and 1.5-in / 5° settling for 0.1 seconds.
Non-READY status or invalid sensor pose aborts movement. All endpoint names and robot-footprint
field bounds are checked before the first leg, but paths do not avoid obstacles automatically.
Motors stop after the route; there is no active position hold afterward.

Telemetry shows selected alliance/start/plan, confirmation, field X/Y/heading, current tile,
raw pod counts, Pinpoint status, coefficient, and named target/error. INIT and the final stopped
display include the 6×6 matrix with `*` marking the robot-center tile.

## Program other named destinations

Register team locations in `BiobuzzFieldAuto` immediately after constructing `field`, for example:

```java
field.registerLocation("RED_APPROACH", new Waypoint(36, 36, 90));
field.registerLocation("BLUE_APPROACH", new Waypoint(108, 108, 270));
```

Then add a `Plan` entry in `BiobuzzAutoRoutes` and return an ordered list of names from
`destinations(start, plan)`. Bumper selection automatically includes new enum entries. For a
route intended only for `RED_B1_AUDIENCE`, an example ordered list is:

```java
Arrays.asList("RED_B1_AUDIENCE_CLEAR_WALL", "RED_APPROACH")
```

Provide a corresponding blue route and reject incompatible starts before movement. Each name
resolves an absolute field pose. The runner creates a straight Pedro path from its current pose
to each destination, so intermediate waypoints must keep the whole robot clear of structures.
Examples are coordinate-programming demonstrations, not tested scoring strategies.

Other existing diagnostics and TeleOps retain their local-origin behavior. In particular,
`TeleTourney` still resets to `(0,0,0)`; this change does not silently transfer absolute field pose
between OpModes or use Limelight to correct it. Pinpoint accuracy still depends on measured pod
resolution, offsets, directions, and initial physical alignment.

Tests: `BiobuzzFieldMapTest`, `BiobuzzAutoRoutesTest`, and `AutonomousStartControlsTest` cover the
matrix, preset symmetry/footprints, named routes, and confirmation behavior. The motor-running
OpMode still needs on-robot verification.
