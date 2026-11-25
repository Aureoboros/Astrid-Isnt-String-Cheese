# Limelight positional pan mount

`servoMotor` is a **Full Range Servo** on **Control Hub servo port 0**. It pans
left/right about a vertical pivot; upward pitch is fixed. It is not a pollen feeder.
Pinpoint remains the robot-position source; camera pan never changes Pedro's pose.

Horizontal image-only tracking is enabled in `config/CameraPanConfig.java` with
`IMAGE_ONLY_ENABLED = true`. The operator confirmed increasing servo position
turns the camera **left**, so `TX_TO_POSITION_SIGN = -1`: a tag to the right
(positive image `tx`) decreases servo position. MIN is **0.05**, CENTER/START
**0.2**, and MAX **0.35**, with verified travel. Tracking starts after Play in
TeleTourney and Test: Limelight; drivetrain and mechanism buttons are unchanged.

Yaw/geometry-aware tracking remains disabled (`ENABLED = false`,
`GEOMETRY_VERIFIED = false`). While image-only tracking is active, robot-relative
forward/left/range/height and AprilTag-versus-odometry validation are **withheld**
because the camera yaw is not calibrated. Pinpoint positioning continues normally.
This mode does not upload guessed camera mounting angles. Do not turn the geometry
flags on to bypass calibration.

## Quick tracking test

Run **Test: Tag Servo Tracking** to test only the Limelight and camera servo.
After Play, the servo starts at **0.2**, with tracking **ON**. Hold **D-pad left/right**
(or **A/B**) on either gamepad to pan the camera physically left/right. Manual
input overrides tracking, stepping **0.005** at most every **0.1 seconds**. Release
the buttons and the test automatically resumes aligning the visible tag after
settling and fresh-frame checks. Increasing servo position pans left, as verified;
D-pad no longer changes the feedback calibration.

**X** pauses/resumes automatic tracking; **Y** centers and pauses. Manual movement
while paused does not re-enable tracking on release: press X to resume. Opposing
manual directions hold position without automatic corrections. Tag ID, image tx,
servo position, manual/auto/paused mode, and status appear in telemetry. No drive
or shooter motors are initialized.

To test: place a tag in view, press Play, hold a manual pan button briefly to move
it away from image center, then release and watch tx approach zero. You can also
move the tag or turn the robot; the camera follows the changing tag bearing. Do
**not** force the powered servo shaft/mount by hand. The positional servo has no
angle feedback and may resist or be damaged. Keep the tag within view: if lost,
the test holds its last position, then reacquires a visible tag; it does not scan
for an off-screen tag.

The servo centers the tag horizontally within a **2-degree deadband**, not
vertically: pitch is fixed. Corrections are limited to **0.01** per step, at least
**0.5 seconds** apart, with settling/fresh-frame checks. Lost targets hold position;
there is no blind sweep. A tag beyond the safe travel limits cannot be centered.

## Commissioning

1. Establish mechanically safe normalized MIN, CENTER, and MAX servo positions,
   checking servo specifications, linkage, hard stops, and cable strain. Disconnect
   the linkage for initial servo setup if necessary. Enter these positions and set
   `TRAVEL_VERIFIED = true` only after verifying them; leave `ENABLED = false`.
2. Run **Test: Servo Motor**. The first B press enables manual positioning at **0.2**;
   press Play first; there is no movement during INIT. D-pad left/right decreases/
   increases position by **0.005** per fresh press, clamped to **0.05–0.35**. A
   requests center (**0.2**), and B again disables further movement while holding position.
   Re-enabling holds the last position; it does not reset to 0.2 each time.
   These are increasing/decreasing servo positions, not guaranteed physical left/right.
   The test initializes no drivetrain or Limelight. Do not trust robot-space tag
   poses during this isolated manual test: it does not update camera extrinsics.
3. Measure physical camera yaw at MIN/CENTER/MAX and record the corresponding
   Limelight UI yaw values as `YAW_AT_MIN/CENTER/MAX_DEGREES`. A monotonic,
   three-point piecewise-linear mapping is used; either servo direction is supported.
   Verify the mapping at intermediate positions, not just the endpoints.
4. Set `TX_TO_POSITION_SIGN` to +1 or -1: choose the servo position direction that
   reduces a positive tag `tx` error. Verify it with small manual steps; a wrong sign
   drives toward a limit instead of centering. `UI_YAW_TO_CCW_SIGN` relates positive
   UI yaw to physical counterclockwise/left pan; enter +1 for same direction or -1
   for opposite direction.
5. At a nonzero manually configured UI yaw, inspect `/results` `t6c_rs[5]`. Set
   `RESULT_YAW_SIGN` to +1 if its sign matches UI yaw or -1 if opposite. The pose
   gate compares frame metadata, rather than assuming all firmware conventions agree.
6. Measure the pivot's forward/right/up coordinates from the robot origin and the
   lens offset from the pivot at CENTER. All translations use meters. The initial
   pivot defaults are the previously measured CENTER lens position (1 inch forward,
   3.75 inches left, 17 inches up), valid as a pivot only when the lens lies on axis.
   If off axis, enter actual pivot and lens offsets: code rotates lens translation
   as well as yaw. Lens height remains constant for a truly vertical pan axis.
7. Verify fixed roll/pitch. Pitch defaults to 45 degrees, but the earlier 15-inch-high
   test tag produced a computed 36.2-inch height: that discrepancy is not resolved
   by pan code. Verify physical pitch and tag size before trusting distance/height.
8. Set a settling time sufficient for the worst allowed move, including initial
   startup positioning and centering from either limit. Check distances and height against measured tag
   locations at center and both sides. Set `GEOMETRY_VERIFIED = true` only after
   verification, then enable automatic pan with `ENABLED = true` and redeploy.

## Controls and supported OpModes

| OpMode | Camera controls |
| --- | --- |
| TeleTourney | Automatic image-only tracking after Play. Hold D-pad LEFT/RIGHT on either controller for manual pan; release to resume tracking. X/Y control Shoot1/intake; B toggles gateMotor; A re-anchors and Back clears the tag/odo check. |
| Test: Limelight | X toggles tracking; Y requests center; A freezes display only |
| Limelight TeleOp (legacy) | B toggles tracking request; movement/mechanism workers remain separate |
| BIOBUZZ: Two Hive Tips + Park | Automatic, no gamepad controls after Play; restricts target to active cell cluster |
| Test: Servo Motor | Isolated manual position calibration only |
| Test: Tag Servo Tracking | Auto alignment starts ON; hold D-pad left/right or A/B for manual pan, release to resume; X pause/resume; Y center/pause |

Both gamepads are supported. The automatic controller commands `START_POSITION`
(**0.2**) only after Play in these integrations; explicit center requests use **0.2**.
Robot-space measurements remain withheld until yaw/geometry calibration is completed.
**Test: Attachments** also enables its manual servo at 0.2 after Play.
Other AprilTag/navigation tests keep the camera
stationary; do not move it externally while relying on their robot-relative poses.
After using image-only tracking, re-center the mount and verify the browser's
stationary-camera extrinsics before using those tests: image-only mode leaves the
last servo position on STOP and does not update Limelight mounting fields.
Existing dual-shooter requirements and autonomous commissioning gates are unchanged.
Live hive AUTO now requires the intake feeder; `SERVO` cannot command the camera.
Dry-run AUTO initializes no camera servo or Limelight.

A center request in Test: Limelight does not disable tracking; use X to disable tracking first if
the camera should remain centered instead of resuming tag following. Commanded
position/yaw display NaN until the automatic controller has actually started.

## Tracking and measurement safety

Tracking uses image-space AprilTag `tx`, not the robot-space left offset. A bounded
proportional correction moves the servo by at most `MAX_POSITION_STEP`; a deadband
reduces chatter. It keeps the selected ID through short losses, then can choose an
actually visible tag nearest the image center. Losing all tags holds the last
position; there is no blind search sweep. Center requests work even without a tag.

In image-only mode robot-space measurements stay blocked throughout. In calibrated
mode (`ENABLED = true`), each pan step immediately blocks them. A single
background worker reads the current pipeline, changes only the six mounting fields,
POSTs without `flush=1`, and verifies the response. Drive control never waits for
these HTTP requests. Afterwards, a settling delay and distinct new frame with
matching `t6c_rs` metadata are required before publishing forward/left/height/range.
Image-space aiming remains available during that gap. HTTP failure holds the mount
and leaves robot-space measurements withheld; stop/restart after fixing configuration.
Do not edit/switch the pipeline in the browser while automatic pan runs.

The hive AUTO holds camera tracking after leaving its WAIT_HIGH shot state so that
camera motion does not interrupt a feed pulse. It also holds during pollen pickup.
This adds no robot aiming/ballistic controller and does not relax scoring safety gates.

Telemetry shows pan status, tracking enabled, commanded servo position/yaw, locked
tag ID, raw aiming-tag count, and robot-space pose readiness. These are **commanded**
angles: the servo has no measured-angle feedback. Calibration, backlash, servo lag,
and camera latency still limit accuracy. STOP prevents new commands and leaves the
last position command unchanged; it never blindly sweeps to center or disables all
hub servo PWM channels. The SDK/hub may remove holding torque on STOP, so physical
holding is not guaranteed. An already-sent camera HTTP request cannot be undone by STOP.

## Verification

Desktop tests cover yaw interpolation, both direction signs, bounded corrections,
off-pivot translation, target locking/loss, mount-field preservation, result yaw
signs, pose settling/freshness gates, and refusal to move an uncalibrated servo.
Image-only tests additionally cover verified left-turning feedback, deadband,
travel limits, fresh-frame barriers, target loss/reacquisition and ID restrictions,
manual/automatic control exclusion, and direction-change disarming.
The tracker-test controls have separate tests for manual override/repeat timing,
automatic alignment after release, changing tag bearing, pause/center priority,
opposing manual buttons, limits, and holding when no tag is visible.
The actual servo, HTTP endpoint behavior, camera geometry, and motion during
driving still require the commissioning checks above; passing unit tests is not
physical calibration or deployment.
