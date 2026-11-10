# Limelight AprilTag pipeline

Pipeline 0 was named `Tag_24`, with `fiducial_idfilters: "20,24"`.
That filter excludes tag 35 even when the browser's detection overlay identifies it.
The pipeline name itself is not a filter.

`AprilTagTracker` now switches to pipeline 0 during INIT, reads its current
configuration, clears `fiducial_idfilters` to an empty string (all IDs), sets
`fiducial_size` to 82.55 mm (3.25 inches), and
checks the configuration again. This applies to every OpMode using the tracker.
The update preserves all other settings and does not request `flush=1` (persistent
storage). Do not edit/switch pipelines in the browser concurrently with INIT.
Other camera-side rejection filters are intentionally unchanged.

`RobotConfig.Localization.LIMELIGHT_API_HOST` must match the camera/server address
in the Control Hub's Limelight hardware configuration. It defaults to `172.29.2.1`,
not the Mac/Hub DHCP client address. If the camera is unreachable, uses an unknown
schema, or has the wrong active pipeline, the update reports failure and existing
tag polling continues. Set `ALLOW_ALL_APRILTAG_IDS` to false to leave camera ID filters
alone; tag size is still set from `RobotConfig.Localization.APRILTAG_SIZE_MM`.

Run **Test: Limelight** after deploying. Its INIT and running telemetry show
`Pipeline switch` and `AprilTag pipeline config`; expect `ALL IDs, size 82.55 mm
(verified)` or the same settings marked `(already configured)`. The status describes the INIT configuration check,
not continuous monitoring of later browser changes. Keep the display LIVE.
With tag 35 visible, `/results` should contain that ID in `Fiducial` and `v:1`.

For a persistent camera-only fix, clear the AprilTag ID filter in the browser,
set tag size to 82.55 mm, and save the pipeline. No robot software deployment is
required for that manual fix.

The supplied pipeline's `fiducial_size` was 165.1 mm (6.5 inches); INIT now corrects
it to 82.55 mm for the manual's 3.25-inch BIOBUZZ tags. Confirm the physical
black-square size matches this value. A size mismatch affects pose/distance scale.
Without calibrated pan enabled, the code preserves camera mount/extrinsics.
Calibrated camera pan updates yaw and lens offsets as described in
[Camera pan setup](LIMELIGHT_PAN_SETUP.md); those settings still require physical calibration.

Unit tests check restricted/already-empty filters, preservation of unrelated
settings and the original configuration, size updates when IDs are already allowed,
the opt-out ID filter, and rejection of invalid sizes or an unexpected schema.
Actual camera HTTP behavior still requires the hardware test above.

## Robot-relative coordinates and mounting validation

For `Fiducial.t6t_rs`, the SDK exposes X right, Y down, Z forward without changing
the axes. `AprilTagTracker` converts meters to inches, then uses
`AprilTagObservation.fromLimelightRobotSpaceInches` to map forward = Z, left = -X,
and height = -Y. Mounting offsets/rotation have already been applied by Limelight;
the code does not apply them a second time. This corrects the previous direct
X/Y/Z labeling in telemetry and all downstream tracking consumers. Do not use this
mapping for field-space odometry or `botpose`, which have different semantics.

The recorded Nov 8 tag-35 frame now yields forward 21.2, left 4.9, height 36.2,
and range 42.3 inches. Range does not change with an axis/sign conversion.
Regression tests cover all six axis directions and the recorded frame.

The confirmed lens height is 17 inches (LL Up = 0.4318 m for a floor-level robot
origin), superseding the earlier 14.8-inch measurement. The browser configuration
also shows LL Forward = 0.0254 m, LL Right = -0.09525 m, and pitch = 45 degrees.
These mounting settings are preserved, not automatically replaced by the code.
The 36.2-inch computed height still disagrees with the reported 15-inch tag-center
height if the tag remains in the same place; verify the physical pitch and offsets.
Fixing the axes alone does not resolve a mounting calibration error.

After deploying, run **Test: Limelight** in LIVE mode with a known tag position.
A centered tag 25 inches ahead and 15 inches above a floor-level robot origin
should report approximately forward 25, left 0, height 15, range 29.2 inches.
Any odometry consistency baseline must be captured again after deploying this fix.
