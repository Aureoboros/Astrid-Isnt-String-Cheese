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
The code does not change the camera mount/extrinsics; those still need calibration.

Unit tests check restricted/already-empty filters, preservation of unrelated
settings and the original configuration, size updates when IDs are already allowed,
the opt-out ID filter, and rejection of invalid sizes or an unexpected schema.
Actual camera HTTP behavior still requires the hardware test above.
