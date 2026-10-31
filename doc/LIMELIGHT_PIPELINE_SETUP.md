# Limelight AprilTag pipeline

Pipeline 0 was named `Tag_24`, with `fiducial_idfilters: "20,24"`.
That filter excludes tag 35 even when the browser's detection overlay identifies it.
The pipeline name itself is not a filter.

`AprilTagTracker` now switches to pipeline 0 during INIT, reads its current
configuration, clears only `fiducial_idfilters` to an empty string (all IDs), and
checks the configuration again. This applies to every OpMode using the tracker.
The update preserves all other settings and does not request `flush=1` (persistent
storage). Do not edit/switch pipelines in the browser concurrently with INIT.
Other camera-side rejection filters are intentionally unchanged.

`RobotConfig.Localization.LIMELIGHT_API_HOST` must match the camera/server address
in the Control Hub's Limelight hardware configuration. It defaults to `172.29.2.1`,
not the Mac/Hub DHCP client address. If the camera is unreachable, uses an unknown
schema, or has the wrong active pipeline, the update reports failure and existing
tag polling continues. Set `ALLOW_ALL_APRILTAG_IDS` to false to leave camera filters alone.

Run **Test: Limelight** after deploying. Its INIT and running telemetry show
`Pipeline switch` and `AprilTag ID filter`; expect `ALL IDs allowed (verified)` or
`ALL IDs already allowed`. The status describes the INIT configuration check,
not continuous monitoring of later browser changes. Keep the display LIVE.
With tag 35 visible, `/results` should contain that ID in `Fiducial` and `v:1`.

For a persistent camera-only fix, clear the AprilTag ID filter in the browser and
save the pipeline. No robot software deployment is required for that manual fix.

Distance calibration is separate: the supplied pipeline's `fiducial_size` is
165.1 mm (6.5 inches). For the manual's 3.25-inch BIOBUZZ tags, set the size to
82.55 mm after confirming the physical black-square size. A size mismatch affects
pose/distance scale; this code deliberately does not change it or the camera mount.

Unit tests check restricted/already-empty filters, preservation of unrelated
settings and the original configuration, and rejection of an unexpected schema.
Actual camera HTTP behavior still requires the hardware test above.
