# Git pull review

Reviewed merge `2337f42` against its pre-pull parent `71dbf30`.
The pull adds a separate `TeleOpTourney`, automatic-to-teleop pose handoff,
and a method to reset push recovery after an automatic move.

The existing **TeleTourney** mappings and settings survived the merge:

- A reverses intake; Y toggles intake ON/OFF and retains direction.
- X toggles Shoot1, initially 3200 RPM in reverse; D-pad up/down steps by 50 RPM.
- B toggles gate START/HOLD, initially 190 RPM in reverse;
  D-pad left/right steps by 10 RPM, bounded to 0–435 RPM.
- Left bumper re-anchors the tag/odometry comparison; Back clears it.
- Drive and push recovery remain capped at 35%.
- Automatic camera tracking retains the 0.05/0.2/0.35 servo range, smaller
  0.005 steps, 10-Hz limit, 2-degree centering/4-degree resume thresholds,
  and fresh-frame confirmation. Pinpoint remains the position source.
- Gate encoder calibration, explicit velocity PIDF, position HOLD and jam
  recovery remain available, as do the isolated gate diagnostics.

Red/Blue three-batch routes retain the flower reach adjustments, 6-inch shooting
backoff, parking corners and movement tolerances. They remain enabled with live
mechanisms, 2900-RPM Shoot1, 60-RPM gate feeding, and unlimited driving-test timing.

## Driver Station entries

The similarly named OpModes have different behavior. Select **TeleTourney** to
use the controls above.

| Behavior | TeleTourney | Pulled TeleOpTourney |
| --- | --- | --- |
| A | Reverse intake | Drive to autonomous shooting pose |
| Y | Intake ON/OFF, starts OFF | Intake ON/OFF, starts ON |
| X | Shoot1 ON/OFF | Gate START/HOLD |
| B | Gate START/HOLD | Held spit: intake/gate opposite feed direction |
| Shoot1 | 3200 RPM selected, starts OFF | Runs from Play at 3200 RPM |
| Gate | 190 RPM reverse; D-pad left/right adjusts | 60 RPM reverse; fixed setting |
| D-pad up/down after Play | Shooter speed ±50 RPM | Shooter speed ±50 RPM |
| Camera tracking | Automatic Limelight pan | No camera tracking |
| Pose at Play | Own (0,0,0) origin | Saved autonomous pose or selected alliance start |
| Drive power | 35% | Full manual power; 70% during A automatic move |

## Review fixes

- Gate holding in TeleOpTourney now begins after Play, keeping it unpowered in INIT.
- Selecting an alliance in TeleOpTourney selects that alliance's start pose;
  D-pad up restores a saved autonomous pose and its matching alliance. The pose
  lookup also rejects a saved pose belonging to the other alliance.
- Starting a new autonomous clears the previous handoff, including when INIT is
  cancelled. Invalid end poses or a Pinpoint fault cannot retain an older pose.
  Running the existing TeleTourney also invalidates an unused handoff.
- Autonomous camera cleanup runs even if mechanism or follower cleanup throws.
- TeleOpTourney reports actual gate HOLD mode and signed applied/measured RPM,
  keeping encoder direction mismatches visible.

Validation includes the Android debug build and all unit tests, plus new checks
for alliance-specific handoff, invalid/consumed poses and recovery re-arming after
an automatic move. Source review/tests do not establish physical gate encoder
polarity or validate the new TeleOpTourney's straight drive-to-shoot path on the field.
