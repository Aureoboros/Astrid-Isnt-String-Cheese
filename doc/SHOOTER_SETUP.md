# Shoot1 RPM setup

Confirmed motor: **goBILDA 5202/3/4 series, 6000 RPM, 1:1**, with
**28 quadrature encoder ticks per output-shaft revolution**. Connect motor power
and encoder to **Expansion Hub port 2**, configured as **Shoot1**.

The SDK's generic `goBILDA5202SeriesMotor` profile uses 2786 ticks/revolution,
99.5:1 gearing and 60 RPM. It does not describe this installed motor. Incorrect
ticks/revolution distort both degree-based velocity commands and measured RPM,
and large commands can clip at the hub's 32767 ticks/second limit.

## Where the correction lives

- [RobotConfig.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/RobotConfig.java):
  `ShooterMotor.TICKS_PER_REVOLUTION`, `GEAR_RATIO`, `MAX_RPM`.
- [GoBilda6000RpmMotor.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/GoBilda6000RpmMotor.java):
  registered custom motor profile **goBILDA 6000 RPM (1:1, 28 ticks/rev)**.
- `SingleShooterSubsystem` applies the corrected metadata to Shoot1 at initialization,
  covering TeleTourney, Test: Shooter, and Test: Attachments. `ShooterSubsystem`
  also corrects Shoot1; Shoot2 remains unchanged because its installed model is unconfirmed.
  The profile is cloned per motor, so drive/intake metadata is not changed.
- [.run/RobotConfig.xml](../.run/RobotConfig.xml) now assigns the custom profile to
  Shoot1 on Expansion motor port 2. Other older entries in that reference XML are
  not reconciled; do not replace the entire active hub configuration blindly.

Deploy the rebuilt Robot Controller app first. The runtime correction works even
if the active hub configuration still selects the generic series profile. For a
persistent hardware configuration, select the new custom profile for Shoot1,
save and activate the configuration. XML edits alone do not update the hub.

## Check on the robot

TeleTourney starts with Shoot1 **OFF**, **4800 RPM selected**, **REVERSE**.
X toggles ON/OFF; D-pad up/down changes speed by 100 RPM per fresh press on either
controller, bounded to 0–6000. Holding a button does not repeatedly adjust speed.

Telemetry shows selected magnitude, signed target, actual magnitude, signed actual,
encoder ticks/revolution and raw signed encoder ticks/second. At steady reverse
4800 RPM, expect approximately **−2240 ticks/second** and **−4800 signed RPM**;
actual speed may differ under load. At 4900 RPM the target is approximately
−2287 ticks/second. The hub rounds velocity commands to whole ticks/second.
The separate Shooter Test still starts with 6000 RPM selected and forward direction.

Check with the launcher secured and its discharge area clear. Verify telemetry
reports **28 ticks/revolution** before spinning. If the motor spins but measured
speed remains zero, stop and inspect the encoder connection on port 2. If measured
RPM varies significantly from target, check battery voltage, load and velocity
PIDF tuning; the profile correction does not guarantee physical speed accuracy.

Desktop tests verify profile isolation, registration annotations, reverse RPM
conversion, distinct 100-RPM commands and bounds. Physical feedback and custom
profile availability in the Driver Station still require on-robot verification.
