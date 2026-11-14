package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

/** Dedicated RC configuration entry; the generic series profile describes different gearing. */
@MotorType(ticksPerRev = RobotConfig.ShooterMotor.TICKS_PER_REVOLUTION,
    gearing = RobotConfig.ShooterMotor.GEAR_RATIO, maxRPM = RobotConfig.ShooterMotor.MAX_RPM,
    orientation = Rotation.CCW)
@DeviceProperties(name = "goBILDA 6000 RPM (1:1, 28 ticks/rev)", xmlTag = "GoBilda6000RpmMotor")
public interface GoBilda6000RpmMotor {
  /** Correct this motor instance even when the active Hub XML still selects the generic series. */
  static void configure(DcMotorEx motor) {
    motor.setMotorType(correctedType(motor.getMotorType()));
  }

  /** Clone so a shared SDK profile cannot change the drive motors or another mechanism. */
  static MotorConfigurationType correctedType(MotorConfigurationType original) {
    MotorConfigurationType corrected = original.clone();
    corrected.setTicksPerRev(RobotConfig.ShooterMotor.TICKS_PER_REVOLUTION);
    corrected.setGearing(RobotConfig.ShooterMotor.GEAR_RATIO);
    corrected.setMaxRPM(RobotConfig.ShooterMotor.MAX_RPM);
    corrected.setOrientation(Rotation.CCW);
    return corrected;
  }
}
