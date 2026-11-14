package org.firstinspires.ftc.teamcode.config;

import static org.junit.Assert.*;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.junit.Test;

public final class GoBilda6000RpmMotorTest {
  @Test public void registeredProfileMatchesConfirmedMotorAndXmlTag() {
    MotorType profile = GoBilda6000RpmMotor.class.getAnnotation(MotorType.class);
    assertEquals(28, profile.ticksPerRev(), 0);
    assertEquals(1, profile.gearing(), 0);
    assertEquals(6000, profile.maxRPM(), 0);
    assertEquals("GoBilda6000RpmMotor",
        GoBilda6000RpmMotor.class.getAnnotation(DeviceProperties.class).xmlTag());
  }

  @Test public void runtimeCorrectionDoesNotMutateSharedDriveOrIntakeProfile() {
    MotorConfigurationType generic = new MotorConfigurationType();
    generic.setTicksPerRev(2786);
    generic.setGearing(99.5);
    generic.setMaxRPM(60);
    MotorConfigurationType corrected = GoBilda6000RpmMotor.correctedType(generic);
    assertNotSame(generic, corrected);
    assertEquals(28, corrected.getTicksPerRev(), 0);
    assertEquals(6000, corrected.getMaxRPM(), 0);
    assertEquals(1, corrected.getGearing(), 0);
    assertEquals(2786, generic.getTicksPerRev(), 0);
    assertEquals(60, generic.getMaxRPM(), 0);
    assertEquals(99.5, generic.getGearing(), 0);
  }
}
