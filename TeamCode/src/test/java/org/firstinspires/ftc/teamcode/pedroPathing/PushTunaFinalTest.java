package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import org.junit.Test;
import static org.junit.Assert.*;

public final class PushTunaFinalTest {
  private static final class FakeDrive implements PushTunaFinal.DriveIO {
    Pose pose = new Pose(0, 0, 0), target;
    boolean ready = true;
    double forward, left, rotation;
    int holds, stops;
    public void updatePose() {}
    public Pose getPose() { return pose; }
    public boolean sensorReady() { return ready; }
    public void drive(double f, double l, double r) { forward = f; left = l; rotation = r; }
    public void update() {}
    public void hold(Pose p) { target = p; holds++; }
    public void returnToDriver() { stops++; forward = left = rotation = 0; }
  }
  private final FakeDrive drive = new FakeDrive();
  private final PushTunaFinal control = new PushTunaFinal(drive);

  private void arm() {
    control.update(0, 0, 0, 0);
    control.update(0, 0, 0, 350);
    assertEquals(PushTunaFinal.State.ARMED, control.getState());
  }
  private void bump() {
    arm();
    drive.pose = new Pose(2, 0, 0);
    control.update(0, 0, 0, 400);
    assertTrue(control.isRecovering());
  }
  @Test public void pedroInputsPassThroughWithoutSecondSignFlip() {
    control.update(.7, -.4, -.3, 0);
    assertEquals(.7, drive.forward, 1e-9);
    assertEquals(-.4, drive.left, 1e-9);
    assertEquals(-.3, drive.rotation, 1e-9);
  }
  @Test public void strafingAddsCcwCorrectionWithoutChangingTranslationOrArmingRecovery() {
    drive.pose=new Pose(0,0,Math.toRadians(-5)); control.update(.2,.6,0,0);
    drive.pose=new Pose(0,0,Math.toRadians(-19)); control.update(.2,.6,0,100);
    assertEquals(.2,drive.forward,0); assertEquals(.6,drive.left,0); assertTrue(drive.rotation>0);
    assertTrue(control.isStrafeHeadingHolding()); assertEquals(PushTunaFinal.State.DRIVING,control.getState());
    assertEquals(0,drive.holds);
    control.update(.2,.6,-.01,150); assertEquals(-.01,drive.rotation,0);
    assertFalse(control.isStrafeHeadingHolding());
  }
  @Test public void stoppingStrafeLeavesExistingPushRecoveryAvailable() {
    control.update(0,.6,0,0); assertTrue(control.isStrafeHeadingHolding());
    control.update(0,0,0,100); assertFalse(control.isStrafeHeadingHolding()); assertEquals(0,drive.rotation,0);
    control.update(0,0,0,450); drive.pose=new Pose(2,0,0); control.update(0,0,0,500);
    assertTrue(control.isRecovering());
  }
  @Test public void poseResetAndSensorFaultCannotReuseStrafeHeading() {
    control.update(0,.6,0,0); control.reset(); drive.pose=new Pose(10,5,Math.toRadians(90));
    control.update(0,.6,0,100); assertEquals(90,control.getStrafeHeadingTargetDegrees(),1e-9);
    assertEquals(0,drive.rotation,0);
    drive.ready=false; control.update(0,.6,0,150); assertFalse(control.isStrafeHeadingHolding());
    assertEquals(0,drive.rotation,0);
  }
  @Test public void tinyMappedTriggerCancelsPushRecoveryImmediately() {
    bump(); control.update(0,0,.01,450);
    assertFalse(control.isRecovering()); assertEquals(.01,drive.rotation,0);
  }
  @Test public void rotationMustSettleBeforeArming() {
    control.update(0, 0, 0, 0);
    drive.pose = new Pose(0, 0, Math.toRadians(2));
    control.update(0, 0, 0, 350);
    assertEquals(PushTunaFinal.State.WAITING_FOR_STILLNESS, control.getState());
    control.update(0, 0, 0, 700);
    assertEquals(PushTunaFinal.State.ARMED, control.getState());
  }
  @Test public void translationResetsStillnessTimer() {
    control.update(0, 0, 0, 0);
    drive.pose = drive.pose.withX(2);
    control.update(0, 0, 0, 350);
    assertEquals(PushTunaFinal.State.WAITING_FOR_STILLNESS, control.getState());
  }
  @Test public void bumpHoldsOriginalPoseAndCompletionReturnsToTeleop() {
    bump();
    assertEquals(0, drive.target.getX(), 1e-9);
    assertEquals(1, drive.holds);
    drive.pose = new Pose(0, 0, 0);
    control.update(0, 0, 0, 450);
    assertEquals(PushTunaFinal.State.ARMED, control.getState());
    assertEquals(1, drive.stops);
  }
  @Test public void driverOverridesRecoveryInSameIteration() {
    bump();
    control.update(.6, .2, -.1, 450);
    assertEquals(PushTunaFinal.State.DRIVING, control.getState());
    assertEquals(.6, drive.forward, 1e-9);
    assertEquals(.2, drive.left, 1e-9);
  }
  @Test public void sensorFaultStopsRecoveryAndDropsStaleTarget() {
    bump();
    drive.ready = false;
    control.update(0, 0, 0, 450);
    assertEquals(PushTunaFinal.State.DRIVING, control.getState());
    assertEquals(1, drive.stops);
    drive.ready = true;
    control.update(0, 0, 0, 500);
    assertEquals(PushTunaFinal.State.WAITING_FOR_STILLNESS, control.getState());
  }
  @Test public void invalidPoseOrInputCannotCommandDrive() {
    drive.pose = new Pose(Double.NaN, 0, 0);
    control.update(1, 0, 0, 0);
    assertEquals(0, drive.forward, 0);
    drive.pose = new Pose(0, 0, 0);
    control.update(1, Double.NaN, 0, 10);
    assertEquals(2, drive.stops);
  }
  @Test public void recoveryTimeoutReturnsControlWithoutRearmingStalePose() {
    bump();
    control.update(0, 0, 0, 2400);
    assertEquals(PushTunaFinal.State.DRIVING, control.getState());
    assertEquals(1, drive.stops);
  }

  @Test public void resetAfterAutomaticMoveDropsOldHoldAndRearmsAtNewLocation() {
    bump();
    drive.pose = new Pose(10, 0, 0);
    control.reset();
    assertEquals(PushTunaFinal.State.DRIVING, control.getState());
    assertFalse(control.isRecovering());
    control.update(0, 0, 0, 500);
    control.update(0, 0, 0, 850);
    drive.pose = new Pose(12, 0, 0);
    control.update(0, 0, 0, 900);
    assertTrue(control.isRecovering());
    assertEquals(10, drive.target.getX(), 1e-9);
  }
}
