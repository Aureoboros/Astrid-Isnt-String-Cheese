package org.firstinspires.ftc.teamcode.localization;

import static org.junit.Assert.*;
import org.junit.Test;

public final class PanPoseGateTest {
  private final CameraMountPose pose = new CameraMountPose(0.0254, -0.09525, 0.4318, 0, 45, 20);
  @Test public void neverPublishesBeforeMountUpdateAcknowledgement() {
    PanPoseGate gate = new PanPoseGate(0.35);
    gate.beginMove(pose, 100);
    assertFalse(gate.ready(pose, 101, 1000));
  }
  @Test public void requiresSettlingAndDistinctPostSettleFrame() {
    PanPoseGate gate = new PanPoseGate(0.35);
    gate.beginMove(pose, 100);
    gate.acknowledgeUpdate(0);
    assertFalse(gate.ready(pose, 101, 0.1));
    assertFalse(gate.ready(pose, 102, 0.4));
    assertFalse(gate.ready(pose, 102, 0.5));
    assertTrue(gate.ready(pose, 103, 0.5));
  }
  @Test public void wrongYawMissingMetadataAndOldFramesStayBlocked() {
    PanPoseGate gate = new PanPoseGate(0);
    gate.beginMove(pose, 100);
    gate.acknowledgeUpdate(0);
    assertFalse(gate.ready(null, 101, 1));
    assertFalse(gate.ready(new CameraMountPose(0.0254, -0.09525, 0.4318, 0, 45, 0), 101, 1));
    assertFalse(gate.ready(pose, 100, 1));
    assertFalse(gate.ready(pose, Double.NaN, 1));
  }
  @Test public void newMovementImmediatelyClosesPreviouslyReadyGate() {
    PanPoseGate gate = new PanPoseGate(0);
    gate.beginMove(pose, 100); gate.acknowledgeUpdate(0);
    assertFalse(gate.ready(pose, 101, 1));
    assertTrue(gate.ready(pose, 102, 1));
    gate.beginMove(pose, 102);
    assertFalse(gate.ready(pose, 103, 10));
  }
}
