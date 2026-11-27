package org.firstinspires.ftc.teamcode.localization;
import static org.junit.Assert.*;
import com.google.gson.JsonParser;
import org.junit.Test;
public final class LimelightFieldPoseTest {
  private PoseEstimate frame(String json) { return LimelightFieldPose.fromFrame(new JsonParser().parse(json).getAsJsonObject()); }
  @Test public void convertsMappedMetersToSignedFieldInchesAndKeepsYaw() {
    PoseEstimate p = LimelightFieldPose.fromMeters(1, .635, -.381, 0, 0, 0, -90);
    assertEquals(25, p.getX(), 1e-9); assertEquals(-15, p.getY(), 1e-9); assertEquals(-90, p.getHeadingDegrees(), 0);
  }
  @Test public void realMappedOriginIsAllowed() {
    assertNotNull(frame("{\"botpose_tagcount\":1,\"botpose\":[0,0,0,0,0,0]}"));
  }
  @Test public void unmappedDefaultZerosAreRejected() {
    assertNull(frame("{\"botpose_tagcount\":0,\"botpose\":[0,0,0,0,0,0]}"));
  }
  @Test public void missingAndMalformedBotposeAreRejectedInsteadOfSdkFallbackOrigin() {
    assertNull(frame("{\"botpose_tagcount\":1}"));
    assertNull(frame("{\"botpose_tagcount\":1,\"botpose\":[0,0,0]}"));
    assertNull(frame("{\"botpose_tagcount\":1,\"botpose\":null}"));
    assertNull(frame("{\"botpose_tagcount\":1,\"botpose\":[0,0,0,0,0,\"bad\"]}"));
    assertNull(frame("{\"botpose\":[0,0,0,0,0,0]}"));
  }
  @Test public void nonfiniteAndImplausibleRobotHeightOrTiltAreRejected() {
    assertNull(LimelightFieldPose.fromMeters(1, Double.NaN, 0, 0, 0, 0, 0));
    assertNull(LimelightFieldPose.fromMeters(1, 0, 0, 0, 0, 0, Double.POSITIVE_INFINITY));
    assertNull(LimelightFieldPose.fromMeters(1, 0, 0, .2, 0, 0, 0));
    assertNull(LimelightFieldPose.fromMeters(1, 0, 0, 0, 16, 0, 0));
    assertNull(LimelightFieldPose.fromMeters(1, 0, 0, 0, 0, -16, 0));
  }
}
