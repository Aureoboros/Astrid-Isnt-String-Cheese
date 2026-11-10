package org.firstinspires.ftc.teamcode.localization;

import static org.junit.Assert.*;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.junit.Test;

public final class CameraMountPoseTest {
  @Test public void changesMountFieldsOnlyAndPreservesPipelineAndOriginal() {
    JsonObject original = new JsonParser().parse("{\"pipeline_type\":\"pipe_fiducial\","
        + "\"rsf\":0,\"rss\":0,\"rsu\":0,\"rsroll\":0,\"rspitch\":0,\"rsyaw\":0,"
        + "\"exposure\":1000,\"fiducial_size\":82.55,\"fiducial_idfilters\":\"\"}").getAsJsonObject();
    CameraMountPose pose = new CameraMountPose(0.0254, -0.09525, 0.4318, 0, 45, 30);
    JsonObject updated = pose.updatePipeline(original);
    assertTrue(pose.matchesPipeline(updated));
    assertEquals(1000, updated.get("exposure").getAsInt());
    assertEquals(82.55, updated.get("fiducial_size").getAsDouble(), 1e-9);
    assertEquals("", updated.get("fiducial_idfilters").getAsString());
    assertEquals(0, original.get("rsyaw").getAsInt());
  }
  @Test public void parsesResultAndDetectsYawMismatch() {
    CameraMountPose pose = CameraMountPose.fromFrame(new JsonParser().parse(
        "{\"t6c_rs\":[0.0254,-0.09525,0.4318,0,45,30]}").getAsJsonObject());
    assertTrue(new CameraMountPose(0.0254, -0.09525, 0.4318, 0, 45, 30).matchesFrame(pose));
    assertFalse(new CameraMountPose(0.0254, -0.09525, 0.4318, 0, 45, 0).matchesFrame(pose));
    assertNull(CameraMountPose.fromFrame(new JsonObject()));
  }
  @Test public void resultYawSignDoesNotChangePositionOrPipelineYaw() {
    CameraMountPose pose = new CameraMountPose(0.0254, -0.09525, 0.4318, 0, 45, 30);
    assertEquals(-30, pose.forResultYawSign(-1).getYawDegrees(), 1e-9);
    assertEquals(30, pose.getYawDegrees(), 1e-9);
    assertEquals(pose.getUpMeters(), pose.forResultYawSign(-1).getUpMeters(), 1e-9);
  }
  @Test(expected = IllegalArgumentException.class) public void rejectsUnknownMountSchema() {
    new CameraMountPose(0, 0, 0, 0, 0, 0).updatePipeline(
        new JsonParser().parse("{\"pipeline_type\":\"pipe_fiducial\"}").getAsJsonObject());
  }
}
