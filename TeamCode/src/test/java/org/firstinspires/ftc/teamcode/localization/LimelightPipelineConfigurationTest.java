package org.firstinspires.ftc.teamcode.localization;

import static org.junit.Assert.assertEquals;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.junit.Test;

public final class LimelightPipelineConfigurationTest {
  @Test
  public void clearsRestrictedIdsWithoutChangingOtherSettingsOrOriginal() {
    JsonObject original = new JsonParser().parse(
        "{\"pipeline_type\":\"pipe_fiducial\",\"fiducial_idfilters\":\"20,24\","
            + "\"desc\":\"Tag_24\",\"fiducial_size\":165.1,\"exposure\":1000}")
        .getAsJsonObject();
    JsonObject updated = LimelightPipelineConfiguration.allowAllIds(original);
    assertEquals("", updated.get("fiducial_idfilters").getAsString());
    assertEquals("20,24", original.get("fiducial_idfilters").getAsString());
    JsonObject expected = new JsonParser().parse(original.toString()).getAsJsonObject();
    expected.addProperty("fiducial_idfilters", "");
    assertEquals(expected, updated);
  }

  @Test
  public void alreadyUnrestrictedPipelineIsUnchanged() {
    JsonObject original = new JsonParser().parse(
        "{\"pipeline_type\":\"pipe_fiducial\",\"fiducial_idfilters\":\"\"}")
        .getAsJsonObject();
    assertEquals(original, LimelightPipelineConfiguration.allowAllIds(original));
  }

  @Test(expected = IllegalArgumentException.class)
  public void rejectsWrongPipelineType() {
    LimelightPipelineConfiguration.allowAllIds(new JsonParser().parse(
        "{\"pipeline_type\":\"pipe_color\",\"fiducial_idfilters\":\"24\"}")
        .getAsJsonObject());
  }

  @Test(expected = IllegalArgumentException.class)
  public void rejectsUnknownFilterSchema() {
    LimelightPipelineConfiguration.allowAllIds(new JsonParser().parse(
        "{\"pipeline_type\":\"pipe_fiducial\"}").getAsJsonObject());
  }
}
