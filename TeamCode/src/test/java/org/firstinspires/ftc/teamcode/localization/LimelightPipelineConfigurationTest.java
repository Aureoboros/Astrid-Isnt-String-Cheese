package org.firstinspires.ftc.teamcode.localization;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.junit.Test;

public final class LimelightPipelineConfigurationTest {
  @Test
  public void correctsSizeAndRestrictedIdsWithoutChangingOtherSettingsOrOriginal() {
    JsonObject original = new JsonParser().parse(
        "{\"pipeline_type\":\"pipe_fiducial\",\"fiducial_idfilters\":\"20,24\","
            + "\"desc\":\"Tag_24\",\"fiducial_size\":165.1,\"exposure\":1000}")
        .getAsJsonObject();
    JsonObject updated = LimelightPipelineConfiguration.configure(original, true, 82.55);
    assertEquals("", updated.get("fiducial_idfilters").getAsString());
    assertEquals("20,24", original.get("fiducial_idfilters").getAsString());
    assertEquals(165.1, original.get("fiducial_size").getAsDouble(), 1e-9);
    JsonObject expected = new JsonParser().parse(original.toString()).getAsJsonObject();
    expected.addProperty("fiducial_idfilters", "");
    expected.addProperty("fiducial_size", 82.55);
    assertEquals(expected, updated);
  }

  @Test
  public void alreadyCorrectPipelineIsUnchanged() {
    JsonObject original = new JsonParser().parse(
        "{\"pipeline_type\":\"pipe_fiducial\",\"fiducial_idfilters\":\"\",\"fiducial_size\":82.55}")
        .getAsJsonObject();
    assertEquals(original, LimelightPipelineConfiguration.configure(original, true, 82.55));
  }

  @Test(expected = IllegalArgumentException.class)
  public void rejectsWrongPipelineType() {
    LimelightPipelineConfiguration.configure(new JsonParser().parse(
        "{\"pipeline_type\":\"pipe_color\",\"fiducial_idfilters\":\"24\"}")
        .getAsJsonObject(), true, 82.55);
  }

  @Test(expected = IllegalArgumentException.class)
  public void rejectsUnknownFilterSchema() {
    LimelightPipelineConfiguration.configure(new JsonParser().parse(
        "{\"pipeline_type\":\"pipe_fiducial\"}").getAsJsonObject(), true, 82.55);
  }

  @Test
  public void correctsSizeEvenWhenIdsAlreadyAllowed() {
    JsonObject original = pipeline("", 165.1);
    JsonObject updated = LimelightPipelineConfiguration.configure(original, true, 82.55);
    assertEquals(82.55, updated.get("fiducial_size").getAsDouble(), 1e-9);
    assertFalse(LimelightPipelineConfiguration.matches(original, updated));
  }

  @Test
  public void preservesIdFilterWhenAllowAllDisabledButStillCorrectsSize() {
    JsonObject updated = LimelightPipelineConfiguration.configure(pipeline("20,24", 165.1), false, 82.55);
    assertEquals("20,24", updated.get("fiducial_idfilters").getAsString());
    assertEquals(82.55, updated.get("fiducial_size").getAsDouble(), 1e-9);
  }

  @Test
  public void rejectsInvalidSizes() {
    for (double size : new double[] {0, -1, Double.NaN, Double.POSITIVE_INFINITY}) {
      try {
        LimelightPipelineConfiguration.configure(pipeline("", 165.1), true, size);
        org.junit.Assert.fail("Invalid size accepted: " + size);
      } catch (IllegalArgumentException expected) {
        // Configuration must not be sent for invalid sizes.
      }
    }
  }

  @Test(expected = IllegalArgumentException.class)
  public void rejectsMissingSizeSchema() {
    LimelightPipelineConfiguration.configure(new JsonParser().parse(
        "{\"pipeline_type\":\"pipe_fiducial\",\"fiducial_idfilters\":\"\"}")
        .getAsJsonObject(), true, 82.55);
  }

  private static JsonObject pipeline(String ids, double size) {
    JsonObject result = new JsonObject();
    result.addProperty("pipeline_type", "pipe_fiducial");
    result.addProperty("fiducial_idfilters", ids);
    result.addProperty("fiducial_size", size);
    return result;
  }
}
