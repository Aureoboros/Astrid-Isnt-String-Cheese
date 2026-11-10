package org.firstinspires.ftc.teamcode.localization;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;

/** Use during INIT or on a background worker, never on the drive loop. */
public final class LimelightPipelineConfiguration {
  private LimelightPipelineConfiguration() {}

  /** Returns a copy, changing only tag size and, optionally, the accepted AprilTag ID list. */
  static JsonObject configure(JsonObject original, boolean allowAllIds, double tagSizeMm) {
    if (!Double.isFinite(tagSizeMm) || tagSizeMm <= 0) {
      throw new IllegalArgumentException("Tag size must be finite and positive (mm)");
    }
    if (!original.has("pipeline_type")
        || !"pipe_fiducial".equals(original.get("pipeline_type").getAsString())
        || !original.has("fiducial_idfilters")
        || !original.has("fiducial_size")) {
      throw new IllegalArgumentException("Expected a fiducial pipeline with ID filter and tag size");
    }
    JsonObject updated = new JsonParser().parse(original.toString()).getAsJsonObject();
    if (allowAllIds) {
      updated.addProperty("fiducial_idfilters", "");
    }
    updated.addProperty("fiducial_size", tagSizeMm);
    return updated;
  }

  /** Updates the active pipeline's size/filter without requesting a persistent flash write. */
  public static String apply(
      String host, int pipelineIndex, boolean allowAllIds, double tagSizeMm) {
    String base = "http://" + host + ":5807";
    try {
      JsonObject status = read(base + "/status");
      if (!status.has("pipelineIndex")
          || status.get("pipelineIndex").getAsInt() != pipelineIndex) {
        return "FAILED: configuration IP/active pipeline mismatch";
      }
      JsonObject original = read(base + "/pipeline-atindex?index=" + pipelineIndex);
      JsonObject updated = configure(original, allowAllIds, tagSizeMm);
      String description = (allowAllIds ? "ALL IDs" : "Camera ID filter")
          + ", size " + tagSizeMm + " mm";
      if (matches(original, updated)) {
        return description + " (already configured)";
      }
      // No flush=1: do not persistently rewrite the team's pipeline settings.
      HttpURLConnection connection = open(base + "/update-pipeline");
      try {
        connection.setRequestMethod("POST");
        connection.setDoOutput(true);
        connection.setRequestProperty("Content-Type", "application/json");
        byte[] body = updated.toString().getBytes(StandardCharsets.UTF_8);
        connection.setFixedLengthStreamingMode(body.length);
        try (OutputStream output = connection.getOutputStream()) {
          output.write(body);
        }
        if (connection.getResponseCode() != HttpURLConnection.HTTP_OK) {
          throw new IOException("Update HTTP " + connection.getResponseCode());
        }
      } finally {
        connection.disconnect();
      }
      JsonObject verified = read(base + "/pipeline-atindex?index=" + pipelineIndex);
      return matches(verified, updated)
          ? description + " (verified)"
          : "FAILED: ID filter/tag size update not verified";
    } catch (IOException | RuntimeException failure) {
      return "FAILED: " + failure.getMessage();
    }
  }

  static boolean matches(JsonObject actual, JsonObject expected) {
    return actual.has("fiducial_idfilters") && actual.has("fiducial_size")
        && actual.get("fiducial_idfilters").equals(expected.get("fiducial_idfilters"))
        && Math.abs(actual.get("fiducial_size").getAsDouble()
            - expected.get("fiducial_size").getAsDouble()) < 0.000001;
  }

  /** Live read/modify/verify of mount fields only; callers must run this off the OpMode loop. */
  public static void applyMount(String host, int pipelineIndex, CameraMountPose mount)
      throws IOException {
    String base = "http://" + host + ":5807";
    JsonObject status = read(base + "/status");
    if (!status.has("pipelineIndex") || status.get("pipelineIndex").getAsInt() != pipelineIndex) {
      throw new IOException("Active pipeline mismatch");
    }
    JsonObject original = read(base + "/pipeline-atindex?index=" + pipelineIndex);
    JsonObject updated = mount.updatePipeline(original);
    if (!mount.matchesPipeline(original)) {
      if (Thread.currentThread().isInterrupted()) { throw new IOException("Pan stopped"); }
      HttpURLConnection connection = open(base + "/update-pipeline");
      try {
        connection.setRequestMethod("POST");
        connection.setDoOutput(true);
        connection.setRequestProperty("Content-Type", "application/json");
        byte[] body = updated.toString().getBytes(StandardCharsets.UTF_8);
        connection.setFixedLengthStreamingMode(body.length);
        try (OutputStream output = connection.getOutputStream()) { output.write(body); }
        if (connection.getResponseCode() != HttpURLConnection.HTTP_OK) {
          throw new IOException("Mount update HTTP " + connection.getResponseCode());
        }
      } finally { connection.disconnect(); }
    }
    if (Thread.currentThread().isInterrupted()) { throw new IOException("Pan stopped"); }
    if (!mount.matchesPipeline(read(base + "/pipeline-atindex?index=" + pipelineIndex))) {
      throw new IOException("Mount update not verified");
    }
  }

  private static JsonObject read(String address) throws IOException {
    if (Thread.currentThread().isInterrupted()) { throw new IOException("Request cancelled"); }
    HttpURLConnection connection = open(address);
    try {
      if (connection.getResponseCode() != HttpURLConnection.HTTP_OK) {
        throw new IOException("Read HTTP " + connection.getResponseCode());
      }
      try (InputStreamReader input =
          new InputStreamReader(connection.getInputStream(), StandardCharsets.UTF_8)) {
        return new JsonParser().parse(input).getAsJsonObject();
      }
    } finally {
      connection.disconnect();
    }
  }

  private static HttpURLConnection open(String address) throws IOException {
    HttpURLConnection connection = (HttpURLConnection) new URL(address).openConnection();
    connection.setConnectTimeout(1000);
    connection.setReadTimeout(1000);
    return connection;
  }
}
