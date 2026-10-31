package org.firstinspires.ftc.teamcode.localization;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;

/** Bounded INIT-only configuration requests; never run these in the movement loop. */
public final class LimelightPipelineConfiguration {
  private LimelightPipelineConfiguration() {}

  /** Returns a copy, preserving every setting except the accepted AprilTag ID list. */
  static JsonObject allowAllIds(JsonObject original) {
    if (!original.has("pipeline_type")
        || !"pipe_fiducial".equals(original.get("pipeline_type").getAsString())
        || !original.has("fiducial_idfilters")) {
      throw new IllegalArgumentException("Expected a fiducial pipeline with an ID filter");
    }
    JsonObject updated = new JsonParser().parse(original.toString()).getAsJsonObject();
    updated.addProperty("fiducial_idfilters", "");
    return updated;
  }

  /** Changes only the active pipeline's ID filter, without requesting a persistent flash write. */
  public static String applyAllowAllIds(String host, int pipelineIndex) {
    String base = "http://" + host + ":5807";
    try {
      JsonObject status = read(base + "/status");
      if (!status.has("pipelineIndex")
          || status.get("pipelineIndex").getAsInt() != pipelineIndex) {
        return "FAILED: configuration IP/active pipeline mismatch";
      }
      JsonObject original = read(base + "/pipeline-atindex?index=" + pipelineIndex);
      JsonObject updated = allowAllIds(original);
      if ("".equals(original.get("fiducial_idfilters").getAsString())) {
        return "ALL IDs already allowed";
      }
      // No flush=1: do not persistently rewrite the team's exposure, size, or other settings.
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
      return verified.has("fiducial_idfilters")
              && "".equals(verified.get("fiducial_idfilters").getAsString())
          ? "ALL IDs allowed (verified)"
          : "FAILED: ID filter update not verified";
    } catch (IOException | RuntimeException failure) {
      return "FAILED: " + failure.getMessage();
    }
  }

  private static JsonObject read(String address) throws IOException {
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
