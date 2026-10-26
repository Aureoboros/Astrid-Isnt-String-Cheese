package org.firstinspires.ftc.teamcode.telemetry;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap;
import org.firstinspires.ftc.teamcode.navigation.Waypoint;

/** Field-centric pose and tile matrix; does not initialize or reset any localization hardware. */
public final class FieldLocationTelemetry {
  private FieldLocationTelemetry() {}

  public static void add(Telemetry telemetry, Pose pose, GoBildaPinpointDriver pinpoint,
      String targetName, Waypoint target, boolean showMatrix) {
    String currentTile = BiobuzzFieldMap.tileAt(pose.getX(), pose.getY());
    telemetry.addData("Field Frame", "Audience-left origin; +X to F, +Y to row 6; inches");
    telemetry.addData("Pose Source", "Pinpoint: selected start + 2 pods + onboard IMU");
    telemetry.addData("Field Pose", "X %.2f, Y %.2f in; heading %.1f deg",
        pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    telemetry.addData("Current Tile", currentTile);
    telemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus());
    telemetry.addData("Forward / Strafe Raw Ticks", "%d / %d",
        pinpoint.getEncoderX(), pinpoint.getEncoderY());
    if (target != null) {
      telemetry.addData("Target Location", targetName);
      telemetry.addData("Target Pose", "X %.2f, Y %.2f in; heading %.1f deg",
          target.getX(), target.getY(), target.getHeadingDegrees());
      telemetry.addData("Position Error", "%.2f in",
          Math.hypot(target.getX() - pose.getX(), target.getY() - pose.getY()));
    }
    if (showMatrix) {
      telemetry.addLine("Tile matrix (* = robot center; NOT obstacle/free-space map)");
      for (int row = 6; row >= 1; row--) {
        StringBuilder line = new StringBuilder();
        for (char column = 'A'; column <= 'F'; column++) {
          String tile = "" + column + row;
          line.append(tile.equals(currentTile) ? "*" : " ").append(tile).append(" ");
          if (column == 'C') {
            line.append("| ");
          }
        }
        telemetry.addData("Row " + row, line.toString());
      }
    }
    OdometryTelemetry.add(telemetry);
  }
}
