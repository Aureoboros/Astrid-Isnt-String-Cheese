package org.firstinspires.ftc.teamcode.navigation;

import static org.junit.Assert.*;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzAutoRoutes.Plan;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.StartLocation;
import org.junit.Test;

public final class BiobuzzAutoRoutesTest {
  @Test
  public void poseOnlyNeverContainsMovementTargets() {
    for (StartLocation start : StartLocation.values()) {
      assertTrue(BiobuzzAutoRoutes.destinations(start, Plan.POSE_ONLY).isEmpty());
    }
  }

  @Test
  public void wallExitRoutesResolveOnlyTheSelectedStartsTarget() {
    BiobuzzFieldMap field = new BiobuzzFieldMap(18, 18);
    for (StartLocation start : StartLocation.values()) {
      assertEquals(1, BiobuzzAutoRoutes.destinations(start, Plan.LEAVE_WALL_12_INCHES).size());
      String name = BiobuzzAutoRoutes.destinations(start, Plan.LEAVE_WALL_12_INCHES).get(0);
      assertEquals(start.clearWallLocationName(), name);
      assertTrue(field.footprintInsideField(field.getLocation(name)));
    }
  }

  @Test
  public void hiveCheckRouteVisitsCheckThenParksForEveryStart() {
    for (StartLocation start : StartLocation.values()) {
      java.util.List<String> route = BiobuzzAutoRoutes.destinations(start, Plan.HIVE_CHECK_AND_PARK);
      String side = start.getAlliance().name();
      assertEquals(start.clearWallLocationName(), route.get(0));
      assertEquals(1, route.stream().filter(BiobuzzAutoRoutes::isHiveCheck).count());
      assertEquals(side + "_HIVE_CHECK", route.get(2));
      assertEquals(side + "_PARK", route.get(route.size() - 1));
      assertTrue(BiobuzzAutoRoutes.usesLimelight(Plan.HIVE_CHECK_AND_PARK));
      assertFalse(BiobuzzAutoRoutes.usesLimelight(Plan.LEAVE_WALL_12_INCHES));
    }
  }

  @Test
  public void hiveCheckPoseMatchesHiveAutoFirstShootingPose() {
    BiobuzzFieldMap field = new BiobuzzFieldMap(18, 18);
    for (BiobuzzFieldMap.Alliance alliance : BiobuzzFieldMap.Alliance.values()) {
      java.util.List<Waypoint> approach = new HiveAutoRoute(alliance).firstApproach();
      Waypoint shooting = approach.get(approach.size() - 1);
      Waypoint check = field.getLocation(alliance.name() + "_HIVE_CHECK");
      assertEquals(shooting.getX(), check.getX(), 1e-6);
      assertEquals(shooting.getY(), check.getY(), 1e-6);
      assertEquals(shooting.getHeadingDegrees(), check.getHeadingDegrees(), 1e-6);
    }
  }

  /** Samples every straight leg (with shortest-way heading turn) from each of the 8 starts. */
  @Test
  public void hiveCheckRouteStaysInFieldOnOwnHalfAndClearOfHiveFrame() {
    BiobuzzFieldMap field = new BiobuzzFieldMap(18, 18);
    double radius = Math.hypot(9, 9);  // circumscribed 18 x 18 footprint, any heading
    for (StartLocation start : StartLocation.values()) {
      Waypoint previous = field.startPose(start);
      for (String name : BiobuzzAutoRoutes.destinations(start, Plan.HIVE_CHECK_AND_PARK)) {
        Waypoint next = field.getLocation(name);
        double turn = Math.toDegrees(Math.atan2(
            Math.sin(Math.toRadians(next.getHeadingDegrees() - previous.getHeadingDegrees())),
            Math.cos(Math.toRadians(next.getHeadingDegrees() - previous.getHeadingDegrees()))));
        for (int sample = 0; sample <= 200; sample++) {
          double t = sample / 200.0;
          double x = previous.getX() + t * (next.getX() - previous.getX());
          double y = previous.getY() + t * (next.getY() - previous.getY());
          double h = previous.getHeadingDegrees() + t * turn;
          String where = start + " -> " + name + " t=" + t;
          assertTrue(where, field.footprintInsideField(new Waypoint(x, y, h)));
          double extentX = 9 * (Math.abs(Math.cos(Math.toRadians(h))) + Math.abs(Math.sin(Math.toRadians(h))));
          assertTrue(where, start.getAlliance() == BiobuzzFieldMap.Alliance.RED
              ? x + extentX <= 1e-6 : x - extentX >= -1e-6);
          // Nominal 49.46 x 38.95 in hive frame centered on the field.
          assertFalse(where, Math.abs(x) < 49.46 / 2 + radius && Math.abs(y) < 38.95 / 2 + radius);
        }
        previous = next;
      }
    }
  }
}
