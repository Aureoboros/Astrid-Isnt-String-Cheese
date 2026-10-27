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
}
