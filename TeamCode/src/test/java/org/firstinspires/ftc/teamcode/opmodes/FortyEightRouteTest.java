package org.firstinspires.ftc.teamcode.opmodes;

import static org.junit.Assert.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.List;
import org.firstinspires.ftc.teamcode.control.PollenBatchInventory;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.Alliance;
import org.firstinspires.ftc.teamcode.navigation.HiveAutoRoute;
import org.firstinspires.ftc.teamcode.navigation.Waypoint;
import org.junit.Test;

public final class FortyEightRouteTest {
  @Test public void retainsDriverStationNameAndCorrectPublicClassName() {
    assertEquals("FortyEight", FortyEight.class.getSimpleName());
    assertEquals("8", FortyEight.class.getAnnotation(Autonomous.class).name());
  }

  @Test public void stagesRetainTwoFlowersAndThreeBatchesWithFinalTipOnly() {
    assertArrayEquals(new FortyEight.Stage[] {
        FortyEight.Stage.SIDE_1_PRELOAD_SHOT, FortyEight.Stage.FLOWER_1_PICKUP,
        FortyEight.Stage.SIDE_2_FIRST_SHOT, FortyEight.Stage.FLOWER_2_PICKUP,
        FortyEight.Stage.SIDE_2_SECOND_SHOT}, FortyEight.Stage.values());
    PollenBatchInventory inventory = new PollenBatchInventory();
    int pickups = 0, tipRequests = 0;
    for (FortyEight.Stage stage : FortyEight.Stage.values()) {
      if (stage.pickup) {
        pickups++;
        assertTrue(inventory.canPickup());
        inventory.recordPickup();
      } else {
        assertEquals(4, inventory.getRemaining());
        inventory.recordFed(4);
      }
      if (stage.requireTip) { tipRequests++; assertEquals(FortyEight.Stage.SIDE_2_SECOND_SHOT, stage); }
      assertEquals(stage == FortyEight.Stage.SIDE_1_PRELOAD_SHOT, stage.side1);
    }
    assertEquals(2, pickups);
    assertEquals(1, tipRequests);
    assertEquals(12, inventory.getTotalFed());
  }

  @Test public void blueWaypointsRotateRedAndEndpointsFitField() {
    BiobuzzFieldMap field = new BiobuzzFieldMap(18, 18);
    HiveAutoRoute redRoute = new HiveAutoRoute(Alliance.RED), blueRoute = new HiveAutoRoute(Alliance.BLUE);
    for (FortyEight.Stage stage : FortyEight.Stage.values()) {
      List<Waypoint> red = FortyEight.approach(stage, Alliance.RED, redRoute);
      List<Waypoint> blue = FortyEight.approach(stage, Alliance.BLUE, blueRoute);
      assertEquals(red.size(), blue.size());
      for (int i = 0; i < red.size(); i++) {
        rotated(red.get(i), blue.get(i));
        assertTrue(field.footprintInsideField(red.get(i)));
        assertTrue(field.footprintInsideField(blue.get(i)));
      }
      if (stage.pickup) {
        Waypoint r = FortyEight.pickupTarget(stage, Alliance.RED);
        Waypoint b = FortyEight.pickupTarget(stage, Alliance.BLUE);
        rotated(r, b);
        assertTrue(field.footprintInsideField(r));
        assertTrue(field.footprintInsideField(b));
      }
    }
  }

  @Test public void sideChangeUsesOutsideCorridorAndRetainsShootingPoses() {
    HiveAutoRoute route = new HiveAutoRoute(Alliance.RED);
    List<Waypoint> crossing = FortyEight.approach(FortyEight.Stage.SIDE_2_FIRST_SHOT, Alliance.RED, route);
    assertEquals(-44, crossing.get(0).getX(), 0);
    assertEquals(-44, crossing.get(1).getX(), 0);
    Waypoint first = last(FortyEight.approach(FortyEight.Stage.SIDE_1_PRELOAD_SHOT, Alliance.RED, route));
    Waypoint second = last(crossing);
    assertEquals(-12.75, first.getX(), 0); assertEquals(-40, first.getY(), 0);
    assertEquals(90, first.getHeadingDegrees(), 0);
    assertEquals(-12.75, second.getX(), 0); assertEquals(40, second.getY(), 0);
    assertEquals(270, second.getHeadingDegrees(), 0);
  }

  @Test(expected = IllegalArgumentException.class) public void shootingStageHasNoPickupTarget() {
    FortyEight.pickupTarget(FortyEight.Stage.SIDE_1_PRELOAD_SHOT, Alliance.RED);
  }

  private static Waypoint last(List<Waypoint> points) { return points.get(points.size() - 1); }
  private static void rotated(Waypoint red, Waypoint blue) {
    assertEquals(-red.getX(), blue.getX(), 1e-9);
    assertEquals(-red.getY(), blue.getY(), 1e-9);
    assertEquals((red.getHeadingDegrees() + 180) % 360, blue.getHeadingDegrees(), 1e-9);
  }
}
