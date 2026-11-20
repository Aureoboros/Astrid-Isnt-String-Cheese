package org.firstinspires.ftc.teamcode.navigation;

import static org.junit.Assert.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.firstinspires.ftc.teamcode.control.PollenBatchInventory;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.Alliance;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.StartLocation;
import org.firstinspires.ftc.teamcode.navigation.HiveAutoRoute.Stage;
import org.junit.Test;

public final class PreloadedHiveAutoRouteTest {
  @Test public void threeBatchesRequireTipsOnlyAfterPreloadsAndFinalOppositeSideRefill() {
    for (Alliance a : Alliance.values()) {
      PreloadedHiveAutoRoute r = new PreloadedHiveAutoRoute(a);
      assertEquals(Arrays.asList(Stage.PRELOAD_SHOT, Stage.FLOWER_1_PICKUP,
          Stage.FIRST_REFILL_SHOT, Stage.FLOWER_2_PICKUP, Stage.SECOND_REFILL_SHOT), r.stages());
      assertTrue(r.requiresTip(Stage.PRELOAD_SHOT));
      assertFalse(r.requiresTip(Stage.FIRST_REFILL_SHOT));
      assertTrue(r.requiresTip(Stage.SECOND_REFILL_SHOT));
      assertEquals(r.firstTagMin(), r.tagMin(Stage.PRELOAD_SHOT));
      assertEquals(r.secondTagMin(), r.tagMin(Stage.FIRST_REFILL_SHOT));
      assertEquals(r.secondTagMin(), r.tagMin(Stage.SECOND_REFILL_SHOT));
      Waypoint second = last(r.approach(Stage.FIRST_REFILL_SHOT));
      Waypoint third = last(r.approach(Stage.SECOND_REFILL_SHOT));
      assertEquals(second.getX(), third.getX(), 0);
      assertEquals(second.getY(), third.getY(), 0);
      assertEquals(second.getHeadingDegrees(), third.getHeadingDegrees(), 0);
    }
  }

  @Test public void inventoryNeverExceedsFourAndOnlyTwoFlowersSupplyTwelveTotalShots() {
    PollenBatchInventory inventory = new PollenBatchInventory();
    int pickups = 0;
    for (Stage stage : new PreloadedHiveAutoRoute(Alliance.RED).stages()) {
      if (stage.isPickup()) {
        assertTrue(inventory.canPickup()); inventory.recordPickup(); pickups++;
      } else {
        assertEquals(4, inventory.getRemaining()); inventory.recordFed(4);
      }
      assertTrue(inventory.getRemaining() >= 0 && inventory.getRemaining() <= 4);
    }
    assertEquals(2, pickups);
    assertEquals(12, inventory.getTotalFed());
    assertEquals(0, inventory.getRemaining());
  }

  @Test public void startsAndEveryBlueWaypointAreRotatedRed() {
    PreloadedHiveAutoRoute red = new PreloadedHiveAutoRoute(Alliance.RED);
    PreloadedHiveAutoRoute blue = new PreloadedHiveAutoRoute(Alliance.BLUE);
    assertEquals(StartLocation.RED_B1_AUDIENCE, red.start());
    assertEquals(StartLocation.BLUE_E6_FAR, blue.start());
    for (Stage stage : red.stages()) {
      List<Waypoint> rp = red.approach(stage), bp = blue.approach(stage);
      assertEquals(rp.size(), bp.size());
      for (int i = 0; i < rp.size(); i++) { compare(rp.get(i), bp.get(i)); }
      if (stage.isPickup()) { compare(red.pickupTarget(stage), blue.pickupTarget(stage)); }
    }
    List<Waypoint> rp = red.parkFrom(last(red.approach(Stage.SECOND_REFILL_SHOT)));
    List<Waypoint> bp = blue.parkFrom(last(blue.approach(Stage.SECOND_REFILL_SHOT)));
    for (int i = 0; i < rp.size(); i++) { compare(rp.get(i), bp.get(i)); }
  }

  @Test public void pickupCreepsAreEightInchesForwardAndFlowerOneToSideTwoUsesOuterCorridor() {
    for (Alliance a : Alliance.values()) {
      PreloadedHiveAutoRoute r = new PreloadedHiveAutoRoute(a);
      for (Stage stage : r.stages()) {
        if (!stage.isPickup()) { continue; }
        Waypoint from = last(r.approach(stage)), to = r.pickupTarget(stage);
        double h = Math.toRadians(to.getHeadingDegrees());
        assertEquals(8, (to.getX() - from.getX()) * Math.cos(h)
            + (to.getY() - from.getY()) * Math.sin(h), 1e-6);
        assertEquals(from.getHeadingDegrees(), to.getHeadingDegrees(), 0);
      }
      List<Waypoint> transit = r.approach(Stage.FIRST_REFILL_SHOT);
      assertEquals(a == Alliance.RED ? -44 : 44, transit.get(0).getX(), 0);
      assertEquals(transit.get(0).getX(), transit.get(1).getX(), 0);
    }
  }

  @Test public void completeRouteFitsFieldOwnHalfAndAvoidsNominalHiveFrame() {
    BiobuzzFieldMap field = new BiobuzzFieldMap(18, 18);
    double radius = Math.hypot(9, 9);
    for (Alliance a : Alliance.values()) {
      PreloadedHiveAutoRoute r = new PreloadedHiveAutoRoute(a);
      Waypoint previous = field.startPose(r.start());
      List<Waypoint> points = new ArrayList<>();
      for (Stage stage : r.stages()) {
        points.addAll(r.approach(stage));
        if (stage.isPickup()) { points.add(r.pickupTarget(stage)); }
      }
      points.addAll(r.parkFrom(last(points)));
      for (Waypoint next : points) {
        for (int sample = 0; sample <= 100; sample++) {
          double t = sample / 100.0;
          double x = previous.getX() + t * (next.getX() - previous.getX());
          double y = previous.getY() + t * (next.getY() - previous.getY());
          double h = previous.getHeadingDegrees() + t * (next.getHeadingDegrees() - previous.getHeadingDegrees());
          assertTrue(field.footprintInsideField(new Waypoint(x, y, h)));
          double extent = 9 * (Math.abs(Math.cos(Math.toRadians(h))) + Math.abs(Math.sin(Math.toRadians(h))));
          assertTrue(a == Alliance.RED ? x + extent <= 0 : x - extent >= 0);
          assertFalse("Nominal hive frame collision", x > -49.46 / 2 - radius
              && x < 49.46 / 2 + radius && y > -38.95 / 2 - radius && y < 38.95 / 2 + radius);
        }
        previous = next;
      }
      Waypoint park = last(points);
      assertTrue(a == Alliance.RED ? park.getX() - 9 < -61 : park.getX() + 9 > 61);
    }
  }

  @Test(expected = IllegalArgumentException.class) public void gardenPickupIsNotPartOfThisPlan() {
    new PreloadedHiveAutoRoute(Alliance.RED).pickupTarget(Stage.GARDEN_PICKUP);
  }

  private static Waypoint last(List<Waypoint> points) { return points.get(points.size() - 1); }
  private static void compare(Waypoint red, Waypoint blue) {
    assertEquals(-red.getX(), blue.getX(), 1e-6);
    assertEquals(-red.getY(), blue.getY(), 1e-6);
    assertEquals((red.getHeadingDegrees() + 180) % 360, blue.getHeadingDegrees(), 1e-6);
  }
}
