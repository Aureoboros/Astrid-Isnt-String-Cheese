package org.firstinspires.ftc.teamcode.navigation;

import static org.junit.Assert.*;
import java.util.List;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.Alliance;
import org.junit.Test;

public final class HiveAutoRouteTest {
  @Test public void blueIsRotatedRedAndTagsMatchManual() {
    HiveAutoRoute red = new HiveAutoRoute(Alliance.RED), blue = new HiveAutoRoute(Alliance.BLUE);
    compare(red.firstApproach(), blue.firstApproach());
    compare(red.oppositeApproach(), blue.oppositeApproach());
    assertEquals(34, red.firstTagMin()); assertEquals(30, red.secondTagMin());
    assertEquals(42, blue.firstTagMin()); assertEquals(38, blue.secondTagMin());
  }
  private void compare(List<Waypoint> red, List<Waypoint> blue) {
    for (int i = 0; i < red.size(); i++) {
      assertEquals(144 - red.get(i).getX(), blue.get(i).getX(), 1e-6);
      assertEquals(144 - red.get(i).getY(), blue.get(i).getY(), 1e-6);
      assertEquals((red.get(i).getHeadingDegrees() + 180) % 360, blue.get(i).getHeadingDegrees(), 1e-6);
    }
  }
  @Test public void endpointsFitMaximumStartingFootprintAndOwnHalf() {
    BiobuzzFieldMap field = new BiobuzzFieldMap(18, 18);
    for (Alliance a : Alliance.values()) {
      HiveAutoRoute r = new HiveAutoRoute(a);
      assertTrue(field.footprintInsideField(field.startPose(r.start())));
      for (List<Waypoint> leg : java.util.Arrays.asList(r.firstApproach(), r.oppositeApproach(),
          r.parkFrom(r.oppositeApproach().get(2)))) {
        for (Waypoint p : leg) {
          assertTrue(field.footprintInsideField(p));
          assertTrue(a == Alliance.RED ? p.getX() + 9 < 72 : p.getX() - 9 > 72);
        }
      }
    }
  }
  @Test public void crossingAndRecoveryStayOnOuterCorridor() {
    HiveAutoRoute r = new HiveAutoRoute(Alliance.RED);
    assertEquals(28, r.oppositeApproach().get(0).getX(), 0);
    assertEquals(28, r.oppositeApproach().get(1).getX(), 0);
    List<Waypoint> park = r.parkFrom(new Waypoint(59.25, 32, 90));
    assertEquals(28, park.get(0).getX(), 0);
    assertEquals(32, park.get(0).getY(), 0);
    assertEquals(12, park.get(2).getX(), 0);
    assertEquals(108, park.get(2).getY(), 0);
    // 18-inch robot partially overlaps nominal red loading zone x=0..11.
    assertTrue(park.get(2).getX() - 9 < 11);
  }

  @Test public void nominalSegmentsAvoidFrameWithConservativeEighteenInchFootprint() {
    BiobuzzFieldMap field = new BiobuzzFieldMap(18, 18);
    double radius = Math.hypot(9, 9); // Circumscribed footprint, independent of heading.
    for (Alliance a : Alliance.values()) {
      HiveAutoRoute r = new HiveAutoRoute(a);
      Waypoint previous = field.startPose(r.start());
      java.util.ArrayList<Waypoint> points = new java.util.ArrayList<>(r.firstApproach());
      points.addAll(r.oppositeApproach()); points.addAll(r.parkFrom(r.oppositeApproach().get(2)));
      for (Waypoint next : points) {
        for (int sample = 0; sample <= 100; sample++) {
          double t = sample / 100.0;
          double x = previous.getX() + t * (next.getX() - previous.getX());
          double y = previous.getY() + t * (next.getY() - previous.getY());
          assertFalse("Nominal frame intersection", x > 72 - 49.46 / 2 - radius
              && x < 72 + 49.46 / 2 + radius && y > 72 - 38.95 / 2 - radius
              && y < 72 + 38.95 / 2 + radius);
        }
        previous = next;
      }
    }
  }

  @Test public void newPlanHasFourShotBatchesTwoFlowersThenOwnGarden() {
    HiveAutoRoute.Stage[] stages = HiveAutoRoute.Stage.values();
    assertEquals(7, stages.length);
    assertEquals(HiveAutoRoute.Stage.PRELOAD_SHOT, stages[0]);
    assertEquals(HiveAutoRoute.Stage.FLOWER_1_PICKUP, stages[1]);
    assertEquals(HiveAutoRoute.Stage.FIRST_REFILL_SHOT, stages[2]);
    assertEquals(HiveAutoRoute.Stage.FLOWER_2_PICKUP, stages[3]);
    assertEquals(HiveAutoRoute.Stage.SECOND_INITIAL_SHOT, stages[4]);
    assertEquals(HiveAutoRoute.Stage.GARDEN_PICKUP, stages[5]);
    assertEquals(HiveAutoRoute.Stage.SECOND_REFILL_SHOT, stages[6]);
    assertFalse(stages[0].requiresTip()); assertTrue(stages[2].requiresTip());
    assertFalse(stages[4].requiresTip()); assertTrue(stages[6].requiresTip());
  }

  @Test public void pickupPosesMirrorAndEachCreepIsEightInchesForward() {
    HiveAutoRoute red = new HiveAutoRoute(Alliance.RED), blue = new HiveAutoRoute(Alliance.BLUE);
    for (HiveAutoRoute.Stage stage : HiveAutoRoute.Stage.values()) {
      compare(red.approach(stage), blue.approach(stage));
      if (!stage.isPickup()) { continue; }
      Waypoint p = red.pickupTarget(stage), b = blue.pickupTarget(stage);
      assertEquals(144 - p.getX(), b.getX(), 1e-6); assertEquals(144 - p.getY(), b.getY(), 1e-6);
      List<Waypoint> approach = red.approach(stage); Waypoint from = approach.get(approach.size() - 1);
      double h = Math.toRadians(p.getHeadingDegrees());
      assertEquals(8, (p.getX() - from.getX()) * Math.cos(h) + (p.getY() - from.getY()) * Math.sin(h), 1e-6);
      assertEquals(from.getHeadingDegrees(), p.getHeadingDegrees(), 1e-6);
    }
  }

  @Test public void fullPickupRouteStaysInFieldOwnHalfAndOutsideNominalFrame() {
    BiobuzzFieldMap field = new BiobuzzFieldMap(18, 18);
    double radius = Math.hypot(9, 9);
    for (Alliance a : Alliance.values()) {
      HiveAutoRoute r = new HiveAutoRoute(a);
      Waypoint previous = field.startPose(r.start());
      java.util.ArrayList<Waypoint> points = new java.util.ArrayList<>();
      for (HiveAutoRoute.Stage stage : HiveAutoRoute.Stage.values()) {
        points.addAll(r.approach(stage));
        if (stage.isPickup()) { points.add(r.pickupTarget(stage)); }
      }
      points.addAll(r.parkFrom(points.get(points.size() - 1)));
      for (Waypoint next : points) {
        for (int sample = 0; sample <= 100; sample++) {
          double t = sample / 100.0;
          double x = previous.getX() + t * (next.getX() - previous.getX());
          double y = previous.getY() + t * (next.getY() - previous.getY());
          double h = previous.getHeadingDegrees() + t * (next.getHeadingDegrees() - previous.getHeadingDegrees());
          assertTrue(field.footprintInsideField(new Waypoint(x, y, h)));
          double extentX = 9 * (Math.abs(Math.cos(Math.toRadians(h))) + Math.abs(Math.sin(Math.toRadians(h))));
          assertTrue(a == Alliance.RED ? x + extentX <= 72 : x - extentX >= 72);
          assertFalse(x > 72 - 49.46 / 2 - radius && x < 72 + 49.46 / 2 + radius
              && y > 72 - 38.95 / 2 - radius && y < 72 + 38.95 / 2 + radius);
        }
        previous = next;
      }
    }
  }
}
