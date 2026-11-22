package org.firstinspires.ftc.teamcode.opmodes;

import static org.junit.Assert.*;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.Alliance;
import org.junit.Test;

public final class PreloadedHiveAutoStartTest {
  @Test public void redAndBlueBothRunInstalledMechanismsAndLegacyAutoStaysDrivingOnly() {
    for (BiobuzzHiveAuto auto : new BiobuzzHiveAuto[] {new BluePreloadedHiveAuto(), new RedPreloadedHiveAuto()}) {
      assertTrue(auto.mechanismsEnabled());
      assertTrue(auto.usesGateFeeder());
      assertTrue(auto.usesSingleShooter());
      assertTrue(auto.canRunRoute());
    }
    assertFalse(new BiobuzzHiveAuto().mechanismsEnabled());
    assertFalse(new BiobuzzHiveAuto().usesGateFeeder());
  }
  @Test public void practicePathsHaveNoPedroEndCorrectionTimerButOldAutoRemainsTimed() {
    for (BiobuzzHiveAuto auto : new BiobuzzHiveAuto[] {new BluePreloadedHiveAuto(), new RedPreloadedHiveAuto()}) {
      Path path = new Path(new BezierLine(new Pose(0, 0, 0), new Pose(12, 0, 0)));
      auto.configureMovementPath(path);
      assertEquals(Double.POSITIVE_INFINITY, path.getPathEndTimeoutConstraint(), 0);
    }
    Path original = new Path(new BezierLine(new Pose(0, 0, 0), new Pose(12, 0, 0)));
    double timeout = original.getPathEndTimeoutConstraint();
    new BiobuzzHiveAuto().configureMovementPath(original);
    assertTrue(Double.isFinite(timeout));
    assertEquals(timeout, original.getPathEndTimeoutConstraint(), 0);
  }

  @Test public void pickupArrivalIsTighterThanOrdinaryThreeInchWaypointArrival() {
    for (BiobuzzHiveAuto auto : new BiobuzzHiveAuto[] {new BluePreloadedHiveAuto(), new RedPreloadedHiveAuto()}) {
      assertEquals(1, auto.pickupPositionToleranceInches(), 0);
      assertTrue(auto.positionWithinTolerance(1, auto.pickupPositionToleranceInches()));
      assertFalse(auto.positionWithinTolerance(3, auto.pickupPositionToleranceInches()));
      assertTrue(auto.positionWithinTolerance(3));
    }
  }

  @Test public void bothPracticeRoutesAcceptThreeInchErrorsButRejectSix() {
    for (BiobuzzHiveAuto auto : new BiobuzzHiveAuto[] {new BluePreloadedHiveAuto(), new RedPreloadedHiveAuto()}) {
      assertEquals(3, auto.positionToleranceInches(), 0);
      assertTrue(auto.positionWithinTolerance(3));
      assertFalse(auto.positionWithinTolerance(3.01));
      assertFalse(auto.positionWithinTolerance(6));
      assertFalse(auto.positionWithinTolerance(Double.NaN));
      assertFalse(auto.positionWithinTolerance(Double.POSITIVE_INFINITY));
    }
  }

  @Test public void originalAutoRetainsTightPositionTolerance() {
    BiobuzzHiveAuto auto = new BiobuzzHiveAuto();
    assertEquals(1.5, auto.positionToleranceInches(), 0);
    assertTrue(auto.positionWithinTolerance(1.5));
    assertFalse(auto.positionWithinTolerance(6));
  }

  @Test public void blueAndRedUsePlayConfirmationAndKeepTheirAllianceLocked() {
    BiobuzzHiveAuto blue = new BluePreloadedHiveAuto();
    BiobuzzHiveAuto red = new RedPreloadedHiveAuto();
    assertFalse(blue.requiresPlacementConfirmation());
    assertFalse(red.requiresPlacementConfirmation());
    assertFalse(blue.allowAllianceSelection());
    assertFalse(red.allowAllianceSelection());
    assertEquals(Alliance.BLUE, blue.initialAlliance());
    assertEquals(Alliance.RED, red.initialAlliance());
    assertTrue(blue.canRunRoute());
    assertTrue(red.canRunRoute());
    assertEquals(Double.POSITIVE_INFINITY, blue.autoBudgetSeconds(), 0);
    assertEquals(Double.POSITIVE_INFINITY, red.autoBudgetSeconds(), 0);
    assertEquals(Double.POSITIVE_INFINITY, blue.legTimeoutSeconds(), 0);
    assertEquals(Double.POSITIVE_INFINITY, red.legTimeoutSeconds(), 0);
  }

  @Test public void originalHiveAutoStillRequiresExplicitPlacementConfirmation() {
    assertTrue(new BiobuzzHiveAuto().requiresPlacementConfirmation());
    assertEquals(28, new BiobuzzHiveAuto().autoBudgetSeconds(), 0);
    assertEquals(5, new BiobuzzHiveAuto().legTimeoutSeconds(), 0);
  }
}
