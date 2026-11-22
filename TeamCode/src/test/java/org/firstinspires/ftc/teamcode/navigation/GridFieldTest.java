package org.firstinspires.ftc.teamcode.navigation;

import static org.junit.Assert.*;
import org.junit.Test;

public final class GridFieldTest {
  private static final double EPSILON = 1e-9;

  @Test
  public void gridUnitsAreHalfATileWithCenteredOrigin() {
    assertEquals(24, GridField.toInches(2), EPSILON);
    assertEquals(72, GridField.toInches(GridField.MAX), EPSILON);
    assertEquals(-72, GridField.toInches(GridField.MIN), EPSILON);
    assertEquals(3, GridField.toGrid(36), EPSILON);
  }

  @Test
  public void clampKeepsPointsOnTheField() {
    assertEquals(-1, GridField.clamp(-1));
    assertEquals(-6, GridField.clamp(-7));
    assertEquals(6, GridField.clamp(7));
    assertEquals(3, GridField.clamp(3));
    assertEquals(6, GridField.nearestGrid(200));
    assertEquals(3, GridField.nearestGrid(30));
  }

  @Test
  public void facingPlusYWithTagStraightAheadPutsRobotBelowTag() {
    double[] p = GridField.robotPositionFromTag(20, 0, Math.PI / 2);
    assertEquals(12, p[0], EPSILON);
    assertEquals(-44, p[1], EPSILON);
  }

  @Test
  public void tagOffToTheLeftShiftsRobotRight() {
    // Facing +Y, the robot's left is -X, so a tag on the left means the robot is at larger X.
    double[] p = GridField.robotPositionFromTag(20, 6, Math.PI / 2);
    assertEquals(18, p[0], EPSILON);
    assertEquals(-44, p[1], EPSILON);
  }

  @Test
  public void facingPlusXWithTagAheadPutsRobotLeftOfTag() {
    double[] p = GridField.robotPositionFromTag(12, 0, 0);
    assertEquals(0, p[0], EPSILON);
    assertEquals(-24, p[1], EPSILON);
  }

  @Test
  public void headingToCenterPointsAtZeroZero() {
    assertEquals(Math.PI / 4, GridField.headingToCenter(-12, -12, 0), EPSILON);
    assertEquals(Math.PI, GridField.headingToCenter(72, 0, 0), EPSILON);
    assertEquals(-Math.PI / 2, GridField.headingToCenter(0, 72, 0), EPSILON);
  }

  @Test
  public void headingToCenterKeepsFallbackWhenAlreadyAtCenter() {
    assertEquals(1.23, GridField.headingToCenter(0, 0, 1.23), EPSILON);
  }

  @Test
  public void angleErrorWrapsAround() {
    assertEquals(Math.toRadians(20), GridField.angleError(Math.toRadians(10), Math.toRadians(350)),
        1e-9);
  }
}
