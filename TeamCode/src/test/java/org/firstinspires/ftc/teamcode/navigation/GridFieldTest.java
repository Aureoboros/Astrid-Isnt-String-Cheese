package org.firstinspires.ftc.teamcode.navigation;

import static org.junit.Assert.*;
import org.junit.Test;

public final class GridFieldTest {
  private static final double EPSILON = 1e-9;

  @Test
  public void gridUnitsAreFeet() {
    assertEquals(24, GridField.toInches(2), EPSILON);
    assertEquals(72, GridField.toInches(GridField.MAX), EPSILON);
    assertEquals(1.5, GridField.toGrid(18), EPSILON);
  }

  @Test
  public void clampKeepsPointsOnTheField() {
    assertEquals(0, GridField.clamp(-1));
    assertEquals(6, GridField.clamp(7));
    assertEquals(3, GridField.clamp(3));
    assertEquals(6, GridField.nearestGrid(100));
    assertEquals(1, GridField.nearestGrid(15));
  }

  @Test
  public void facingPlusYWithTagStraightAheadPutsRobotBelowTag() {
    double[] p = GridField.robotPositionFromTag(10, 0, Math.PI / 2);
    assertEquals(24, p[0], EPSILON);
    assertEquals(2, p[1], EPSILON);
  }

  @Test
  public void tagOffToTheLeftShiftsRobotRight() {
    // Facing +Y, the robot's left is -X, so a tag on the left means the robot is at larger X.
    double[] p = GridField.robotPositionFromTag(10, 6, Math.PI / 2);
    assertEquals(30, p[0], EPSILON);
    assertEquals(2, p[1], EPSILON);
  }

  @Test
  public void facingPlusXWithTagAheadPutsRobotLeftOfTag() {
    double[] p = GridField.robotPositionFromTag(12, 0, 0);
    assertEquals(12, p[0], EPSILON);
    assertEquals(12, p[1], EPSILON);
  }

  @Test
  public void headingToCenterPointsAtThreeThree() {
    assertEquals(Math.PI / 4, GridField.headingToCenter(0, 0, 0), EPSILON);
    assertEquals(Math.PI, GridField.headingToCenter(72, 36, 0), EPSILON);
    assertEquals(-Math.PI / 2, GridField.headingToCenter(36, 72, 0), EPSILON);
  }

  @Test
  public void headingToCenterKeepsFallbackWhenAlreadyAtCenter() {
    assertEquals(1.23, GridField.headingToCenter(36, 36, 1.23), EPSILON);
  }

  @Test
  public void angleErrorWrapsAround() {
    assertEquals(Math.toRadians(20), GridField.angleError(Math.toRadians(10), Math.toRadians(350)),
        1e-9);
  }
}
