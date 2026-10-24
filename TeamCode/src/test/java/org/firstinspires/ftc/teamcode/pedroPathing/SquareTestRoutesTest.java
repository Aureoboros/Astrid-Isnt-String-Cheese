package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.junit.Assert.*;
import java.util.List;
import org.firstinspires.ftc.teamcode.pedroPathing.SquareTestRoutes.Corner;
import org.firstinspires.ftc.teamcode.pedroPathing.SquareTestRoutes.Shape;
import org.junit.Test;

public final class SquareTestRoutesTest {
  private static final double EPSILON = 1e-9;

  @Test
  public void shapesRunInRequestedOrder() {
    assertArrayEquals(
        new Shape[] {
          Shape.SQUARE_6_INCH, Shape.SQUARE_1_FOOT, Shape.SQUARE_4_FOOT,
          Shape.SQUARE_1_FOOT_ROTATED, Shape.TRIANGLE_2_FOOT
        },
        Shape.values());
  }

  @Test
  public void squaresGoForwardThenLeftFromTheRightCorner() {
    assertCorners(Shape.SQUARE_6_INCH.getCorners(),
        0, 0, 0, 6, 0, 0, 6, 6, 0, 0, 6, 0, 0, 0, 0);
    assertCorners(Shape.SQUARE_1_FOOT.getCorners(),
        0, 0, 0, 12, 0, 0, 12, 12, 0, 0, 12, 0, 0, 0, 0);
    assertCorners(Shape.SQUARE_4_FOOT.getCorners(),
        0, 0, 0, 48, 0, 0, 48, 48, 0, 0, 48, 0, 0, 0, 0);
  }

  @Test
  public void rotatedSquareTurnsOnceThenKeepsThirtyDegreeHeadingOnAllFourSides() {
    double forwardX = 12 * Math.cos(Math.toRadians(30));
    double forwardY = 12 * Math.sin(Math.toRadians(30));
    assertCorners(Shape.SQUARE_1_FOOT_ROTATED.getCorners(),
        0, 0, 0, 0, 0, 30,
        forwardX, forwardY, 30,
        forwardX - forwardY, forwardY + forwardX, 30,
        -forwardY, forwardX, 30,
        0, 0, 30);
  }

  @Test
  public void rotatedSquareHasFourEqualPerpendicularSidesAndReturnsHome() {
    List<Corner> corners = Shape.SQUARE_1_FOOT_ROTATED.getCorners();
    assertEquals(6, corners.size());
    assertTrue(corners.get(0).samePositionAs(corners.get(1)));
    for (int i = 2; i < corners.size(); i++) {
      Corner a = corners.get(i - 1);
      Corner b = corners.get(i);
      assertEquals(12, Math.hypot(b.getX() - a.getX(), b.getY() - a.getY()), EPSILON);
      if (i > 2) {
        Corner previous = corners.get(i - 2);
        double dot = (a.getX() - previous.getX()) * (b.getX() - a.getX())
            + (a.getY() - previous.getY()) * (b.getY() - a.getY());
        assertEquals(0, dot, EPSILON);
      }
    }
    assertTrue(corners.get(corners.size() - 1).samePositionAs(corners.get(0)));
  }

  @Test
  public void triangleHasThreeEqualTwoFootSidesAndReturnsHome() {
    List<Corner> corners = Shape.TRIANGLE_2_FOOT.getCorners();
    assertEquals(4, corners.size());
    for (int i = 1; i < corners.size(); i++) {
      Corner a = corners.get(i - 1);
      Corner b = corners.get(i);
      assertEquals(24, Math.hypot(b.getX() - a.getX(), b.getY() - a.getY()), EPSILON);
      assertEquals(0, b.getHeadingDegrees(), EPSILON);
    }
    assertTrue(corners.get(3).samePositionAs(corners.get(0)));
    // Starting at the right corner means the other corners are forward and to the left.
    assertTrue(corners.get(1).getX() > 0 && corners.get(2).getY() > 0);
  }

  private static void assertCorners(List<Corner> corners, double... expected) {
    assertEquals(expected.length / 3, corners.size());
    for (int i = 0; i < corners.size(); i++) {
      Corner corner = corners.get(i);
      assertEquals("x " + i, expected[i * 3], corner.getX(), EPSILON);
      assertEquals("y " + i, expected[i * 3 + 1], corner.getY(), EPSILON);
      assertEquals("heading " + i, expected[i * 3 + 2], corner.getHeadingDegrees(), EPSILON);
    }
  }
}
