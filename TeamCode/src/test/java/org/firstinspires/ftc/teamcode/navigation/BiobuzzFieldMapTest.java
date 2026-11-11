package org.firstinspires.ftc.teamcode.navigation;

import static org.junit.Assert.*;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.Alliance;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.StartLocation;
import org.junit.Test;

public final class BiobuzzFieldMapTest {
  private static final double EPSILON = 1e-6;
  private final BiobuzzFieldMap field = new BiobuzzFieldMap(18, 18);

  @Test
  public void matrixMatchesManualColumnsAndAudienceToFarRows() {
    Waypoint[][] matrix = field.getLocationMatrix();
    assertEquals(6, matrix.length);
    assertEquals(6, matrix[0].length);
    assertEquals(-60, matrix[0][0].getX(), EPSILON);
    assertEquals(-60, matrix[0][0].getY(), EPSILON);
    assertEquals(60, matrix[5][5].getX(), EPSILON);
    assertEquals(60, matrix[5][5].getY(), EPSILON);
    for (int row = 0; row < 6; row++) {
      for (int column = 0; column < 6; column++) {
        assertEquals("" + (char) ('A' + column) + (row + 1),
            BiobuzzFieldMap.tileAt(matrix[row][column].getX(), matrix[row][column].getY()));
      }
    }
  }

  @Test
  public void tileSeamsAndWallsHaveDeterministicAssignments() {
    assertEquals("A1", BiobuzzFieldMap.tileAt(-72, -72));
    assertEquals("A1", BiobuzzFieldMap.tileAt(-48.01, -48.01));
    assertEquals("B2", BiobuzzFieldMap.tileAt(-48, -48));
    assertEquals("D4", BiobuzzFieldMap.tileAt(0, 0));
    assertEquals("C3", BiobuzzFieldMap.tileAt(-0.01, -0.01));
    assertEquals("F6", BiobuzzFieldMap.tileAt(72, 72));
    assertTrue(BiobuzzFieldMap.tileAt(-73, 20).startsWith("OUTSIDE"));
    assertTrue(BiobuzzFieldMap.tileAt(73, 20).startsWith("OUTSIDE"));
    assertTrue(BiobuzzFieldMap.tileAt(Double.NaN, 20).startsWith("OUTSIDE"));
  }

  @Test
  public void redAndBluePresetsHaveCorrespondingRotatedGeometry() {
    for (int i = 0; i < 4; i++) {
      Waypoint red = field.startPose(BiobuzzFieldMap.startsFor(Alliance.RED).get(i));
      Waypoint blue = field.startPose(BiobuzzFieldMap.startsFor(Alliance.BLUE).get(i));
      assertEquals(-red.getX(), blue.getX(), EPSILON);
      assertEquals(-red.getY(), blue.getY(), EPSILON);
      assertEquals(0, Math.sin(Math.toRadians(blue.getHeadingDegrees()
          - red.getHeadingDegrees() - 180)), EPSILON);
      assertEquals(1, Math.cos(Math.toRadians(blue.getHeadingDegrees()
          - red.getHeadingDegrees() - 180)), EPSILON);
    }
  }

  @Test
  public void presetsAreWallAlignedAndFullyOnOwnAllianceHalf() {
    for (StartLocation start : StartLocation.values()) {
      Waypoint pose = field.startPose(start);
      assertTrue(start.name(), field.footprintInsideField(pose));
      if (start.getAlliance() == Alliance.RED) {
        assertTrue(pose.getX() + 9 < 0);
      } else {
        assertTrue(pose.getX() - 9 > 0);
      }
      assertTrue(Math.abs(pose.getX()) == 63 || Math.abs(pose.getY()) == 63);
      // Nominal loading rectangles in Fig 9-2: red west rows 5; blue east row 2.
      assertFalse(pose.getX() - 9 < -61 && pose.getY() + 9 >= 24 && pose.getY() - 9 <= 48);
      assertFalse(pose.getX() + 9 > 61 && pose.getY() + 9 >= -48 && pose.getY() - 9 <= -24);
    }
  }

  @Test
  public void actualRobotLengthChangesWallContactOffset() {
    BiobuzzFieldMap smallRobot = new BiobuzzFieldMap(16, 14);
    assertEquals(-64, smallRobot.startPose(StartLocation.RED_A3_WEST).getX(), EPSILON);
    assertEquals(64, smallRobot.startPose(StartLocation.BLUE_F4_EAST).getX(), EPSILON);
    assertEquals(-64, smallRobot.startPose(StartLocation.RED_B1_AUDIENCE).getY(), EPSILON);
  }

  @Test
  public void wallExitTargetsAreTwelveInchesAheadAndKeepHeading() {
    for (StartLocation start : StartLocation.values()) {
      Waypoint pose = field.startPose(start);
      Waypoint target = field.getLocation(start.clearWallLocationName());
      assertEquals(12, Math.hypot(target.getX() - pose.getX(), target.getY() - pose.getY()), EPSILON);
      assertEquals(pose.getHeadingDegrees(), target.getHeadingDegrees(), EPSILON);
      double angle = Math.toRadians(pose.getHeadingDegrees());
      assertEquals(12, (target.getX() - pose.getX()) * Math.cos(angle)
          + (target.getY() - pose.getY()) * Math.sin(angle), EPSILON);
      assertTrue(field.footprintInsideField(target));
    }
  }

  @Test
  public void namedTeamLocationsAreResolvedWithoutASecondCoordinateSystem() {
    Waypoint target = new Waypoint(-36, -36, 90);
    field.registerLocation("RED_APPROACH", target);
    assertSame(target, field.getLocation("RED_APPROACH"));
    assertEquals("B2", BiobuzzFieldMap.tileAt(target.getX(), target.getY()));
  }

  @Test
  public void originIsFieldCenterAndQuadrantsFollowSigns() {
    assertEquals("CENTER", BiobuzzFieldMap.quadrantAt(0, 0));
    assertTrue(BiobuzzFieldMap.quadrantAt(10, 10).startsWith("Q1"));
    assertTrue(BiobuzzFieldMap.quadrantAt(-10, 10).startsWith("Q2"));
    assertTrue(BiobuzzFieldMap.quadrantAt(-10, -10).startsWith("Q3"));
    assertTrue(BiobuzzFieldMap.quadrantAt(10, -10).startsWith("Q4"));
    assertEquals("ON X AXIS", BiobuzzFieldMap.quadrantAt(10, 0));
    assertEquals("ON Y AXIS", BiobuzzFieldMap.quadrantAt(0, -10));
    assertTrue(BiobuzzFieldMap.quadrantAt(80, 0).startsWith("OUTSIDE"));
    // Red starts are on the -X half, blue starts on the +X half.
    for (StartLocation start : StartLocation.values()) {
      String quadrant = BiobuzzFieldMap.quadrantAt(field.startPose(start).getX(),
          field.startPose(start).getY());
      assertTrue(start.name(), start.getAlliance() == Alliance.RED
          ? quadrant.startsWith("Q2") || quadrant.startsWith("Q3")
          : quadrant.startsWith("Q1") || quadrant.startsWith("Q4"));
    }
  }

  @Test(expected = IllegalArgumentException.class)
  public void duplicateNamesCannotSilentlyChangeRoutes() {
    field.registerLocation("A1", new Waypoint(1, 1, 0));
  }

  @Test(expected = IllegalArgumentException.class)
  public void unknownLocationsFailBeforeDriving() { field.getLocation("UNKNOWN"); }

  @Test(expected = IllegalArgumentException.class)
  public void invalidStartingFootprintIsRejected() { new BiobuzzFieldMap(19, 18); }

  @Test(expected = IllegalArgumentException.class)
  public void invalidTileIsRejected() { BiobuzzFieldMap.tileCenter('G', 2, 0); }

  @Test
  public void footprintChecksIncludeRotatedRobotCorners() {
    assertTrue(field.footprintInsideField(new Waypoint(-63, -63, 0)));
    assertFalse(field.footprintInsideField(new Waypoint(-63, -63, 45)));
    assertTrue(field.footprintInsideField(new Waypoint(63, 63, 0)));
    assertFalse(field.footprintInsideField(new Waypoint(64, 0, 0)));
    assertFalse(field.footprintInsideField(new Waypoint(Double.NaN, 20, 0)));
  }
}
