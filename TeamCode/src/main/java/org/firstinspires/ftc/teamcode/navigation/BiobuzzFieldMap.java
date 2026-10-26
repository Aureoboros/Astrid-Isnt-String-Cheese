package org.firstinspires.ftc.teamcode.navigation;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * BIOBUZZ TU06 nominal tile map (sections 9.2/9.4, G304). Origin is the audience-left
 * inside-wall corner: +X goes toward column F, +Y goes away from the audience toward row 6.
 * Heading 0 faces +X, 90 faces +Y. Positions refer to the robot center, not a bumper or pod.
 * This is a location map, NOT a collision map or a certification of legal physical placement.
 */
public final class BiobuzzFieldMap {
  public static final double FIELD_SIZE_INCHES = 144.0;
  public static final double TILE_SIZE_INCHES = 24.0;
  public static final double WALL_EXIT_INCHES = 12.0;
  public static final String MANUAL_REFERENCE = "BIOBUZZ TU06: 9.2, 9.4, G304, Figure 11-1";

  public enum Alliance { RED, BLUE }

  private enum Wall { WEST, EAST, SOUTH, NORTH }

  /** Team-chosen rectangular-robot presets in legal starting regions, not official fixed slots. */
  public enum StartLocation {
    RED_A3_WEST(Alliance.RED, Wall.WEST, 64),
    RED_A6_WEST(Alliance.RED, Wall.WEST, 132),
    RED_B1_AUDIENCE(Alliance.RED, Wall.SOUTH, 36),
    RED_B6_FAR(Alliance.RED, Wall.NORTH, 32),
    BLUE_F4_EAST(Alliance.BLUE, Wall.EAST, 80),
    BLUE_F1_EAST(Alliance.BLUE, Wall.EAST, 12),
    BLUE_E6_FAR(Alliance.BLUE, Wall.NORTH, 108),
    BLUE_E1_AUDIENCE(Alliance.BLUE, Wall.SOUTH, 112);

    private final Alliance alliance;
    private final Wall wall;
    private final double alongWallInches;

    StartLocation(Alliance alliance, Wall wall, double alongWallInches) {
      this.alliance = alliance;
      this.wall = wall;
      this.alongWallInches = alongWallInches;
    }

    public Alliance getAlliance() { return alliance; }
    public String getWall() { return wall.name(); }
    public String clearWallLocationName() { return name() + "_CLEAR_WALL"; }
  }

  private final double robotLengthInches;
  private final double robotWidthInches;
  private final Map<String, Waypoint> locations = new LinkedHashMap<>();

  public BiobuzzFieldMap(double robotLengthInches, double robotWidthInches) {
    // R102: nominal maximum 18 x 18 inch starting footprint. Use actual measured dimensions.
    if (!Double.isFinite(robotLengthInches) || !Double.isFinite(robotWidthInches)
        || robotLengthInches <= 0 || robotWidthInches <= 0
        || robotLengthInches > 18 || robotWidthInches > 18) {
      throw new IllegalArgumentException("Starting footprint must be positive and at most 18 x 18 in");
    }
    this.robotLengthInches = robotLengthInches;
    this.robotWidthInches = robotWidthInches;
    for (int row = 1; row <= 6; row++) {
      for (char column = 'A'; column <= 'F'; column++) {
        registerLocation("" + column + row, tileCenter(column, row, 0));
      }
    }
    for (StartLocation start : StartLocation.values()) {
      Waypoint pose = startPose(start);
      registerLocation(start.name(), pose);
      double heading = Math.toRadians(pose.getHeadingDegrees());
      registerLocation(start.clearWallLocationName(), new Waypoint(
          pose.getX() + WALL_EXIT_INCHES * Math.cos(heading),
          pose.getY() + WALL_EXIT_INCHES * Math.sin(heading), pose.getHeadingDegrees()));
    }
  }

  public Waypoint startPose(StartLocation start) {
    double halfLength = robotLengthInches / 2.0;
    switch (start.wall) {
      case WEST: return new Waypoint(halfLength, start.alongWallInches, 0);
      case EAST: return new Waypoint(FIELD_SIZE_INCHES - halfLength, start.alongWallInches, 180);
      case SOUTH: return new Waypoint(start.alongWallInches, halfLength, 90);
      case NORTH: return new Waypoint(start.alongWallInches, FIELD_SIZE_INCHES - halfLength, 270);
      default: throw new IllegalStateException("Unknown wall");
    }
  }

  /** Adds a programmed destination. Duplicate names are rejected to avoid silently changing routes. */
  public void registerLocation(String name, Waypoint pose) {
    if (name == null || name.trim().isEmpty() || pose == null
        || !insideField(pose.getX(), pose.getY()) || !Double.isFinite(pose.getHeadingDegrees())) {
      throw new IllegalArgumentException("Location requires a name and a finite in-field pose");
    }
    if (locations.containsKey(name)) {
      throw new IllegalArgumentException("Duplicate location: " + name);
    }
    locations.put(name, pose);
  }

  public Waypoint getLocation(String name) {
    Waypoint location = locations.get(name);
    if (location == null) {
      throw new IllegalArgumentException("Unknown field location: " + name);
    }
    return location;
  }

  public Map<String, Waypoint> getLocations() { return Collections.unmodifiableMap(locations); }

  /** Matrix indexing is [row - 1][column - 'A']; row 1 is nearest the audience. */
  public Waypoint[][] getLocationMatrix() {
    Waypoint[][] matrix = new Waypoint[6][6];
    for (int row = 1; row <= 6; row++) {
      for (char column = 'A'; column <= 'F'; column++) {
        matrix[row - 1][column - 'A'] = getLocation("" + column + row);
      }
    }
    return matrix;
  }

  public boolean footprintInsideField(Waypoint pose) {
    if (pose == null || !Double.isFinite(pose.getHeadingDegrees())) {
      return false;
    }
    double angle = Math.toRadians(pose.getHeadingDegrees());
    double halfX = (Math.abs(Math.cos(angle)) * robotLengthInches
        + Math.abs(Math.sin(angle)) * robotWidthInches) / 2;
    double halfY = (Math.abs(Math.sin(angle)) * robotLengthInches
        + Math.abs(Math.cos(angle)) * robotWidthInches) / 2;
    double epsilon = 1e-6;
    return insideField(pose.getX(), pose.getY())
        && pose.getX() - halfX >= -epsilon && pose.getY() - halfY >= -epsilon
        && pose.getX() + halfX <= FIELD_SIZE_INCHES + epsilon
        && pose.getY() + halfY <= FIELD_SIZE_INCHES + epsilon;
  }

  public static List<StartLocation> startsFor(Alliance alliance) {
    List<StartLocation> starts = new ArrayList<>();
    for (StartLocation start : StartLocation.values()) {
      if (start.alliance == alliance) {
        starts.add(start);
      }
    }
    return Collections.unmodifiableList(starts);
  }

  public static Waypoint tileCenter(char column, int row, double headingDegrees) {
    if (column < 'A' || column > 'F' || row < 1 || row > 6 || !Double.isFinite(headingDegrees)) {
      throw new IllegalArgumentException("Tile must be A1 through F6 with finite heading");
    }
    return new Waypoint((column - 'A' + 0.5) * TILE_SIZE_INCHES,
        (row - 0.5) * TILE_SIZE_INCHES, headingDegrees);
  }

  /** Pose outside the nominal field is never clamped into a valid tile. */
  public static String tileAt(double x, double y) {
    if (!insideField(x, y)) {
      return "OUTSIDE FIELD / INVALID POSE";
    }
    int column = Math.min(5, (int) (x / TILE_SIZE_INCHES));
    int row = Math.min(5, (int) (y / TILE_SIZE_INCHES));
    return "" + (char) ('A' + column) + (row + 1);
  }

  private static boolean insideField(double x, double y) {
    return Double.isFinite(x) && Double.isFinite(y) && x >= 0 && y >= 0
        && x <= FIELD_SIZE_INCHES && y <= FIELD_SIZE_INCHES;
  }
}
