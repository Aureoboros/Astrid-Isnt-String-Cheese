package org.firstinspires.ftc.teamcode.navigation;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.Alliance;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.StartLocation;

/**
 * Nominal, team-chosen route; coordinates must be checked against actual field CAD and robot.
 * Field-center coordinates (see BiobuzzFieldMap): red routes live in -X, blue is red rotated 180.
 */
public final class HiveAutoRoute {
  /** Ordered four-pollen batches; two flowers and own garden supply the three refills. */
  public enum Stage {
    PRELOAD_SHOT(false, false), FLOWER_1_PICKUP(true, false), FIRST_REFILL_SHOT(false, true),
    FLOWER_2_PICKUP(true, false), SECOND_INITIAL_SHOT(false, false),
    GARDEN_PICKUP(true, false), SECOND_REFILL_SHOT(false, true);
    private final boolean pickup, requireTip;
    Stage(boolean pickup, boolean requireTip) { this.pickup = pickup; this.requireTip = requireTip; }
    public boolean isPickup() { return pickup; }
    public boolean requiresTip() { return requireTip; }
  }
  private final Alliance alliance;
  public HiveAutoRoute(Alliance alliance) { this.alliance = alliance; }

  public StartLocation start() {
    return alliance == Alliance.RED ? StartLocation.RED_B1_AUDIENCE : StartLocation.BLUE_E6_FAR;
  }

  public List<Waypoint> firstApproach() {
    return points(new Waypoint(-36, -51, 90), new Waypoint(-36, -40, 90),
        new Waypoint(-12.75, -40, 90));
  }

  public List<Waypoint> oppositeApproach() {
    return points(new Waypoint(-44, -40, 90), new Waypoint(-44, 40, 90),
        new Waypoint(-12.75, 40, 270));
  }

  /** Retreat laterally before traversing the corridor; never draw a straight line through the hive. */
  public List<Waypoint> parkFrom(Waypoint current) {
    double corridorX = alliance == Alliance.RED ? -44 : 44;
    double parkY = alliance == Alliance.RED ? 36 : -36;
    double parkX = alliance == Alliance.RED ? -60 : 60;
    double heading = alliance == Alliance.RED ? 180 : 0;
    return Collections.unmodifiableList(Arrays.asList(
        new Waypoint(corridorX, current.getY(), current.getHeadingDegrees()),
        new Waypoint(corridorX, parkY, heading), new Waypoint(parkX, parkY, heading)));
  }

  public int firstTagMin() { return alliance == Alliance.RED ? 34 : 42; }
  public int secondTagMin() { return alliance == Alliance.RED ? 30 : 38; }

  public int tagMin(Stage stage) {
    if (stage.isPickup()) { throw new IllegalArgumentException("Pickup has no target tag cluster"); }
    return stage == Stage.PRELOAD_SHOT || stage == Stage.FIRST_REFILL_SHOT ? firstTagMin() : secondTagMin();
  }

  /** All flower/garden positions are provisional ROBOT-CENTER approach poses, not flower centers. */
  public List<Waypoint> approach(Stage stage) {
    switch (stage) {
      case PRELOAD_SHOT: return firstApproach();
      case FLOWER_1_PICKUP:
        return points(new Waypoint(-44, -40, 180), new Waypoint(-44, -24, 180), new Waypoint(-49, -24, 180));
      case FIRST_REFILL_SHOT:
        return points(new Waypoint(-44, -24, 180), new Waypoint(-44, -40, 90), new Waypoint(-12.75, -40, 90));
      case FLOWER_2_PICKUP:
        return points(new Waypoint(-44, -40, 90), new Waypoint(-44, 40, 90),
            new Waypoint(-24, 40, 90), new Waypoint(-24, 49, 90));
      case SECOND_INITIAL_SHOT:
        return points(new Waypoint(-24, 49, 90), new Waypoint(-24, 40, 270), new Waypoint(-12.75, 40, 270));
      case GARDEN_PICKUP:
        return points(new Waypoint(-44, 40, 270), new Waypoint(-44, -52, 270), new Waypoint(-60, -52, 270));
      case SECOND_REFILL_SHOT:
        // Reverse away from the garden wall before turning; do not rotate an 18-inch robot at Y=-60.
        return points(new Waypoint(-60, -52, 270), new Waypoint(-44, -52, 0),
            new Waypoint(-44, 40, 0), new Waypoint(-12.75, 40, 270));
      default: throw new IllegalArgumentException("Unknown stage");
    }
  }

  /** Last 8-inch intake creep. Confirm actual reach, opening clearance and pickup mechanism first. */
  public Waypoint pickupTarget(Stage stage) {
    switch (stage) {
      case FLOWER_1_PICKUP: return points(new Waypoint(-57, -24, 180)).get(0);
      case FLOWER_2_PICKUP: return points(new Waypoint(-24, 57, 90)).get(0);
      case GARDEN_PICKUP: return points(new Waypoint(-60, -60, 270)).get(0);
      default: throw new IllegalArgumentException("Not a pickup stage");
    }
  }

  private List<Waypoint> points(Waypoint... red) {
    if (alliance == Alliance.BLUE) {
      for (int i = 0; i < red.length; i++) {
        Waypoint p = red[i];
        red[i] = new Waypoint(-p.getX(), -p.getY(),
            (p.getHeadingDegrees() + 180) % 360);
      }
    }
    return Collections.unmodifiableList(Arrays.asList(red));
  }
}
