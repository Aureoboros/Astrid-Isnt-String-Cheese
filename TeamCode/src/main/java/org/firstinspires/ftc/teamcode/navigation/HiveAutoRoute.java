package org.firstinspires.ftc.teamcode.navigation;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.Alliance;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.StartLocation;

/** Nominal, team-chosen route; coordinates must be checked against actual field CAD and robot. */
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
    return points(new Waypoint(36, 21, 90), new Waypoint(36, 32, 90),
        new Waypoint(59.25, 32, 90));
  }

  public List<Waypoint> oppositeApproach() {
    return points(new Waypoint(28, 32, 90), new Waypoint(28, 112, 90),
        new Waypoint(59.25, 112, 270));
  }

  /** Retreat laterally before traversing the corridor; never draw a straight line through the hive. */
  public List<Waypoint> parkFrom(Waypoint current) {
    double corridorX = alliance == Alliance.RED ? 28 : 116;
    double parkY = alliance == Alliance.RED ? 108 : 36;
    double parkX = alliance == Alliance.RED ? 12 : 132;
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
        return points(new Waypoint(28, 32, 180), new Waypoint(28, 48, 180), new Waypoint(23, 48, 180));
      case FIRST_REFILL_SHOT:
        return points(new Waypoint(28, 48, 180), new Waypoint(28, 32, 90), new Waypoint(59.25, 32, 90));
      case FLOWER_2_PICKUP:
        return points(new Waypoint(28, 32, 90), new Waypoint(28, 112, 90),
            new Waypoint(48, 112, 90), new Waypoint(48, 121, 90));
      case SECOND_INITIAL_SHOT:
        return points(new Waypoint(48, 121, 90), new Waypoint(48, 112, 270), new Waypoint(59.25, 112, 270));
      case GARDEN_PICKUP:
        return points(new Waypoint(28, 112, 270), new Waypoint(28, 20, 270), new Waypoint(12, 20, 270));
      case SECOND_REFILL_SHOT:
        // Reverse away from the garden wall before turning; do not rotate an 18-inch robot at Y=12.
        return points(new Waypoint(12, 20, 270), new Waypoint(28, 20, 0),
            new Waypoint(28, 112, 0), new Waypoint(59.25, 112, 270));
      default: throw new IllegalArgumentException("Unknown stage");
    }
  }

  /** Last 8-inch intake creep. Confirm actual reach, opening clearance and pickup mechanism first. */
  public Waypoint pickupTarget(Stage stage) {
    switch (stage) {
      case FLOWER_1_PICKUP: return points(new Waypoint(15, 48, 180)).get(0);
      case FLOWER_2_PICKUP: return points(new Waypoint(48, 129, 90)).get(0);
      case GARDEN_PICKUP: return points(new Waypoint(12, 12, 270)).get(0);
      default: throw new IllegalArgumentException("Not a pickup stage");
    }
  }

  private List<Waypoint> points(Waypoint... red) {
    if (alliance == Alliance.BLUE) {
      for (int i = 0; i < red.length; i++) {
        Waypoint p = red[i];
        red[i] = new Waypoint(144 - p.getX(), 144 - p.getY(),
            (p.getHeadingDegrees() + 180) % 360);
      }
    }
    return Collections.unmodifiableList(Arrays.asList(red));
  }
}
