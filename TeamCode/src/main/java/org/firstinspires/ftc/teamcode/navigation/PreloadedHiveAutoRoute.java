package org.firstinspires.ftc.teamcode.navigation;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.teamcode.config.PreloadedHiveAutoConfig;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.Alliance;

/** Only side 1 starts with three nectars: four tip it; two four-nectar batches tip side 2. */
public final class PreloadedHiveAutoRoute extends HiveAutoRoute {
  private final Alliance alliance;
  private static final List<Stage> STAGES = Collections.unmodifiableList(Arrays.asList(
      Stage.PRELOAD_SHOT, Stage.FLOWER_1_PICKUP, Stage.FIRST_REFILL_SHOT,
      Stage.FLOWER_2_PICKUP, Stage.SECOND_REFILL_SHOT));

  public PreloadedHiveAutoRoute(Alliance alliance) { super(alliance); this.alliance = alliance; }

  private static double oppositeShotY() {
    // Red faces -Y here, so backing away from the hive increases Y; Blue is mirrored.
    return 40 + PreloadedHiveAutoConfig.OPPOSITE_SHOT_BACKOFF_INCHES;
  }

  @Override public List<Waypoint> oppositeApproach() {
    return points(new Waypoint(-44, -40, 90), new Waypoint(-44, 40, 90),
        new Waypoint(-44, oppositeShotY(), 90), new Waypoint(-12.75, oppositeShotY(), 270));
  }

  @Override public List<Stage> stages() { return STAGES; }
  @Override public boolean requiresTip(Stage stage) {
    return stage == Stage.PRELOAD_SHOT || stage == Stage.SECOND_REFILL_SHOT;
  }
  @Override public String description() {
    return "Side 1: shoot 4 -> left flower: pick 4 -> side 2: shoot 4"
        + " -> next flower: pick 4 -> side 2: shoot 4 -> park";
  }

  @Override public int tagMin(Stage stage) {
    if (stage.isPickup()) { throw new IllegalArgumentException("Pickup has no target tags"); }
    switch (stage) {
      case PRELOAD_SHOT: return firstTagMin();
      case FIRST_REFILL_SHOT: case SECOND_REFILL_SHOT: return secondTagMin();
      default: throw new IllegalArgumentException("Stage is not in this route");
    }
  }

  @Override public List<Waypoint> approach(Stage stage) {
    switch (stage) {
      case PRELOAD_SHOT: return firstApproach();
      case FLOWER_1_PICKUP:
        // Keep the depth-adjusted approach; extra reach is added only to the final intake creep.
        return points(new Waypoint(-44, -40, 180), new Waypoint(-44, -24, 180),
            new Waypoint(-49 + PreloadedHiveAutoConfig.FLOWER_PICKUP_DEPTH_INCHES, -24, 180));
      case FIRST_REFILL_SHOT:
        // Back away from flower 1; use the outer corridor around the hive.
        return points(new Waypoint(-44, -24, 180), new Waypoint(-44, 40, 90),
            new Waypoint(-44, oppositeShotY(), 90), new Waypoint(-12.75, oppositeShotY(), 270));
      case FLOWER_2_PICKUP:
        // Face the flower during lateral travel, not on the short final approach segment.
        return points(new Waypoint(-24, oppositeShotY(), 90),
            new Waypoint(-24, 49 - PreloadedHiveAutoConfig.FLOWER_PICKUP_DEPTH_INCHES, 90));
      case SECOND_REFILL_SHOT:
        // Back away from flower 2, then return to the SAME side 2 for its second batch.
        return points(new Waypoint(-24, 49, 90), new Waypoint(-24, oppositeShotY(), 270),
            new Waypoint(-12.75, oppositeShotY(), 270));
      default: throw new IllegalArgumentException("Stage is not in this route");
    }
  }

  @Override public Waypoint pickupTarget(Stage stage) {
    if (stage != Stage.FLOWER_1_PICKUP && stage != Stage.FLOWER_2_PICKUP) {
      throw new IllegalArgumentException("Only the two alliance flowers are visited");
    }
    Waypoint previous = super.pickupTarget(stage);
    double h = Math.toRadians(previous.getHeadingDegrees());
    double standoff = PreloadedHiveAutoConfig.FLOWER_PICKUP_DEPTH_INCHES
        - PreloadedHiveAutoConfig.FLOWER_PICKUP_EXTRA_REACH_INCHES
        - (stage == Stage.FLOWER_2_PICKUP ? PreloadedHiveAutoConfig.FLOWER_2_ADDITIONAL_REACH_INCHES : 0);
    return new Waypoint(previous.getX() - standoff * Math.cos(h),
        previous.getY() - standoff * Math.sin(h), previous.getHeadingDegrees());
  }

  @Override public List<Waypoint> parkFrom(Waypoint current) {
    // Turn in the outer corridor before moving laterally to the inner parking corner.
    double redCurrentY = alliance == Alliance.RED ? current.getY() : -current.getY();
    double redHeading = alliance == Alliance.RED ? current.getHeadingDegrees()
        : (current.getHeadingDegrees() + 180) % 360;
    return points(new Waypoint(-44, redCurrentY, redHeading),
        new Waypoint(-44, PreloadedHiveAutoConfig.RED_PARK_CORNER_Y, 180),
        new Waypoint(PreloadedHiveAutoConfig.RED_PARK_CORNER_X,
            PreloadedHiveAutoConfig.RED_PARK_CORNER_Y, 180));
  }
}
