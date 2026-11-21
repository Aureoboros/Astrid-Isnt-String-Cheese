/**
package org.firstinspires.ftc.teamcode.navigation;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.Alliance;

/** Only side 1 starts with three nectars: four tip it; two four-nectar batches tip side 2. */
/**
public final class PreloadedHiveAutoRoute extends HiveAutoRoute {
  private static final List<Stage> STAGES = Collections.unmodifiableList(Arrays.asList(
      Stage.PRELOAD_SHOT, Stage.FLOWER_1_PICKUP, Stage.FIRST_REFILL_SHOT,
      Stage.FLOWER_2_PICKUP, Stage.SECOND_REFILL_SHOT));

  public PreloadedHiveAutoRoute(Alliance alliance) { super(alliance); }

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
      case FLOWER_1_PICKUP: return super.approach(stage);
      case FIRST_REFILL_SHOT:
        // Back away from flower 1; use the outer corridor around the hive.
        return points(new Waypoint(-44, -24, 180), new Waypoint(-44, 40, 90),
            new Waypoint(-12.75, 40, 270));
      case FLOWER_2_PICKUP:
        return points(new Waypoint(-24, 40, 270), new Waypoint(-24, 49, 90));
      case SECOND_REFILL_SHOT:
        // Back away from flower 2, then return to the SAME side 2 for its second batch.
        return points(new Waypoint(-24, 49, 90), new Waypoint(-24, 40, 270),
            new Waypoint(-12.75, 40, 270));
      default: throw new IllegalArgumentException("Stage is not in this route");
    }
  }

  @Override public Waypoint pickupTarget(Stage stage) {
    if (stage != Stage.FLOWER_1_PICKUP && stage != Stage.FLOWER_2_PICKUP) {
      throw new IllegalArgumentException("Only the two alliance flowers are visited");
    }
    return super.pickupTarget(stage);
  }
}
*/