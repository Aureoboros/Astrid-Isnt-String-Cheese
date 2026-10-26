package org.firstinspires.ftc.teamcode.navigation;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.Alliance;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.StartLocation;

/** Nominal, team-chosen route; coordinates must be checked against actual field CAD and robot. */
public final class HiveAutoRoute {
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
