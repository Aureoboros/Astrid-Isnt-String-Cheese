package org.firstinspires.ftc.teamcode.localization;

import static org.junit.Assert.*;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.Alliance;
import org.firstinspires.ftc.teamcode.navigation.Waypoint;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public final class AutoPoseHandoffTest {
  @Before public void startClean() { AutoPoseHandoff.clear(); }
  @After public void clearSavedPose() { AutoPoseHandoff.clear(); }

  @Test public void savedPoseIsAvailableOnlyForItsAlliance() {
    Waypoint blue = new Waypoint(61, -48, 0);
    AutoPoseHandoff.save(blue, Alliance.BLUE);
    assertTrue(AutoPoseHandoff.hasPose());
    assertTrue(AutoPoseHandoff.hasPoseFor(Alliance.BLUE));
    assertFalse(AutoPoseHandoff.hasPoseFor(Alliance.RED));
    assertFalse(AutoPoseHandoff.hasPoseFor(null));
    assertSame(blue, AutoPoseHandoff.getPose());
    assertEquals(Alliance.BLUE, AutoPoseHandoff.getAlliance());
  }

  @Test public void clearingCancelledOrConsumedRunRemovesPoseAndAlliance() {
    AutoPoseHandoff.save(new Waypoint(-61, 48, 180), Alliance.RED);
    AutoPoseHandoff.clear();
    assertFalse(AutoPoseHandoff.hasPose());
    assertNull(AutoPoseHandoff.getPose()); assertNull(AutoPoseHandoff.getAlliance());
  }

  @Test public void invalidNewPoseCannotRetainPreviousAutonomousPose() {
    for (Waypoint invalid : new Waypoint[] {null, new Waypoint(Double.NaN, 0, 0),
        new Waypoint(0, Double.POSITIVE_INFINITY, 0), new Waypoint(0, 0, Double.NaN)}) {
      AutoPoseHandoff.save(new Waypoint(-61, 48, 180), Alliance.RED);
      AutoPoseHandoff.save(invalid, Alliance.BLUE);
      assertFalse(AutoPoseHandoff.hasPose());
    }
    AutoPoseHandoff.save(new Waypoint(-61, 48, 180), Alliance.RED);
    AutoPoseHandoff.save(new Waypoint(0, 0, 0), null);
    assertFalse(AutoPoseHandoff.hasPose());
  }
}
