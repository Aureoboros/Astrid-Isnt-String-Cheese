package org.firstinspires.ftc.teamcode.opmodes;

import java.util.HashSet;
import java.util.Set;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.junit.Test;
import static org.junit.Assert.*;

public final class CompetitionRegistrationTest {
  @Test public void pulledAutosHaveDistinctEnabledDriverStationEntries() {
    Set<String> names = new HashSet<>();
    for (Class<?> auto : new Class<?>[] {BluePreloadedHiveAuto.class,
        RedPreloadedHiveAuto.class, EightJustEight.class, FortyEight.class}) {
      assertNull(auto.getAnnotation(Disabled.class));
      Autonomous annotation = auto.getAnnotation(Autonomous.class);
      assertNotNull(annotation);
      assertTrue("Duplicate Driver Station name: " + annotation.name(), names.add(annotation.name()));
    }
    assertTrue(names.contains("BIOBUZZ: Red 3 Batches + Park"));
    assertTrue(names.contains("BIOBUZZ: Blue 3 Batches + Park"));
  }
}
