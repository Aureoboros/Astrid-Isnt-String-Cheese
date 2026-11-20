package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.config.PreloadedHiveAutoConfig;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.Alliance;
import org.firstinspires.ftc.teamcode.navigation.HiveAutoRoute;
import org.firstinspires.ftc.teamcode.navigation.PreloadedHiveAutoRoute;

/** Blue wall start; three four-nectar batches and two alliance-flower pickups, then park. */
@Autonomous(name = "BIOBUZZ: Blue 3 Batches + Park", group = "Competition")
public final class BluePreloadedHiveAuto extends BiobuzzHiveAuto {
  @Override protected Alliance initialAlliance() { return Alliance.BLUE; }
  @Override protected boolean allowAllianceSelection() { return false; }
  @Override protected boolean requiresPlacementConfirmation() { return false; }
  @Override protected HiveAutoRoute createRoute(Alliance alliance) { return new PreloadedHiveAutoRoute(alliance); }
  @Override protected boolean canRunRoute() { return PreloadedHiveAutoConfig.canRun(); }
  @Override protected boolean usesSingleShooter() { return true; }
}
