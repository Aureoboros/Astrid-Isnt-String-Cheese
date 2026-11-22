/**
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.config.PreloadedHiveAutoConfig;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.Alliance;
import org.firstinspires.ftc.teamcode.navigation.HiveAutoRoute;
import org.firstinspires.ftc.teamcode.navigation.PreloadedHiveAutoRoute;

/** Red wall start; three four-nectar batches and two alliance-flower pickups, then park. */
/**
@Autonomous(name = "BIOBUZZ: Red 3 Batches + Park", group = "Competition")
public final class RedPreloadedHiveAuto extends BiobuzzHiveAuto {
  @Override protected Alliance initialAlliance() { return Alliance.RED; }
  @Override protected boolean allowAllianceSelection() { return false; }
  @Override protected boolean requiresPlacementConfirmation() { return false; }
  @Override protected HiveAutoRoute createRoute(Alliance alliance) { return new PreloadedHiveAutoRoute(alliance); }
  @Override protected boolean canRunRoute() { return PreloadedHiveAutoConfig.canRun(); }
  @Override protected boolean usesSingleShooter() { return true; }
  @Override protected double autoBudgetSeconds() { return PreloadedHiveAutoConfig.autoBudgetSeconds(); }
  @Override protected double legTimeoutSeconds() { return PreloadedHiveAutoConfig.legTimeoutSeconds(); }
  @Override protected double positionToleranceInches() { return PreloadedHiveAutoConfig.positionToleranceInches(); }
  @Override protected double pickupPositionToleranceInches() { return PreloadedHiveAutoConfig.PICKUP_POSITION_TOLERANCE_INCHES; }
}
*/