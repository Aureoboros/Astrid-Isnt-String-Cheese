/**
package org.firstinspires.ftc.teamcode.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.AutomaticNectarSeparator;

public final class NectarSeparatorTelemetry {
  private NectarSeparatorTelemetry() {}
  public static void add(Telemetry telemetry, AutomaticNectarSeparator gate) {
    telemetry.addData("Nectar separator", gate.getStatus());
    telemetry.addData("Gate completed indexes", "%d (estimated releases, no ball sensor)", gate.getCompletedIndexes());
    if (gate.isCalibrated()) {
      telemetry.addData("Gate encoder / target", "%d / %d ticks", gate.getPositionTicks(), gate.getTargetTicks());
    }
  }
}
*/
