package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.NectarSeparatorController;
import org.junit.Test;

public final class AutomaticNectarSeparatorTest {
  private int position, target;
  private double power;
  private DcMotor.RunMode mode;
  private DcMotor.Direction direction;
  private final List<String> commands = new ArrayList<>();
  private DcMotor motor() {
    return (DcMotor) Proxy.newProxyInstance(DcMotor.class.getClassLoader(), new Class<?>[] {DcMotor.class},
        (proxy, method, args) -> {
          switch (method.getName()) {
            case "setPower": power = (double) args[0]; break;
            case "setMode": mode = (DcMotor.RunMode) args[0]; break;
            case "getMode": return mode;
            case "getCurrentPosition": return position;
            case "setTargetPosition": target = (int) args[0]; break;
            case "setDirection": direction = (DcMotor.Direction) args[0]; break;
            case "setZeroPowerBehavior": break;
            default: throw new AssertionError("Unexpected gate access " + method.getName());
          }
          commands.add(method.getName()); return null;
        });
  }

  @Test public void uncalibratedConfigurationDoesNotEvenLookUpGateHardware() {
    assertFalse(RobotConfig.GateMotor.autoFeedReady());
    AutomaticNectarSeparator gate = new AutomaticNectarSeparator(null);
    gate.update(true, -4800, -4800); gate.close();
    assertFalse(gate.isCalibrated()); assertTrue(gate.getStatus().contains("BLOCKED"));
  }

  @Test public void indexedHardwareHoldsBlockFeedsOneAndStopsOnXOffAndClose() {
    position = 10;
    NectarSeparatorController controller = new NectarSeparatorController(100, 2, 150, 1000, 0.25, 0.1, 0.05, 1);
    AutomaticNectarSeparator gate = new AutomaticNectarSeparator(motor(), controller, 0.2);
    assertEquals(0, power, 0); // INIT never moves.
    assertEquals(RobotConfig.GateMotor.MOTOR_DIRECTION, direction);
    gate.update(true, -4800, -4800, 0);
    assertEquals(10, target); assertEquals(DcMotor.RunMode.RUN_TO_POSITION, mode);
    gate.update(true, -4800, -4800, 0.3);
    assertEquals(110, target); assertEquals(0.2, power, 0);
    position = 50;
    gate.update(false, 0, -3000, 0.4);
    assertEquals(0, power, 0); assertTrue(gate.getStatus().contains("FAULT"));
    gate.close(); int count = commands.size();
    gate.update(true, -4800, -4800, 5);
    assertEquals(count, commands.size()); assertEquals(0, power, 0);
  }
}
