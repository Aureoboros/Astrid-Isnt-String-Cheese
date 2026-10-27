package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

/** Standalone X/Y/heading push-resistance test; independent of the Tuning menu's static state. */
@TeleOp(name = "Tuna", group = "Pedro Pathing")
public final class XYZPushTuna extends OpMode {
    private static final double MAX_POWER = 0.25;
    private static final double ROBOT_RADIUS = 9;
    private static final Style ROBOT_STYLE = new Style("", "#3F51B5", 0.75);
    private static final Style HISTORY_STYLE = new Style("", "#4CAF50", 0.75);
    private Follower follower;
    private GoBildaPinpointDriver pinpoint;
    private TelemetryManager panelsTelemetry;
    private FieldManager panelsField;
    private PoseHistory poseHistory;
    private Pose holdTarget;
    private boolean holding;
    private String status = "INIT: keep robot still during Pinpoint calibration";

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(50);
        follower = Constants.createFollower(hardwareMap);
        pinpoint = ((PinpointLocalizer) follower.getPoseTracker().getLocalizer()).getPinpoint();
        follower.setMaxPower(MAX_POWER);
        follower.setStartingPose(new Pose(72, 72, 0));
        poseHistory = follower.getPoseHistory();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        panelsField = PanelsField.INSTANCE.getField();
        if (panelsField != null) {
            panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        }
        showTelemetry();
    }

    @Override
    public void init_loop() {
        // Sensors only: INIT must not command a position hold or move the robot.
        follower.updatePose();
        status = sensorReady() ? "INIT: READY; Play captures the current pose"
                : "INIT: waiting for Pinpoint READY / finite pose";
        showTelemetry();
    }

    @Override
    public void start() {
        follower.updatePose();
        if (!sensorReady()) {
            abort("STOPPED: Pinpoint was not ready at Play; reinitialize to retry");
            showTelemetry();
            return;
        }
        // Capture at Play to avoid driving back to an old INIT pose after manual movement.
        Pose current = follower.getPose();
        holdTarget = new Pose(current.getX(), current.getY(), current.getHeading());
        follower.deactivateAllPIDFs();
        follower.activateTranslational();
        follower.activateHeading();
        follower.holdPoint(holdTarget, false);
        holding = true;
        status = "HOLDING: gently displace X/Y or turn; robot returns to captured pose";
    }

    @Override
    public void loop() {
        try {
            follower.updatePose();
            if (holding) {
                if (gamepad1.b || gamepad2.b) {
                    abort("STOPPED by B; reinitialize to restart holding");
                } else if (!sensorReady()) {
                    abort("STOPPED: Pinpoint fault / invalid pose; reinitialize to retry");
                } else {
                    follower.update();
                    if (!sensorReady()) {
                        abort("STOPPED: Pinpoint fault / invalid pose; reinitialize to retry");
                    }
                }
            }
            showTelemetry();
        } catch (RuntimeException e) {
            abort("STOPPED after runtime error");
            throw e;
        }
    }

    @Override
    public void stop() { abort("STOPPED"); }

    private void abort(String reason) {
        holding = false;
        status = reason;
        if (follower != null) { follower.breakFollowing(); }
    }

    private boolean sensorReady() {
        Pose p = follower.getPose();
        return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY
                && Double.isFinite(p.getX()) && Double.isFinite(p.getY()) && Double.isFinite(p.getHeading());
    }

    private void showTelemetry() {
        Pose p = follower.getPose();
        telemetry.addData("Status", status);
        telemetry.addLine("Play captures hold pose | B: stop hold | Motor power limited to 25%");
        telemetry.addLine("Tune shared Constants.followerConstants translational / heading PIDF.");
        telemetry.addData("Pose Source", "Pinpoint: two pods + onboard IMU; local tuning origin (72,72)");
        telemetry.addData("Pose", "X %.2f, Y %.2f in; heading %.1f deg",
                p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        telemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus());
        telemetry.addData("Raw Odo X / Y Ticks", "%d / %d", pinpoint.getEncoderX(), pinpoint.getEncoderY());
        if (panelsTelemetry != null) { panelsTelemetry.addData("Status", status); }
        if (holdTarget != null) {
            double angle = holdTarget.getHeading() - p.getHeading();
            double headingError = Math.toDegrees(Math.atan2(Math.sin(angle), Math.cos(angle)));
            telemetry.addData("Hold Target", "X %.2f, Y %.2f in; heading %.1f deg",
                    holdTarget.getX(), holdTarget.getY(), Math.toDegrees(holdTarget.getHeading()));
            telemetry.addData("Error X / Y", "%.2f / %.2f in",
                    holdTarget.getX() - p.getX(), holdTarget.getY() - p.getY());
            telemetry.addData("Heading Error", "%.2f deg", headingError);
            if (panelsTelemetry != null) {
                panelsTelemetry.addData("Zero Line", 0);
                panelsTelemetry.addData("Error X (in)", holdTarget.getX() - p.getX());
                panelsTelemetry.addData("Error Y (in)", holdTarget.getY() - p.getY());
                panelsTelemetry.addData("Heading Error (deg)", headingError);
            }
        }
        OdometryTelemetry.add(telemetry);
        if (panelsTelemetry != null) {
            OdometryTelemetry.add(panelsTelemetry);
            panelsTelemetry.update(telemetry);
        } else { telemetry.update(); }
        drawOwnHistory(p);
    }

    private void drawOwnHistory(Pose current) {
        if (panelsField == null || poseHistory == null || !Double.isFinite(current.getX())
                || !Double.isFinite(current.getY()) || !Double.isFinite(current.getHeading())) { return; }
        double[] x = poseHistory.getXPositionsArray(), y = poseHistory.getYPositionsArray();
        panelsField.setStyle(HISTORY_STYLE);
        for (int i = 0; i + 1 < x.length; i++) {
            if (Double.isFinite(x[i]) && Double.isFinite(y[i])
                    && Double.isFinite(x[i + 1]) && Double.isFinite(y[i + 1])) {
                panelsField.moveCursor(x[i], y[i]);
                panelsField.line(x[i + 1], y[i + 1]);
            }
        }
        panelsField.setStyle(ROBOT_STYLE);
        panelsField.moveCursor(current.getX(), current.getY());
        panelsField.circle(ROBOT_RADIUS);
        panelsField.moveCursor(current.getX(), current.getY());
        panelsField.line(current.getX() + ROBOT_RADIUS * Math.cos(current.getHeading()),
                current.getY() + ROBOT_RADIUS * Math.sin(current.getHeading()));
        panelsField.update();
    }
}
