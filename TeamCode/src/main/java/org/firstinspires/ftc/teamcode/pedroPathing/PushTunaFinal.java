package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

public class PushTunaFinal {
    public enum State {
        DRIVING,
        WAITING_FOR_STILLNESS,
        ARMED,
        RECOVERING
    }

    private final Follower follower;
    private final GoBildaPinpointDriver pinpoint;

    private State state = State.DRIVING;

    private Pose holdTarget;
    private Pose previousPose;

    private long stillSince;
    private long recoveryStarted;

    // Tune these values on your robot
    private static final double INPUT_DEADBAND = 0.05;

    private static final double STILLNESS_STEP = 0.04;
    private static final long STILLNESS_TIME = 350;

    private static final double BUMP_THRESHOLD = 1.5;
    private static final double HEADING_THRESHOLD = Math.toRadians(8);

    private static final double POSITION_TOLERANCE = 0.6;
    private static final double HEADING_TOLERANCE = Math.toRadians(4);

    private static final long RECOVERY_TIMEOUT = 2000;

    public PushTunaFinal(Follower follower) {
        this.follower = follower;

        pinpoint = ((PinpointLocalizer)
                follower.getPoseTracker().getLocalizer()).getPinpoint();
    }

    public void update(double drive, double strafe, double rotate) {

        boolean neutral =
                Math.abs(drive) < INPUT_DEADBAND &&
                        Math.abs(strafe) < INPUT_DEADBAND &&
                        Math.abs(rotate) < INPUT_DEADBAND;

        // Driver always overrides automatic recovery
        if (state == State.RECOVERING && !neutral) {
            cancelRecovery();
        }

        // RECOVERY MODE
        if (state == State.RECOVERING) {

            follower.updatePose();

            if (!sensorReady()) {
                cancelRecovery();
                return;
            }

            follower.update();

            if (reachedTarget()) {
                finishRecovery();
            } else if (System.currentTimeMillis() - recoveryStarted
                    > RECOVERY_TIMEOUT) {
                cancelRecovery();
            }

            return;
        }

        // NORMAL TELEOP DRIVE
        follower.setTeleOpDrive(drive, -strafe, -rotate, true);
        follower.update();

        Pose current = follower.getPose();

        // Driver is actively moving
        if (!neutral) {
            state = State.DRIVING;
            previousPose = null;
            holdTarget = null;
            return;
        }

        // Begin checking for stillness
        if (state == State.DRIVING) {
            state = State.WAITING_FOR_STILLNESS;
            stillSince = System.currentTimeMillis();
            previousPose = current;
            return;
        }

        // Wait until robot has actually stopped moving
        if (state == State.WAITING_FOR_STILLNESS) {

            double movement = distance(current, previousPose);

            if (movement > STILLNESS_STEP) {
                stillSince = System.currentTimeMillis();
            }

            previousPose = current;

            if (System.currentTimeMillis() - stillSince
                    >= STILLNESS_TIME) {

                holdTarget = copyPose(current);
                state = State.ARMED;
            }

            return;
        }

        // MONITOR FOR UNEXPECTED DISPLACEMENT
        if (state == State.ARMED) {

            double displacement = distance(current, holdTarget);
            double headingError = headingDifference(
                    current.getHeading(),
                    holdTarget.getHeading()
            );

            if (displacement >= BUMP_THRESHOLD ||
                    headingError >= HEADING_THRESHOLD) {

                if (sensorReady()) {
                    startRecovery();
                }
            }
        }
    }

    private void startRecovery() {

        follower.breakFollowing();

        follower.deactivateAllPIDFs();
        follower.activateTranslational();
        follower.activateHeading();

        follower.holdPoint(holdTarget, false);

        recoveryStarted = System.currentTimeMillis();
        state = State.RECOVERING;
    }

    private boolean reachedTarget() {

        Pose current = follower.getPose();

        return distance(current, holdTarget) <= POSITION_TOLERANCE
                && headingDifference(
                current.getHeading(),
                holdTarget.getHeading()
        ) <= HEADING_TOLERANCE;
    }

    private void finishRecovery() {

        follower.breakFollowing();
        follower.startTeleopDrive(true);

        state = State.ARMED;
    }

    private void cancelRecovery() {

        follower.breakFollowing();
        follower.startTeleopDrive(true);

        state = State.DRIVING;
        holdTarget = null;
        previousPose = null;
    }

    private boolean sensorReady() {

        Pose p = follower.getPose();

        return pinpoint.getDeviceStatus()
                == GoBildaPinpointDriver.DeviceStatus.READY
                && Double.isFinite(p.getX())
                && Double.isFinite(p.getY())
                && Double.isFinite(p.getHeading());
    }

    private double distance(Pose a, Pose b) {

        return Math.hypot(
                a.getX() - b.getX(),
                a.getY() - b.getY()
        );
    }

    private double headingDifference(double a, double b) {

        double difference = a - b;

        return Math.abs(Math.atan2(
                Math.sin(difference),
                Math.cos(difference)
        ));
    }

    private Pose copyPose(Pose p) {
        return new Pose(p.getX(), p.getY(), p.getHeading());
    }

    public State getState() {
        return state;
    }

    public boolean isRecovering() {
        return state == State.RECOVERING;
    }
}
