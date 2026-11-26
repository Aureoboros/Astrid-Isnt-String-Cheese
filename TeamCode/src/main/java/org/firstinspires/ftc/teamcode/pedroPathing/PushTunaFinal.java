package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

/** Arms after driver input and motion stop; restores the held pose after an unexpected bump. */
public class PushTunaFinal {
    public enum State { DRIVING, WAITING_FOR_STILLNESS, ARMED, RECOVERING }

    // Small hardware boundary lets unit tests exercise the same recovery state machine.
    interface DriveIO {
        void updatePose();
        Pose getPose();
        boolean sensorReady();
        void drive(double forward, double left, double counterclockwise);
        void update();
        void hold(Pose target);
        void returnToDriver();
    }

    private final DriveIO io;
    private State state = State.DRIVING;
    private Pose holdTarget, previousPose;
    private long stillSince, recoveryStarted;
    private static final double INPUT_DEADBAND = 0.05;
    private static final double STILLNESS_STEP = 0.04;
    private static final double STILLNESS_HEADING_STEP = Math.toRadians(0.5);
    private static final long STILLNESS_TIME = 350;
    private static final double BUMP_THRESHOLD = 1.5;
    private static final double HEADING_THRESHOLD = Math.toRadians(8);
    private static final double POSITION_TOLERANCE = 0.6;
    private static final double HEADING_TOLERANCE = Math.toRadians(4);
    private static final long RECOVERY_TIMEOUT = 2000;

    public PushTunaFinal(Follower follower) {
        this(new DriveIO() {
            private final GoBildaPinpointDriver pinpoint =
                    ((PinpointLocalizer) follower.getPoseTracker().getLocalizer()).getPinpoint();
            public void updatePose() { follower.updatePose(); }
            public Pose getPose() { return follower.getPose(); }
            public boolean sensorReady() {
                return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
            }
            public void drive(double forward, double left, double counterclockwise) {
                follower.setTeleOpDrive(forward, left, counterclockwise, true);
            }
            public void update() { follower.update(); }
            public void hold(Pose target) {
                follower.breakFollowing();
                // holdPoint selects Pedro's hold mode; leave the tuned PID enable flags intact.
                follower.holdPoint(target, false);
            }
            public void returnToDriver() {
                follower.breakFollowing();
                follower.startTeleopDrive(true);
            }
        });
    }

    PushTunaFinal(DriveIO io) { this.io = io; }

    /** Input is already in Pedro's frame: forward, left strafe, counterclockwise rotation. */
    public void update(double forward, double left, double counterclockwise) {
        update(forward, left, counterclockwise, System.nanoTime() / 1_000_000);
    }

    void update(double forward, double left, double counterclockwise, long now) {
        io.updatePose();
        Pose current = io.getPose();
        if (!io.sensorReady() || !finite(current) || !Double.isFinite(forward)
                || !Double.isFinite(left) || !Double.isFinite(counterclockwise)) {
            cancelRecovery(); // Stop motors and clear the stale hold target before accepting input again.
            return;
        }
        boolean neutral = Math.abs(forward) < INPUT_DEADBAND
                && Math.abs(left) < INPUT_DEADBAND
                && Math.abs(counterclockwise) < INPUT_DEADBAND;
        if (state == State.RECOVERING && !neutral) { cancelRecovery(); }
        if (state == State.RECOVERING) {
            if (now - recoveryStarted >= RECOVERY_TIMEOUT) {
                cancelRecovery();
                return;
            }
            io.update();
            current = io.getPose();
            if (!io.sensorReady() || !finite(current)) { cancelRecovery(); return; }
            if (distance(current, holdTarget) <= POSITION_TOLERANCE
                    && headingDifference(current.getHeading(), holdTarget.getHeading()) <= HEADING_TOLERANCE) {
                io.returnToDriver();
                state = State.ARMED;
            }
            return;
        }

        io.drive(forward, left, counterclockwise);
        io.update();
        current = io.getPose();
        if (!io.sensorReady() || !finite(current)) { cancelRecovery(); return; }
        if (!neutral) {
            state = State.DRIVING;
            previousPose = null;
            holdTarget = null;
            return;
        }
        if (state == State.DRIVING) {
            state = State.WAITING_FOR_STILLNESS;
            stillSince = now;
            previousPose = copyPose(current);
        } else if (state == State.WAITING_FOR_STILLNESS) {
            if (distance(current, previousPose) > STILLNESS_STEP
                    || headingDifference(current.getHeading(), previousPose.getHeading()) > STILLNESS_HEADING_STEP) {
                stillSince = now;
            }
            previousPose = copyPose(current);
            if (now - stillSince >= STILLNESS_TIME) {
                holdTarget = copyPose(current);
                state = State.ARMED;
            }
        } else if (state == State.ARMED
                && (distance(current, holdTarget) >= BUMP_THRESHOLD
                || headingDifference(current.getHeading(), holdTarget.getHeading()) >= HEADING_THRESHOLD)) {
            io.hold(copyPose(holdTarget));
            recoveryStarted = now;
            state = State.RECOVERING;
        }
    }

    private void cancelRecovery() {
        io.returnToDriver();
        state = State.DRIVING;
        holdTarget = null;
        previousPose = null;
    }
    private static boolean finite(Pose p) {
        return p != null && Double.isFinite(p.getX()) && Double.isFinite(p.getY())
                && Double.isFinite(p.getHeading());
    }
    private static double distance(Pose a, Pose b) {
        return Math.hypot(a.getX() - b.getX(), a.getY() - b.getY());
    }
    private static double headingDifference(double a, double b) {
        return Math.abs(Math.atan2(Math.sin(a - b), Math.cos(a - b)));
    }
    private static Pose copyPose(Pose p) { return new Pose(p.getX(), p.getY(), p.getHeading()); }
    /** Forget any held pose, e.g. after an automatic move put the robot somewhere new. */
    public void reset() { cancelRecovery(); }
    public State getState() { return state; }
    public boolean isRecovering() { return state == State.RECOVERING; }
}
