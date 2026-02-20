package org.firstinspires.ftc.teamcode.pedroPathing.BlueSide;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.FieldTransform;

@Autonomous(name = "BlueAuto_PathOnly_15Ball_gateReload", group = "Blue")
public class BlueAutoCloseRange15Ball_PathOnly extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    // ==========================================================
    // Drive-state guards (prevents “didn't drive”)
    // ==========================================================
    private boolean driveSawBusy = false;
    private boolean driveStartedThisState = false;

    // Early exit tolerance
    private static final double END_TOL_IN = 2.0;

    // Gate position wait after arriving at gate start pose
    private static final double GATE_POS_WAIT_SEC = 0.15;
    private final com.qualcomm.robotcore.util.ElapsedTime gatePosWaitTimer =
            new com.qualcomm.robotcore.util.ElapsedTime();

    // Gate reload wait
    private static final double GATE_RELOAD_WAIT_SEC = 1.10;
    private final com.qualcomm.robotcore.util.ElapsedTime gateReloadWaitTimer =
            new com.qualcomm.robotcore.util.ElapsedTime();

    // ==========================================================
    // PATH STATES (same order as your blue auto)
    // ==========================================================
    public enum PathState {
        DRIVE_START_TO_SHOT,
        WAIT_AT_SHOT_1,

        // shot -> reload2 -> 2nd shot
        DRIVE_SHOT_TO_RELOAD2_START,
        DRIVE_RELOAD2_START_TO_END,
        DRIVE_RELOAD2_END_TO_SHOT2,
        WAIT_AT_SHOT_2,

        // shot2 -> gate -> shot2
        DRIVE_SHOT2_TO_GATERELOAD_START,
        WAIT_GATEPOS_A,
        DRIVE_GATERELOAD_START_TO_RELOAD_A,
        WAIT_GATERELOAD_A,
        DRIVE_GATERELOAD_TO_SHOT_A,
        WAIT_AT_SHOT_3,

        // shot2 -> reload1 -> shot
        DRIVE_SHOT2_TO_RELOAD1_START,
        DRIVE_RELOAD1_START_TO_END,
        DRIVE_RELOAD1_END_TO_SHOT,
        WAIT_AT_SHOT_4,

        // shot -> gate -> final shot
        DRIVE_SHOT_TO_GATERELOAD_START_B,
        WAIT_GATEPOS_B,
        DRIVE_GATERELOAD_START_TO_RELOAD_B,
        WAIT_GATERELOAD_B,
        DRIVE_GATERELOAD_TO_FINALSHOT,
        HOLD_FINAL
    }

    private PathState pathState;

    // Small settle pause at shot points (purely for testing)
    private static final double SHOT_SETTLE_SEC = 0.20;
    private final com.qualcomm.robotcore.util.ElapsedTime settleTimer =
            new com.qualcomm.robotcore.util.ElapsedTime();

    // ==========================================================
    // POSES (BLUE)
    // ==========================================================
    private final Pose startPose = new Pose(20.571428571428577, 122.84909456740445, Math.toRadians(138));
    private final Pose shotPose  = new Pose(47.88732394366197,   95.68611670020124, Math.toRadians(180));

    // Mirrored from your red file: (89,89,0)->(55,89,180)
    private final Pose secondShotPose = new Pose(55.000, 89.000, Math.toRadians(180));

    private final Pose reload1Start = new Pose(44.000, 88.000, Math.toRadians(180));
    private final Pose reload1End   = new Pose(28.000, 88.000, Math.toRadians(180));

    private final Pose reload2Start = new Pose(50.000, 66.500, Math.toRadians(180));
    private final Pose reload2End   = new Pose(29.000, 66.500, Math.toRadians(180));

    // Gate reload points
    private final Pose gateReloadStartPose   = new Pose(21.250, 63.350, Math.toRadians(180));
    private final Pose gateReloadControlPose = new Pose(21.000, 57.000, Math.toRadians(155)); // control; heading not used
    private final Pose gateReloadPose        = new Pose(17.750, 53.000, Math.toRadians(132.5));

    // Mirrored control from red (85,65)->(59,65)
    private final Pose shot2ToGateControlPose = new Pose(50.0, 65.0, Math.toRadians(180)); // control; heading not used

    private final Pose finalShotPose = new Pose(55.000, 108.000, Math.toRadians(90));

    // ==========================================================
    // PATHS
    // ==========================================================
    private PathChain startToShot;

    private PathChain shotToReload2Start;
    private PathChain reload2StartToEnd;
    private PathChain reload2EndToShot2;

    private PathChain shot2ToGateReloadStart;
    private PathChain gateReloadStartToReload_A;
    private PathChain gateReloadToShot_A;

    private PathChain shot2ToReload1Start;
    private PathChain reload1StartToEnd;
    private PathChain reload1EndToShot;

    private PathChain shotToGateReloadStart_B;
    private PathChain gateReloadStartToReload_B;
    private PathChain gateReloadToFinalShot;

    // ==========================================================
    // BUILD PATHS
    // ==========================================================
    public void buildPaths() {

        startToShot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shotPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shotPose.getHeading())
                .build();

        shotToReload2Start = follower.pathBuilder()
                .addPath(new BezierLine(shotPose, reload2Start))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        reload2StartToEnd = follower.pathBuilder()
                .addPath(new BezierLine(reload2Start, reload2End))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        reload2EndToShot2 = follower.pathBuilder()
                .addPath(new BezierLine(reload2End, secondShotPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shot2ToGateReloadStart = follower.pathBuilder()
                .addPath(new BezierCurve(secondShotPose, shot2ToGateControlPose, gateReloadStartPose))
                .setLinearHeadingInterpolation(secondShotPose.getHeading(), gateReloadStartPose.getHeading())
                .build();

        gateReloadStartToReload_A = follower.pathBuilder()
                .addPath(new BezierCurve(gateReloadStartPose, gateReloadControlPose, gateReloadPose))
                .setLinearHeadingInterpolation(gateReloadStartPose.getHeading(), gateReloadPose.getHeading())
                .build();

        gateReloadToShot_A = follower.pathBuilder()
                .addPath(new BezierCurve(gateReloadPose, shot2ToGateControlPose, secondShotPose))
                .setLinearHeadingInterpolation(gateReloadPose.getHeading(), secondShotPose.getHeading())
                .build();

        shot2ToReload1Start = follower.pathBuilder()
                .addPath(new BezierLine(secondShotPose, reload1Start))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        reload1StartToEnd = follower.pathBuilder()
                .addPath(new BezierLine(reload1Start, reload1End))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        reload1EndToShot = follower.pathBuilder()
                .addPath(new BezierLine(reload1End, shotPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shotToGateReloadStart_B = follower.pathBuilder()
                .addPath(new BezierLine(shotPose, gateReloadStartPose))
                .setLinearHeadingInterpolation(shotPose.getHeading(), gateReloadStartPose.getHeading())
                .build();

        gateReloadStartToReload_B = follower.pathBuilder()
                .addPath(new BezierCurve(gateReloadStartPose, gateReloadControlPose, gateReloadPose))
                .setLinearHeadingInterpolation(gateReloadStartPose.getHeading(), gateReloadPose.getHeading())
                .build();

        gateReloadToFinalShot = follower.pathBuilder()
                .addPath(new BezierLine(gateReloadPose, finalShotPose))
                .setLinearHeadingInterpolation(gateReloadPose.getHeading(), finalShotPose.getHeading())
                .build();
    }

    // ==========================================================
    // STATE MACHINE
    // ==========================================================
    public void statePathUpdate() {
        switch (pathState) {

            case DRIVE_START_TO_SHOT:
                drivePath(startToShot, shotPose, PathState.WAIT_AT_SHOT_1);
                break;

            case WAIT_AT_SHOT_1:
                if (settleTimer.seconds() >= SHOT_SETTLE_SEC) setPathState(PathState.DRIVE_SHOT_TO_RELOAD2_START);
                break;

            case DRIVE_SHOT_TO_RELOAD2_START:
                drivePath(shotToReload2Start, reload2Start, PathState.DRIVE_RELOAD2_START_TO_END);
                break;

            case DRIVE_RELOAD2_START_TO_END:
                drivePath(reload2StartToEnd, reload2End, PathState.DRIVE_RELOAD2_END_TO_SHOT2);
                break;

            case DRIVE_RELOAD2_END_TO_SHOT2:
                drivePath(reload2EndToShot2, secondShotPose, PathState.WAIT_AT_SHOT_2);
                break;

            case WAIT_AT_SHOT_2:
                if (settleTimer.seconds() >= SHOT_SETTLE_SEC) setPathState(PathState.DRIVE_SHOT2_TO_GATERELOAD_START);
                break;

            case DRIVE_SHOT2_TO_GATERELOAD_START:
                drivePath(shot2ToGateReloadStart, gateReloadStartPose, PathState.WAIT_GATEPOS_A);
                break;

            case WAIT_GATEPOS_A:
                if (gatePosWaitTimer.seconds() >= GATE_POS_WAIT_SEC) setPathState(PathState.DRIVE_GATERELOAD_START_TO_RELOAD_A);
                break;

            case DRIVE_GATERELOAD_START_TO_RELOAD_A:
                drivePath(gateReloadStartToReload_A, gateReloadPose, PathState.WAIT_GATERELOAD_A);
                break;

            case WAIT_GATERELOAD_A:
                if (gateReloadWaitTimer.seconds() >= GATE_RELOAD_WAIT_SEC) setPathState(PathState.DRIVE_GATERELOAD_TO_SHOT_A);
                break;

            case DRIVE_GATERELOAD_TO_SHOT_A:
                drivePath(gateReloadToShot_A, secondShotPose, PathState.WAIT_AT_SHOT_3);
                break;

            case WAIT_AT_SHOT_3:
                if (settleTimer.seconds() >= SHOT_SETTLE_SEC) setPathState(PathState.DRIVE_SHOT2_TO_RELOAD1_START);
                break;

            case DRIVE_SHOT2_TO_RELOAD1_START:
                drivePath(shot2ToReload1Start, reload1Start, PathState.DRIVE_RELOAD1_START_TO_END);
                break;

            case DRIVE_RELOAD1_START_TO_END:
                drivePath(reload1StartToEnd, reload1End, PathState.DRIVE_RELOAD1_END_TO_SHOT);
                break;

            case DRIVE_RELOAD1_END_TO_SHOT:
                drivePath(reload1EndToShot, shotPose, PathState.WAIT_AT_SHOT_4);
                break;

            case WAIT_AT_SHOT_4:
                if (settleTimer.seconds() >= SHOT_SETTLE_SEC) setPathState(PathState.DRIVE_SHOT_TO_GATERELOAD_START_B);
                break;

            case DRIVE_SHOT_TO_GATERELOAD_START_B:
                drivePath(shotToGateReloadStart_B, gateReloadStartPose, PathState.WAIT_GATEPOS_B);
                break;

            case WAIT_GATEPOS_B:
                if (gatePosWaitTimer.seconds() >= GATE_POS_WAIT_SEC) setPathState(PathState.DRIVE_GATERELOAD_START_TO_RELOAD_B);
                break;

            case DRIVE_GATERELOAD_START_TO_RELOAD_B:
                drivePath(gateReloadStartToReload_B, gateReloadPose, PathState.WAIT_GATERELOAD_B);
                break;

            case WAIT_GATERELOAD_B:
                if (gateReloadWaitTimer.seconds() >= GATE_RELOAD_WAIT_SEC) setPathState(PathState.DRIVE_GATERELOAD_TO_FINALSHOT);
                break;

            case DRIVE_GATERELOAD_TO_FINALSHOT:
                drivePath(gateReloadToFinalShot, finalShotPose, PathState.HOLD_FINAL);
                break;

            case HOLD_FINAL:
            default:
                // Just hold pose; keep follower updated in loop()
                break;
        }
    }

    // ==========================================================
    // DRIVE helper (PATH ONLY)
    // ==========================================================
    private void drivePath(PathChain path, Pose endPose, PathState next) {

        if (!driveStartedThisState) {
            follower.followPath(path, true);
            driveStartedThisState = true;
        }

        if (follower.isBusy()) driveSawBusy = true;

        Pose cur = follower.getPose();
        boolean closeEnough = dist(cur, endPose) <= END_TOL_IN;

        if (driveSawBusy && (!follower.isBusy() || closeEnough)) {
            setPathState(next);
        }
    }

    // ==========================================================
    // STATE ENTRY
    // ==========================================================
    public void setPathState(PathState newState) {
        pathState = newState;

        pathTimer.resetTimer();

        driveSawBusy = false;
        driveStartedThisState = false;

        // Reset the appropriate timers for waits
        if (newState == PathState.WAIT_GATEPOS_A || newState == PathState.WAIT_GATEPOS_B) {
            gatePosWaitTimer.reset();
        }
        if (newState == PathState.WAIT_GATERELOAD_A || newState == PathState.WAIT_GATERELOAD_B) {
            gateReloadWaitTimer.reset();
        }
        if (newState == PathState.WAIT_AT_SHOT_1 || newState == PathState.WAIT_AT_SHOT_2
                || newState == PathState.WAIT_AT_SHOT_3 || newState == PathState.WAIT_AT_SHOT_4) {
            settleTimer.reset();
        }
    }

    // ==========================================================
    // INIT / START / LOOP / STOP
    // ==========================================================
    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(startPose);

        driveSawBusy = false;
        driveStartedThisState = false;

        gateReloadWaitTimer.reset();
        gatePosWaitTimer.reset();
        settleTimer.reset();

        pathState = PathState.DRIVE_START_TO_SHOT;

        telemetry.addLine("PATH ONLY ready. Waiting for start.");
        telemetry.update();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();

        // optional: keep pose storage updated for debugging/telemetry tools
        Pose p = follower.getPose();
        FieldTransform.writePoseStorageFromPedro(p.getX(), p.getY(), Math.toDegrees(p.getHeading()));

        statePathUpdate();

        telemetry.addData("state", pathState.toString());
        telemetry.addData("isBusy", follower.isBusy());
        telemetry.addData("driveStartedThisState", driveStartedThisState);
        telemetry.addData("driveSawBusy", driveSawBusy);

        telemetry.addData("x", "%.2f", follower.getPose().getX());
        telemetry.addData("y", "%.2f", follower.getPose().getY());
        telemetry.addData("headingDeg", "%.1f", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.addData("gatePosWaitT(s)", "%.2f", gatePosWaitTimer.seconds());
        telemetry.addData("gateReloadWaitT(s)", "%.2f", gateReloadWaitTimer.seconds());
        telemetry.addData("settleT(s)", "%.2f", settleTimer.seconds());

        telemetry.update();
    }

    @Override
    public void stop() {
        // nothing to stop (path-only)
    }

    // ==========================================================
    // helpers
    // ==========================================================
    private static double dist(Pose a, Pose b) {
        double dx = a.getX() - b.getX();
        double dy = a.getY() - b.getY();
        return Math.hypot(dx, dy);
    }
}