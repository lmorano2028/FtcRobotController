package org.firstinspires.ftc.teamcode.pedroPathing.RedSide;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.FieldTransform;
@Disabled

@Autonomous(name = "RedAutoCloseRange12Ball_PATH_ONLY", group = "Red")
public class RedAutoCloseRange12Ball_PathOnly extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    // ==========================================================
    // PATH STATES (DRIVE-ONLY)
    // ==========================================================
    public enum PathState {
        DRIVE_START_TO_SHOT,

        DRIVE_SHOT_TO_RELOAD1_START,
        DRIVE_RELOAD1_START_TO_END,
        DRIVE_RELOAD1_END_TO_SHOT,

        DRIVE_SHOT_TO_RELOAD2_START,
        DRIVE_RELOAD2_START_TO_END,
        DRIVE_RELOAD2_END_TO_SHOT,

        DRIVE_SHOT_TO_GATERELOAD_START,
        WAIT_GATEPOS,
        DRIVE_GATERELOAD_START_TO_RELOAD,
        WAIT_GATERELOAD,

        DRIVE_GATERELOAD_TO_SHOT,

        DRIVE_SHOT_TO_RELOAD3_START,
        DRIVE_RELOAD3_START_TO_END,
        DRIVE_RELOAD3_END_TO_FINALSHOT,

        DONE_HOLD
    }

    private PathState pathState;

    // ==========================================================
    // POSES (same as your original)
    // ==========================================================
    private final Pose startPose = new Pose(123.429, 122.849, Math.toRadians(42));
    private final Pose shotPose  = new Pose(96.113,   95.686, Math.toRadians(0));

    private final Pose reload1Start = new Pose(95.000, 89.000, Math.toRadians(0));
    private final Pose reload1End   = new Pose(122.000, 89.000, Math.toRadians(0));

    private final Pose reload2Start = new Pose(95.000, 66.500, Math.toRadians(0));
    private final Pose reload2End   = new Pose(119.000, 66.500, Math.toRadians(0));

    // Gate reload points (VARIABLES)
    private final Pose gateReloadStartPose   = new Pose(126.0, 66.75, Math.toRadians(0));
    private final Pose gateReloadControlPose = new Pose(122.000, 57.000, Math.toRadians(0)); // control point; heading not used
    private final Pose gateReloadPose        = new Pose(127.500, 56.00, Math.toRadians(42.5));
    private final Pose reload3Start = new Pose(95.000, 42.500, Math.toRadians(0));
    private final Pose reload3End   = new Pose(125.000, 42.500, Math.toRadians(0));

    private final Pose finalShotPose = new Pose(89.000, 104.000, Math.toRadians(90));

    // ==========================================================
    // PATHS
    // ==========================================================
    private PathChain startToShot;

    private PathChain shotToReload1Start;
    private PathChain reload1StartToEnd;
    private PathChain reload1EndToShot;

    private PathChain shotToReload2Start;
    private PathChain reload2StartToEnd;
    private PathChain reload2EndToShot;

    private PathChain shotToGateReloadStart;
    private PathChain gateReloadStartToReload;
    private PathChain gateReloadToShot;

    private PathChain shotToReload3Start;
    private PathChain reload3StartToEnd;
    private PathChain reload3EndToFinalShot;

    // ==========================================================
    // Drive-state guards (prevents “didn't drive”)
    // ==========================================================
    private boolean driveSawBusy = false;
    private boolean driveStartedThisState = false;

    // Early exit tolerance
    private static final double END_TOL_IN = 2.0;

    // Waits (kept from your original intent)
    private static final double GATE_POS_WAIT_SEC    = 0.35;
    private static final double GATE_RELOAD_WAIT_SEC = 1.750;

    private final com.qualcomm.robotcore.util.ElapsedTime gatePosWaitTimer =
            new com.qualcomm.robotcore.util.ElapsedTime();
    private final com.qualcomm.robotcore.util.ElapsedTime gateReloadWaitTimer =
            new com.qualcomm.robotcore.util.ElapsedTime();

    // ==========================================================
    // BUILD PATHS (same geometry as your original)
    // ==========================================================
    public void buildPaths() {

        startToShot = follower.pathBuilder()
                .addPath(new BezierLine(
                        startPose,
                        shotPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(0))
                .build();

        shotToReload1Start = follower.pathBuilder()
                .addPath(new BezierLine(
                        shotPose,
                        reload1Start
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        reload1StartToEnd = follower.pathBuilder()
                .addPath(new BezierLine(
                        reload1Start,
                        reload1End
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        reload1EndToShot = follower.pathBuilder()
                .addPath(new BezierLine(
                        reload1End,
                        shotPose
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        shotToReload2Start = follower.pathBuilder()
                .addPath(new BezierLine(
                        shotPose,
                        reload2Start
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        reload2StartToEnd = follower.pathBuilder()
                .addPath(new BezierLine(
                        reload2Start,
                        reload2End
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        reload2EndToShot = follower.pathBuilder()
                .addPath(new BezierLine(
                        reload2End,
                        shotPose
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // shot -> gateReloadStartPose
        shotToGateReloadStart = follower.pathBuilder()
                .addPath(new BezierLine(
                        shotPose,
                        gateReloadStartPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // gateReloadStartPose -> gateReloadPose (bezier curve w/ control)
        gateReloadStartToReload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        gateReloadStartPose,
                        gateReloadControlPose,
                        gateReloadPose
                ))
                .setLinearHeadingInterpolation(
                        gateReloadStartPose.getHeading(),
                        gateReloadPose.getHeading()
                )
                .build();

        // gateReloadPose -> shot
        gateReloadToShot = follower.pathBuilder()
                .addPath(new BezierLine(
                        gateReloadPose,
                        shotPose
                ))
                .setLinearHeadingInterpolation(gateReloadPose.getHeading(), Math.toRadians(0))
                .build();

        shotToReload3Start = follower.pathBuilder()
                .addPath(new BezierLine(
                        shotPose,
                        reload3Start
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        reload3StartToEnd = follower.pathBuilder()
                .addPath(new BezierLine(
                        reload3Start,
                        reload3End
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        reload3EndToFinalShot = follower.pathBuilder()
                .addPath(new BezierLine(
                        reload3End,
                        finalShotPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();
    }

    // ==========================================================
    // STATE MACHINE (DRIVE-ONLY)
    // ==========================================================
    public void statePathUpdate() {
        switch (pathState) {

            case DRIVE_START_TO_SHOT:
                driveOnly(startToShot, shotPose, PathState.DRIVE_SHOT_TO_RELOAD1_START);
                break;

            case DRIVE_SHOT_TO_RELOAD1_START:
                driveOnly(shotToReload1Start, reload1Start, PathState.DRIVE_RELOAD1_START_TO_END);
                break;

            case DRIVE_RELOAD1_START_TO_END:
                driveOnly(reload1StartToEnd, reload1End, PathState.DRIVE_RELOAD1_END_TO_SHOT);
                break;

            case DRIVE_RELOAD1_END_TO_SHOT:
                driveOnly(reload1EndToShot, shotPose, PathState.DRIVE_SHOT_TO_RELOAD2_START);
                break;

            case DRIVE_SHOT_TO_RELOAD2_START:
                driveOnly(shotToReload2Start, reload2Start, PathState.DRIVE_RELOAD2_START_TO_END);
                break;

            case DRIVE_RELOAD2_START_TO_END:
                driveOnly(reload2StartToEnd, reload2End, PathState.DRIVE_RELOAD2_END_TO_SHOT);
                break;

            case DRIVE_RELOAD2_END_TO_SHOT:
                driveOnly(reload2EndToShot, shotPose, PathState.DRIVE_SHOT_TO_GATERELOAD_START);
                break;

            case DRIVE_SHOT_TO_GATERELOAD_START:
                driveOnly(shotToGateReloadStart, gateReloadStartPose, PathState.WAIT_GATEPOS);
                break;

            case WAIT_GATEPOS:
                if (gatePosWaitTimer.seconds() >= GATE_POS_WAIT_SEC) {
                    setPathState(PathState.DRIVE_GATERELOAD_START_TO_RELOAD);
                }
                break;

            case DRIVE_GATERELOAD_START_TO_RELOAD:
                driveOnly(gateReloadStartToReload, gateReloadPose, PathState.WAIT_GATERELOAD);
                break;

            case WAIT_GATERELOAD:
                if (gateReloadWaitTimer.seconds() >= GATE_RELOAD_WAIT_SEC) {
                    setPathState(PathState.DRIVE_GATERELOAD_TO_SHOT);
                }
                break;

            case DRIVE_GATERELOAD_TO_SHOT:
                driveOnly(gateReloadToShot, shotPose, PathState.DRIVE_SHOT_TO_RELOAD3_START);
                break;

            case DRIVE_SHOT_TO_RELOAD3_START:
                driveOnly(shotToReload3Start, reload3Start, PathState.DRIVE_RELOAD3_START_TO_END);
                break;

            case DRIVE_RELOAD3_START_TO_END:
                driveOnly(reload3StartToEnd, reload3End, PathState.DRIVE_RELOAD3_END_TO_FINALSHOT);
                break;

            case DRIVE_RELOAD3_END_TO_FINALSHOT:
                driveOnly(reload3EndToFinalShot, finalShotPose, PathState.DONE_HOLD);
                break;

            case DONE_HOLD:
            default:
                // Just hold here; still keep PoseStorage updated.
                break;
        }
    }

    // ==========================================================
    // DRIVE helper (start once, require “saw busy” before completing)
    // ==========================================================
    private void driveOnly(PathChain path, Pose endPose, PathState next) {

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

        if (newState == PathState.WAIT_GATEPOS) {
            gatePosWaitTimer.reset();
        }
        if (newState == PathState.WAIT_GATERELOAD) {
            gateReloadWaitTimer.reset();
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

        gatePosWaitTimer.reset();
        gateReloadWaitTimer.reset();

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

        // Keep PoseStorage updated like your original (helpful for next opmodes)
        Pose p = follower.getPose();
        FieldTransform.writePoseStorageFromPedro(p.getX(), p.getY(), Math.toDegrees(p.getHeading()));

        statePathUpdate();

        telemetry.addData("state", pathState.toString());
        telemetry.addData("isBusy", follower.isBusy());
        telemetry.addData("driveStartedThisState", driveStartedThisState);
        telemetry.addData("driveSawBusy", driveSawBusy);

        telemetry.addData("gatePosWaitT(s)", "%.2f", gatePosWaitTimer.seconds());
        telemetry.addData("gatePosWaitGoal(s)", "%.2f", GATE_POS_WAIT_SEC);

        telemetry.addData("gateReloadWaitT(s)", "%.2f", gateReloadWaitTimer.seconds());
        telemetry.addData("gateReloadWaitGoal(s)", "%.2f", GATE_RELOAD_WAIT_SEC);

        telemetry.addData("x", p.getX());
        telemetry.addData("y", p.getY());
        telemetry.addData("heading(rad)", p.getHeading());

        telemetry.update();
    }

    @Override
    public void stop() {
        // Nothing to stop besides writing pose storage one last time
        Pose p = follower.getPose();
        FieldTransform.writePoseStorageFromPedro(p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
    }

    // ==========================================================
    // Small helpers
    // ==========================================================
    private static double dist(Pose a, Pose b) {
        double dx = a.getX() - b.getX();
        double dy = a.getY() - b.getY();
        return Math.hypot(dx, dy);
    }
}
