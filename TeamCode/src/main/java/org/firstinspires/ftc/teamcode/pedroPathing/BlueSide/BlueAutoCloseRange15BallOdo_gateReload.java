package org.firstinspires.ftc.teamcode.pedroPathing.BlueSide;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.FieldTransform;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.ShooterSubsystem;

import java.util.List;

@Autonomous(name = "B - 15pt Gate", group = "Blue")
public class BlueAutoCloseRange15BallOdo_gateReload extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private static final String LIMELIGHT_NAME = "limelight";
    private static final String SHOOTER_NAME   = "ShooterMotor";
    private static final String FLICKER_NAME   = "Outertake";
    private static final String INTAKE1_NAME   = "intakeOneMotor";
    private static final String INTAKE2_NAME   = "intakeTwoMotor";
    private static final String HOOD_NAME      = "Shooter hood";
    private static final String FLIPPER_NAME   = "fingler";

    // ======= TURRET =======
    private static final String TURRET_LEFT_NAME  = "LRotation";
    private static final String TURRET_RIGHT_NAME = "RRotation";
    private static final boolean MIRROR_RIGHT = false;

    private static final double TURRET_START_POS  = 0.54;
    private Servo turretLeft, turretRight;

    // turret settle gate (separate timer so it can't get stomped)
    private static final double TURRET_SETTLE_SEC = 0.15;
    private boolean turretSettleStarted = false;
    private double turretTargetPos = TURRET_START_POS;
    private final com.qualcomm.robotcore.util.ElapsedTime turretSettleTimer =
            new com.qualcomm.robotcore.util.ElapsedTime();

    // ==========================================================
    // ODOMETRY + VISION TRIM TURRET AIM (PEDRO POSE, BLUE)
    // ==========================================================
    // Per your instruction:
    //   Goal ID = 20
    //   Goal coordinates = (2, 140)
    private static final double BLUE_GOAL_PX = 4;
    private static final double BLUE_GOAL_PY = 140;
    private static final int BLUE_GOAL_TAG_ID = 20;

    private static final double TURRET_HOME    = 0.50;
    private static final double POS_PER_DEG_CW = 0.007643;   // tuned
    private static final double SERVO_MIN_SAFE = 0.10;
    private static final double SERVO_MAX_SAFE = 0.95;

    private static final double TURRET_FWD_OFFSET_IN  = -4.0;
    private static final double TURRET_LEFT_OFFSET_IN =  0.0;

    private static final double BEARING_DEADBAND_DEG = 0.75;
    private static final double SERVO_SLEW_PER_SEC   = 1.2;
    private static final double MAX_STEP_PER_LOOP    = 0.020;
    private static final double KP_DEG_TO_SERVO      = 0.0026;

    private static final double VISION_X_SIGN = +1.0;
    private static final double VISION_MIN_Z_IN = 12.0;
    private static final double VISION_MAX_Z_IN = 160.0;

    private static final double VISION_MAX_TRIM_DEG = 10.0;
    private static final double VISION_LPF_ALPHA = 0.35;

    private static final int VISION_HOLD_MS = 250;
    private static final int VISION_DECAY_MS = 900;

    private static final double VISION_MAX_ABS_TX_DEG = 8.0;

    private double lastTurretCmd = TURRET_HOME;
    private final com.qualcomm.robotcore.util.ElapsedTime loopTimer =
            new com.qualcomm.robotcore.util.ElapsedTime();

    private boolean visionHasGood = false;
    private double visionTrimDegCW = 0.0;
    private double visionTrimDegCW_raw = 0.0;
    private final com.qualcomm.robotcore.util.ElapsedTime visionLastGoodTimer =
            new com.qualcomm.robotcore.util.ElapsedTime();
    private final com.qualcomm.robotcore.util.ElapsedTime visionDecayTimer =
            new com.qualcomm.robotcore.util.ElapsedTime();

    private boolean tagSeen = false;
    private int tagId = -1;
    private double camX_in = Double.NaN;
    private double camZ_in = Double.NaN;
    private double camBearingDeg = Double.NaN;
    private double txDeg = Double.NaN;

    // ==========================================================
    // Pre-shoot readiness latch
    // ==========================================================
    private static final double READY_LATCH_HOLD_SEC = 0.50;
    private static final double PRECHECK_DIST_IN = 10.0;

    private boolean readyLatched = false;
    private final com.qualcomm.robotcore.util.ElapsedTime readyLatchTimer =
            new com.qualcomm.robotcore.util.ElapsedTime();

    // ==========================================================
    // "never deadlocks" gate when tag is missing
    // ==========================================================
    private static final double NO_TAG_START_TIMEOUT_SEC = 0.35;
    private final com.qualcomm.robotcore.util.ElapsedTime shootGateTimer =
            new com.qualcomm.robotcore.util.ElapsedTime();

    // ==========================================================
    // PoseStorage write
    // ==========================================================
    private void writePoseStorageNow() {
        if (follower == null) return;
        Pose p = follower.getPose();
        FieldTransform.writePoseStorageFromPedro(p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
    }

    // ==========================================================
    // DRIVING INTAKE
    // ==========================================================
    private static final double DRIVE_INTAKE1_PWR = -1.0;
    private static final double DRIVE_INTAKE2_PWR =  0.15;

    // ==========================================================
    // FAST THROUGHOUT FEEDER
    // ==========================================================
    private static final class ThroughputFeeder {
        private static final double INTAKE1_PWR = -1.0;
        private static final double INTAKE2_PWR =  1.0;
        private static final double FLICKER_PWR =  1.0;

        private static final double FLIP_DOWN = 0.662;
        private static final double FLIP_UP   = 0.36;

        private static final double FLICKER_DELAY_SEC = 0.9;
        private static final double BURST_TOTAL_SEC   = 1.1;

        private static final double FLIP_DOWN_BEFORE_STOP_SEC = 0.10;
        private static final double FLIP_DOWN_START_SEC =
                Math.max(FLICKER_DELAY_SEC, BURST_TOTAL_SEC - FLIP_DOWN_BEFORE_STOP_SEC);

        private final DcMotorEx intake1, intake2, flicker;
        private final Servo flipper;
        private final com.qualcomm.robotcore.util.ElapsedTime t =
                new com.qualcomm.robotcore.util.ElapsedTime();
        private boolean active = false;

        ThroughputFeeder(DcMotorEx intake1, DcMotorEx intake2, DcMotorEx flicker, Servo flipper) {
            this.intake1 = intake1;
            this.intake2 = intake2;
            this.flicker = flicker;
            this.flipper = flipper;
            stop();
        }

        void start() {
            active = true;
            t.reset();
            intake1.setPower(INTAKE1_PWR);
            intake2.setPower(INTAKE2_PWR);
            flicker.setPower(0.0);
            flipper.setPosition(FLIP_DOWN);
        }

        void update() {
            if (!active) return;

            double s = t.seconds();

            intake1.setPower(INTAKE1_PWR);
            intake2.setPower(INTAKE2_PWR);
            flicker.setPower(FLICKER_PWR);

            if (s >= FLICKER_DELAY_SEC) flipper.setPosition(FLIP_UP);
            else flipper.setPosition(FLIP_DOWN);

            if (s >= FLIP_DOWN_START_SEC) flipper.setPosition(FLIP_DOWN);

            if (s >= BURST_TOTAL_SEC) stop();
        }

        void stop() {
            active = false;
            intake1.setPower(0.0);
            intake2.setPower(0.0);
            flicker.setPower(0.0);
            flipper.setPosition(FLIP_DOWN);
        }

        boolean isActive() { return active; }
        double seconds() { return t.seconds(); }
    }

    private ThroughputFeeder feeder;
    private boolean feederStarted = false;

    // ==========================================================
    // Drive-state guards (prevents “didn't drive”)
    // ==========================================================
    private boolean driveSawBusy = false;
    private boolean driveStartedThisState = false;

    // ==========================================================
    // Gate reload wait
    // ==========================================================
    private static final double GATE_RELOAD_WAIT_SEC = 0.75;
    private final com.qualcomm.robotcore.util.ElapsedTime gateReloadWaitTimer =
            new com.qualcomm.robotcore.util.ElapsedTime();

    // Gate position wait after arriving at gate start pose
    private static final double GATE_POS_WAIT_SEC = 0.15;
    private final com.qualcomm.robotcore.util.ElapsedTime gatePosWaitTimer =
            new com.qualcomm.robotcore.util.ElapsedTime();

    // ==========================================================
    // PATH STATES (same)
    // ==========================================================
    public enum PathState {
        DRIVE_START_TO_SHOT,
        SHOOT_1,

        // shot -> reload2 -> 2nd shot
        DRIVE_SHOT_TO_RELOAD2_START,
        DRIVE_RELOAD2_START_TO_END,
        DRIVE_RELOAD2_END_TO_SHOT2,
        SHOOT_2,

        // shot2 -> gate -> shot2
        DRIVE_SHOT2_TO_GATERELOAD_START,
        WAIT_GATEPOS_0P5S_A,
        DRIVE_GATERELOAD_START_TO_RELOAD_A,
        WAIT_GATERELOAD_1P5S_A,
        DRIVE_GATERELOAD_TO_SHOT_A,
        SHOOT_3,

        // shot2 -> reload1 -> shot
        DRIVE_SHOT_TO_RELOAD1_START,
        DRIVE_RELOAD1_START_TO_END,
        DRIVE_RELOAD1_END_TO_SHOT,
        SHOOT_4,

        // split the B gate approach into two legs for head-on impact
        DRIVE_SHOT_TO_GATE_MID_B,
        DRIVE_GATE_MID_TO_GATERELOAD_START_B,

        // shot -> gate -> final shot
        WAIT_GATEPOS_0P5S_B,
        DRIVE_GATERELOAD_START_TO_RELOAD_B,
        WAIT_GATERELOAD_1P5S_B,
        DRIVE_GATERELOAD_TO_FINALSHOT,
        SHOOT_FINAL
    }

    private PathState pathState;

    // ==========================================================
    // POSES (MIRRORED FOR BLUE)
    // Mirror rule applied:
    //   x' = 144 - x
    //   y' = y
    //   heading' = heading + 180deg (wrapped)
    // ==========================================================
    private final Pose startPose      = new Pose(20.571, 122.849, Math.toRadians(138.0));
    private final Pose shotPose       = new Pose(47.887,  95.686, Math.toRadians(180.0));

    private final Pose secondShotPose = new Pose(55.000,  85.000, Math.toRadians(180.0));

    private final Pose reload1Start   = new Pose(44.000,  88.000, Math.toRadians(180.0));
    private final Pose reload1End     = new Pose(28.000,  88.000, Math.toRadians(180.0));

    private final Pose reload2Start   = new Pose(50.000,  64.00, Math.toRadians(180.0));
    private final Pose reload2End     = new Pose(32.000,  64.00, Math.toRadians(180.0));

    // ===== Gate reload points (VARIABLES) =====
    private final Pose gateReloadStartPose      = new Pose(23.25, 63.250, Math.toRadians(180.0));
    private final Pose getGateReloadStartPose2  = new Pose(22.5, 66.750, Math.toRadians(180.0));
    private final Pose gateReloadControlPose    = new Pose(22.000, 57.000, Math.toRadians(155.0)); // control; heading not used
    private final Pose gateReloadPose           = new Pose(18.250, 54.000, Math.toRadians(132.5));

    // NEW: postGateMidPose (after each gateReloadPose and before the shot pose)
    private final Pose postGateMidPose          = new Pose(18.250, 55.000, Math.toRadians(132.5));

    // control point for shot2 <-> gateReloadStartPose bezier
    private final Pose shot2ToGateControlPose   = new Pose(58.000, 61.000, Math.toRadians(180.0)); // heading not used

    // midpoint to hit gate head-on (B approach)
    private final Pose gateMidPose_B            = new Pose(29.000, 67.250, Math.toRadians(180.0));

    private final Pose finalShotPose            = new Pose(55.000, 104.000, Math.toRadians(90.0));

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

    private PathChain shotToReload1Start;
    private PathChain reload1StartToEnd;
    private PathChain reload1EndToShot;

    private PathChain shotToGateMid_B;
    private PathChain gateMidToGateReloadStart_B;

    private PathChain gateReloadStartToReload_B;
    private PathChain gateReloadToFinalShot;

    // ==========================================================
    // Hardware
    // ==========================================================
    private Limelight3A limelight;
    private DcMotorEx shooter;
    private DcMotorEx flicker;
    private DcMotorEx intake1;
    private DcMotorEx intake2;
    private Servo hood;
    private Servo flipper;

    private ShooterSubsystem shooterSys;

    private boolean didStartCommands = false;

    // Early exit tolerance
    private static final double END_TOL_IN = 2.0;

    // ==========================================================
    // BUILD PATHS
    // ==========================================================
    public void buildPaths() {

        startToShot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shotPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shotPose.getHeading())
                .build();

        // shot -> reload2Start
        shotToReload2Start = follower.pathBuilder()
                .addPath(new BezierLine(shotPose, reload2Start))
                .setConstantHeadingInterpolation(shotPose.getHeading())
                .build();

        // reload2Start -> reload2End
        reload2StartToEnd = follower.pathBuilder()
                .addPath(new BezierLine(reload2Start, reload2End))
                .setConstantHeadingInterpolation(reload2Start.getHeading())
                .build();

        // reload2End -> secondShotPose
        reload2EndToShot2 = follower.pathBuilder()
                .addPath(new BezierLine(reload2End, secondShotPose))
                .setConstantHeadingInterpolation(reload2End.getHeading())
                .build();

        // secondShotPose -> gateReloadStartPose (BezierCurve through control)
        shot2ToGateReloadStart = follower.pathBuilder()
                .addPath(new BezierCurve(secondShotPose, shot2ToGateControlPose, gateReloadStartPose))
                .setLinearHeadingInterpolation(secondShotPose.getHeading(), gateReloadStartPose.getHeading())
                .build();

        // gateReloadStartPose -> gateReloadPose (bezier curve w/ control)
        gateReloadStartToReload_A = follower.pathBuilder()
                .addPath(new BezierCurve(gateReloadStartPose, gateReloadControlPose, gateReloadPose))
                .setLinearHeadingInterpolation(gateReloadStartPose.getHeading(), gateReloadPose.getHeading())
                .build();

        // gateReloadPose -> postGateMidPose -> secondShotPose
        gateReloadToShot_A = follower.pathBuilder()
                .addPath(new BezierLine(gateReloadPose, postGateMidPose))
                .setLinearHeadingInterpolation(gateReloadPose.getHeading(), postGateMidPose.getHeading())
                .addPath(new BezierCurve(postGateMidPose, shot2ToGateControlPose, secondShotPose))
                .setLinearHeadingInterpolation(postGateMidPose.getHeading(), secondShotPose.getHeading())
                .build();

        // secondShotPose -> reload1Start
        shotToReload1Start = follower.pathBuilder()
                .addPath(new BezierLine(secondShotPose, reload1Start))
                .setConstantHeadingInterpolation(secondShotPose.getHeading())
                .build();

        // reload1Start -> reload1End
        reload1StartToEnd = follower.pathBuilder()
                .addPath(new BezierLine(reload1Start, reload1End))
                .setConstantHeadingInterpolation(reload1Start.getHeading())
                .build();

        // reload1End -> secondShotPose (kept exactly as original behavior)
        reload1EndToShot = follower.pathBuilder()
                .addPath(new BezierLine(reload1End, secondShotPose))
                .setConstantHeadingInterpolation(reload1End.getHeading())
                .build();

        // After reload1EndToShot: go through mid point first, then to gate start pose2
        shotToGateMid_B = follower.pathBuilder()
                .addPath(new BezierLine(secondShotPose, gateMidPose_B))
                .setLinearHeadingInterpolation(secondShotPose.getHeading(), gateMidPose_B.getHeading())
                .build();

        gateMidToGateReloadStart_B = follower.pathBuilder()
                .addPath(new BezierLine(gateMidPose_B, getGateReloadStartPose2))
                .setLinearHeadingInterpolation(gateMidPose_B.getHeading(), getGateReloadStartPose2.getHeading())
                .build();

        // gateReloadStartPose2 -> gateReloadPose (bezier curve w/ control) (second time)
        gateReloadStartToReload_B = follower.pathBuilder()
                .addPath(new BezierCurve(getGateReloadStartPose2, gateReloadControlPose, gateReloadPose))
                .setLinearHeadingInterpolation(getGateReloadStartPose2.getHeading(), gateReloadPose.getHeading())
                .build();

        // gateReloadPose -> postGateMidPose -> finalShotPose
        gateReloadToFinalShot = follower.pathBuilder()
                .addPath(new BezierLine(gateReloadPose, postGateMidPose))
                .setLinearHeadingInterpolation(gateReloadPose.getHeading(), postGateMidPose.getHeading())
                .addPath(new BezierLine(postGateMidPose, finalShotPose))
                .setLinearHeadingInterpolation(postGateMidPose.getHeading(), finalShotPose.getHeading())
                .build();
    }

    // ==========================================================
    // STATE MACHINE
    // ==========================================================
    public void statePathUpdate() {
        switch (pathState) {

            case DRIVE_START_TO_SHOT:
                driveCollect(startToShot, shotPose, PathState.SHOOT_1);
                break;

            case SHOOT_1:
                runShootStateThenAdvance(PathState.DRIVE_SHOT_TO_RELOAD2_START);
                break;

            // -------- reload2 -> shot2 --------
            case DRIVE_SHOT_TO_RELOAD2_START:
                driveCollect(shotToReload2Start, reload2Start, PathState.DRIVE_RELOAD2_START_TO_END);
                break;

            case DRIVE_RELOAD2_START_TO_END:
                driveCollect(reload2StartToEnd, reload2End, PathState.DRIVE_RELOAD2_END_TO_SHOT2);
                break;

            case DRIVE_RELOAD2_END_TO_SHOT2:
                driveCollect(reload2EndToShot2, secondShotPose, PathState.SHOOT_2);
                break;

            case SHOOT_2:
                runShootStateThenAdvance(PathState.DRIVE_SHOT2_TO_GATERELOAD_START);
                break;

            // -------- shot2 -> gate -> shot3 --------
            case DRIVE_SHOT2_TO_GATERELOAD_START:
                driveCollect(shot2ToGateReloadStart, gateReloadStartPose, PathState.WAIT_GATEPOS_0P5S_A);
                break;

            case WAIT_GATEPOS_0P5S_A:
                shooterSys.update();
                startTurretSettle(getAutoAimTurretCmd());

                intake1.setPower(0.0);
                intake2.setPower(0.0);
                flicker.setPower(0.0);
                flipper.setPosition(0.662);

                if (gatePosWaitTimer.seconds() >= GATE_POS_WAIT_SEC) {
                    setPathState(PathState.DRIVE_GATERELOAD_START_TO_RELOAD_A);
                }
                break;

            case DRIVE_GATERELOAD_START_TO_RELOAD_A:
                driveCollect(gateReloadStartToReload_A, gateReloadPose, PathState.WAIT_GATERELOAD_1P5S_A);
                break;

            case WAIT_GATERELOAD_1P5S_A:
                shooterSys.update();
                startTurretSettle(getAutoAimTurretCmd());

                intake1.setPower(-1.0);
                intake2.setPower(0.2);
                flicker.setPower(0.0);
                flipper.setPosition(0.662);

                if (gateReloadWaitTimer.seconds() >= GATE_RELOAD_WAIT_SEC) {
                    setPathState(PathState.DRIVE_GATERELOAD_TO_SHOT_A);
                }
                break;

            case DRIVE_GATERELOAD_TO_SHOT_A:
                driveNoCollect(gateReloadToShot_A, secondShotPose, PathState.SHOOT_3);
                break;

            case SHOOT_3:
                runShootStateThenAdvance(PathState.DRIVE_SHOT_TO_RELOAD1_START);
                break;

            // -------- shot2 -> reload1 -> shot --------
            case DRIVE_SHOT_TO_RELOAD1_START:
                driveCollect(shotToReload1Start, reload1Start, PathState.DRIVE_RELOAD1_START_TO_END);
                break;

            case DRIVE_RELOAD1_START_TO_END:
                driveCollect(reload1StartToEnd, reload1End, PathState.DRIVE_RELOAD1_END_TO_SHOT);
                break;

            case DRIVE_RELOAD1_END_TO_SHOT:
                driveCollect(reload1EndToShot, shotPose, PathState.SHOOT_4);
                break;

            case SHOOT_4:
                runShootStateThenAdvance(PathState.DRIVE_SHOT_TO_GATE_MID_B);
                break;

            case DRIVE_SHOT_TO_GATE_MID_B:
                driveCollect(shotToGateMid_B, gateMidPose_B, PathState.DRIVE_GATE_MID_TO_GATERELOAD_START_B);
                break;

            case DRIVE_GATE_MID_TO_GATERELOAD_START_B:
                driveCollect(gateMidToGateReloadStart_B, getGateReloadStartPose2, PathState.WAIT_GATEPOS_0P5S_B);
                break;

            // -------- shot -> gate -> final shot --------
            case WAIT_GATEPOS_0P5S_B:
                shooterSys.update();
                startTurretSettle(getAutoAimTurretCmd());

                intake1.setPower(0.0);
                intake2.setPower(0.0);
                flicker.setPower(0.0);
                flipper.setPosition(0.662);

                if (gatePosWaitTimer.seconds() >= GATE_POS_WAIT_SEC) {
                    setPathState(PathState.DRIVE_GATERELOAD_START_TO_RELOAD_B);
                }
                break;

            case DRIVE_GATERELOAD_START_TO_RELOAD_B:
                driveCollect(gateReloadStartToReload_B, gateReloadPose, PathState.WAIT_GATERELOAD_1P5S_B);
                break;

            case WAIT_GATERELOAD_1P5S_B:
                shooterSys.update();
                startTurretSettle(getAutoAimTurretCmd());

                intake1.setPower(-1.0);
                intake2.setPower(0.4);
                flicker.setPower(0.0);
                flipper.setPosition(0.662);

                if (gateReloadWaitTimer.seconds() >= GATE_RELOAD_WAIT_SEC) {
                    setPathState(PathState.DRIVE_GATERELOAD_TO_FINALSHOT);
                }
                break;

            case DRIVE_GATERELOAD_TO_FINALSHOT:
                driveNoCollect(gateReloadToFinalShot, finalShotPose, PathState.SHOOT_FINAL);
                break;

            case SHOOT_FINAL:
            default:
                runShootStateThenHold();
                break;
        }
    }

    // ==========================================================
    // DRIVE helper (COLLECT)
    // ==========================================================
    private void driveCollect(PathChain path, Pose endPose, PathState next) {

        if (!driveStartedThisState) {
            follower.followPath(path, true);
            driveStartedThisState = true;
        }

        if (follower.isBusy()) driveSawBusy = true;

        shooterSys.update();
        startTurretSettle(getAutoAimTurretCmd());

        intake1.setPower(DRIVE_INTAKE1_PWR);
        intake2.setPower(DRIVE_INTAKE2_PWR);
        flicker.setPower(0.0);
        flipper.setPosition(0.662);

        updateReadyLatchNear(shotPose);

        Pose cur = follower.getPose();
        boolean closeEnough = dist(cur, endPose) <= END_TOL_IN;

        if (driveSawBusy && (!follower.isBusy() || closeEnough)) {
            intake1.setPower(0.0);
            intake2.setPower(0.0);
            setPathState(next);
        }
    }

    // ==========================================================
    // DRIVE helper (NO COLLECT)
    // ==========================================================
    private void driveNoCollect(PathChain path, Pose endPose, PathState next) {

        if (!driveStartedThisState) {
            follower.followPath(path, true);
            driveStartedThisState = true;
        }

        if (follower.isBusy()) driveSawBusy = true;

        shooterSys.update();
        startTurretSettle(getAutoAimTurretCmd());

        intake1.setPower(0.0);
        intake2.setPower(0.0);
        flicker.setPower(0.0);
        flipper.setPosition(0.662);

        updateReadyLatchNear(shotPose);

        Pose cur = follower.getPose();
        boolean closeEnough = dist(cur, endPose) <= END_TOL_IN;

        if (driveSawBusy && (!follower.isBusy() || closeEnough)) {
            intake1.setPower(0.0);
            intake2.setPower(0.0);
            setPathState(next);
        }
    }

    // ==========================================================
    // SHOOT helper: fast feeder
    // ==========================================================
    private void runShootStateThenAdvance(PathState nextState) {
        shooterSys.update();
        startTurretSettle(getAutoAimTurretCmd());

        if (!feederStarted) {

            if (follower.isBusy()) {
                feeder.stop();
                feederStarted = false;
                stopFeedHardware();
                return;
            }

            boolean tagNow = shooterSys.isTagSeen();
            boolean readyStable = shooterSys.isShooterReadyStable();
            boolean timeoutNoTag = shootGateTimer.seconds() >= NO_TAG_START_TIMEOUT_SEC;

            boolean okToStart = (tagNow && readyStable) || isReadyLatched() || (timeoutNoTag && readyStable);

            feeder.stop();
            stopFeedHardware();

            if (shooterSys.isEnabled() && okToStart) {
                feederStarted = true;
                shooterSys.resetReadyStable();
                feeder.start();
            }
            return;
        }

        feeder.update();

        if (!feeder.isActive()) {
            feederStarted = false;
            feeder.stop();
            stopFeedHardware();
            setPathState(nextState);
        }
    }

    private void runShootStateThenHold() {
        shooterSys.update();
        startTurretSettle(getAutoAimTurretCmd());

        if (!feederStarted) {

            if (follower.isBusy()) {
                feeder.stop();
                feederStarted = false;
                stopFeedHardware();
                return;
            }

            boolean tagNow = shooterSys.isTagSeen();
            boolean readyStable = shooterSys.isShooterReadyStable();
            boolean timeoutNoTag = shootGateTimer.seconds() >= NO_TAG_START_TIMEOUT_SEC;

            boolean okToStart = (tagNow && readyStable) || isReadyLatched() || (timeoutNoTag && readyStable);

            feeder.stop();
            stopFeedHardware();

            if (shooterSys.isEnabled() && okToStart) {
                feederStarted = true;
                shooterSys.resetReadyStable();
                feeder.start();
            }
            return;
        }

        feeder.update();

        if (!feeder.isActive()) {
            feederStarted = false;
            feeder.stop();
            stopFeedHardware();
            writePoseStorageNow();
            startTurretSettle(0.5);
        }
    }

    private void stopFeedHardware() {
        intake1.setPower(0.0);
        intake2.setPower(0.0);
        flicker.setPower(0.0);
        flipper.setPosition(0.662);
    }

    // ==========================================================
    // STATE ENTRY
    // ==========================================================
    public void setPathState(PathState newState) {
        pathState = newState;

        pathTimer.resetTimer();
        shootGateTimer.reset();
        resetReadyLatch();

        driveSawBusy = false;
        driveStartedThisState = false;

        if (newState == PathState.WAIT_GATEPOS_0P5S_A || newState == PathState.WAIT_GATEPOS_0P5S_B) {
            gatePosWaitTimer.reset();
        }

        if (newState == PathState.WAIT_GATERELOAD_1P5S_A || newState == PathState.WAIT_GATERELOAD_1P5S_B) {
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

        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);
        flicker = hardwareMap.get(DcMotorEx.class, FLICKER_NAME);
        intake1 = hardwareMap.get(DcMotorEx.class, INTAKE1_NAME);
        intake2 = hardwareMap.get(DcMotorEx.class, INTAKE2_NAME);
        hood = hardwareMap.get(Servo.class, HOOD_NAME);
        flipper = hardwareMap.get(Servo.class, FLIPPER_NAME);

        turretLeft  = hardwareMap.get(Servo.class, TURRET_LEFT_NAME);
        turretRight = hardwareMap.get(Servo.class, TURRET_RIGHT_NAME);

        turretTargetPos = TURRET_START_POS;
        turretSettleStarted = false;
        turretSettleTimer.reset();

        lastTurretCmd = clamp(TURRET_START_POS, SERVO_MIN_SAFE, SERVO_MAX_SAFE);
        loopTimer.reset();

        visionTrimDegCW = 0.0;
        visionTrimDegCW_raw = 0.0;
        visionHasGood = false;
        visionLastGoodTimer.reset();
        visionDecayTimer.reset();

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidf = new PIDFCoefficients(180, 0, 0, 15.5022);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        // Only changes here are tag ID -> 20 and the aiming goal coords handled above.
        shooterSys = new ShooterSubsystem(
                limelight, shooter, hood,
                0,
                -1, 20,
                8.0,
                180, 15.5022,
                new double[]{16, 32, 48, 64, 80, 120},
                new double[]{0.310, 0.520, 0.680, 0.690, 0.780, 0.790},
                new double[]{1020, 1055, 1120, 1210, 1260, 1440},
                new double[]{1040, 1080, 1140, 1225, 1280, 1470},
                new double[]{1050, 1090, 1160, 1240, 1300, 1500},
                110
        );

        shooterSys.startVision();
        shooterSys.setEnabled(false);

        feeder = new ThroughputFeeder(intake1, intake2, flicker, flipper);
        feeder.stop();
        feederStarted = false;

        resetReadyLatch();

        buildPaths();
        follower.setPose(startPose);

        stopFeedHardware();

        didStartCommands = false;

        driveSawBusy = false;
        driveStartedThisState = false;

        gateReloadWaitTimer.reset();
        gatePosWaitTimer.reset();

        pathState = PathState.DRIVE_START_TO_SHOT;

        telemetry.addLine("Ready. Waiting for start.");
        telemetry.update();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);

        if (!didStartCommands) {
            applyTurret(TURRET_START_POS);
            turretTargetPos = TURRET_START_POS;
            turretSettleStarted = false;
            turretSettleTimer.reset();

            flipper.setPosition(0.662);

            shooterSys.setEnabled(true);

            didStartCommands = true;
        }
    }

    @Override
    public void loop() {
        follower.update();

        shooterSys.setExternalDistanceIn(getAutoAimShooterDistanceIn());
        shooterSys.update();

        updateVisionTrim();

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

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());

        telemetry.addData("turret target", "%.3f", turretTargetPos);
        telemetry.addData("turret settled", isTurretSettled());

        telemetry.addData("ReadyLatched", isReadyLatched());
        telemetry.addData("Shooter ReadyStable", shooterSys.isShooterReadyStable());

        telemetry.addData("FeederStarted", feederStarted);
        telemetry.addData("FeederActive", feeder.isActive());
        telemetry.addData("FeederT(s)", "%.2f", feeder.seconds());

        telemetry.update();
    }

    @Override
    public void stop() {
        feeder.stop();
        stopFeedHardware();
        writePoseStorageNow();
    }

    // ======= TURRET HELPERS =======
    private void applyTurret(double p) {
        turretLeft.setPosition(p);
        if (MIRROR_RIGHT) turretRight.setPosition(1.0 - p);
        else turretRight.setPosition(p);
    }

    private void startTurretSettle(double targetPos) {
        if (Math.abs(targetPos - turretTargetPos) > 1e-6) {
            turretTargetPos = targetPos;
            turretSettleStarted = false;
        }

        applyTurret(turretTargetPos);

        if (!turretSettleStarted) {
            turretSettleStarted = true;
            turretSettleTimer.reset();
        }
    }

    private boolean isTurretSettled() {
        return turretSettleStarted && turretSettleTimer.seconds() >= TURRET_SETTLE_SEC;
    }

    // ==========================================================
    // Ready latch helpers
    // ==========================================================
    private void resetReadyLatch() {
        readyLatched = false;
        readyLatchTimer.reset();
    }

    private boolean isReadyLatched() {
        return readyLatched && (readyLatchTimer.seconds() <= READY_LATCH_HOLD_SEC);
    }

    private static double dist(Pose a, Pose b) {
        double dx = a.getX() - b.getX();
        double dy = a.getY() - b.getY();
        return Math.hypot(dx, dy);
    }

    private void updateReadyLatchNear(Pose shotPoseRef) {
        if (dist(follower.getPose(), shotPoseRef) > PRECHECK_DIST_IN) return;

        boolean ready = shooterSys.isShooterReadyStable();
        boolean tag = shooterSys.isTagSeen();

        if (ready && tag) {
            readyLatched = true;
            readyLatchTimer.reset();
        }
    }

    private double getAutoAimShooterDistanceIn() {
        Pose pose = follower.getPose();
        double robotX = pose.getX();
        double robotY = pose.getY();
        double headingRad = pose.getHeading();

        double fwdX = Math.cos(headingRad);
        double fwdY = Math.sin(headingRad);

        double leftX = -Math.sin(headingRad);
        double leftY =  Math.cos(headingRad);

        double turretX = robotX + fwdX * TURRET_FWD_OFFSET_IN + leftX * TURRET_LEFT_OFFSET_IN;
        double turretY = robotY + fwdY * TURRET_FWD_OFFSET_IN + leftY * TURRET_LEFT_OFFSET_IN;

        return Math.hypot(BLUE_GOAL_PX - turretX, BLUE_GOAL_PY - turretY);
    }

    private double getAutoAimTurretCmd() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        dt = clamp(dt, 0.005, 0.050);

        Pose pose = follower.getPose();
        double robotX = pose.getX();
        double robotY = pose.getY();
        double headingRad = pose.getHeading();

        double fwdX = Math.cos(headingRad);
        double fwdY = Math.sin(headingRad);

        double leftX = -Math.sin(headingRad);
        double leftY =  Math.cos(headingRad);

        double turretX = robotX + fwdX * TURRET_FWD_OFFSET_IN + leftX * TURRET_LEFT_OFFSET_IN;
        double turretY = robotY + fwdY * TURRET_FWD_OFFSET_IN + leftY * TURRET_LEFT_OFFSET_IN;

        double bearingDegField = Math.toDegrees(Math.atan2(BLUE_GOAL_PY - turretY, BLUE_GOAL_PX - turretX));
        bearingDegField = wrapDeg180(bearingDegField);

        double headingDeg = wrapDeg180(Math.toDegrees(headingRad));

        double relDegCCW = wrapDeg180(bearingDegField - headingDeg);
        double turretDegCW_odo = -relDegCCW;

        double turretDegCW_total = wrapDeg180(turretDegCW_odo + visionTrimDegCW);

        double turretCmdTarget = TURRET_HOME + turretDegCW_total * POS_PER_DEG_CW;
        turretCmdTarget = clamp(turretCmdTarget, SERVO_MIN_SAFE, SERVO_MAX_SAFE);

        double currentTurretDegCW = (lastTurretCmd - TURRET_HOME) / POS_PER_DEG_CW;

        double errDegCW = wrapDeg180(turretDegCW_total - currentTurretDegCW);
        if (Math.abs(errDegCW) < BEARING_DEADBAND_DEG) errDegCW = 0.0;

        double deltaServo = errDegCW * KP_DEG_TO_SERVO;

        double desiredCmd = lastTurretCmd + deltaServo;
        desiredCmd = clamp(desiredCmd, SERVO_MIN_SAFE, SERVO_MAX_SAFE);

        double appliedCmd = slewTo(lastTurretCmd, desiredCmd, SERVO_SLEW_PER_SEC, dt);
        appliedCmd = clamp(appliedCmd,
                Math.max(SERVO_MIN_SAFE, lastTurretCmd - MAX_STEP_PER_LOOP),
                Math.min(SERVO_MAX_SAFE, lastTurretCmd + MAX_STEP_PER_LOOP));

        lastTurretCmd = appliedCmd;
        return lastTurretCmd;
    }

    private void updateVisionTrim() {
        tagSeen = false;
        tagId = -1;
        camX_in = Double.NaN;
        camZ_in = Double.NaN;
        camBearingDeg = Double.NaN;
        txDeg = Double.NaN;

        if (limelight == null) { handleVisionLost(); return; }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) { handleVisionLost(); return; }

        try { txDeg = result.getTx(); } catch (Exception ignored) { txDeg = Double.NaN; }

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) { handleVisionLost(); return; }

        LLResultTypes.FiducialResult chosen = null;
        for (LLResultTypes.FiducialResult t : tags) {
            int id = (int) t.getFiducialId();
            if (id == BLUE_GOAL_TAG_ID) { chosen = t; break; }
        }
        if (chosen == null) { handleVisionLost(); return; }

        Pose3D pose = null;
        try { pose = chosen.getCameraPoseTargetSpace(); } catch (Exception ignored) {}
        if (pose == null) { handleVisionLost(); return; }

        Position p = pose.getPosition();

        double xIn = DistanceUnit.INCH.fromUnit(p.unit, p.x);
        double zIn = DistanceUnit.INCH.fromUnit(p.unit, p.z);
        double zAbs = Math.abs(zIn);

        if (zAbs < VISION_MIN_Z_IN || zAbs > VISION_MAX_Z_IN) { handleVisionLost(); return; }
        if (Double.isNaN(txDeg) || Math.abs(txDeg) > VISION_MAX_ABS_TX_DEG) { handleVisionLost(); return; }

        tagSeen = true;
        tagId = (int) chosen.getFiducialId();
        camX_in = xIn;
        camZ_in = zIn;
        camBearingDeg = txDeg;

        double trimCW = (VISION_X_SIGN * txDeg);
        trimCW = clamp(trimCW, -VISION_MAX_TRIM_DEG, +VISION_MAX_TRIM_DEG);

        visionTrimDegCW_raw = trimCW;

        if (!visionHasGood) {
            visionTrimDegCW = trimCW;
            visionHasGood = true;
        } else {
            visionTrimDegCW = (VISION_LPF_ALPHA * trimCW) + ((1.0 - VISION_LPF_ALPHA) * visionTrimDegCW);
        }

        visionLastGoodTimer.reset();
        visionDecayTimer.reset();
    }

    private void handleVisionLost() {
        if (!visionHasGood) {
            visionTrimDegCW_raw = 0.0;
            visionTrimDegCW = 0.0;
            return;
        }

        long msSince = (long) visionLastGoodTimer.milliseconds();
        if (msSince <= VISION_HOLD_MS) return;

        double t = clamp(visionDecayTimer.milliseconds() / (double) VISION_DECAY_MS, 0.0, 1.0);
        visionTrimDegCW = (1.0 - t) * visionTrimDegCW;

        if (t >= 1.0) {
            visionTrimDegCW = 0.0;
            visionTrimDegCW_raw = 0.0;
            visionHasGood = false;
        }
    }

    private double slewTo(double current, double target, double ratePerSec, double dt) {
        double maxStep = ratePerSec * dt;
        double delta = target - current;
        if (delta >  maxStep) delta =  maxStep;
        if (delta < -maxStep) delta = -maxStep;
        return current + delta;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double wrapDeg180(double d) {
        while (d > 180.0) d -= 360.0;
        while (d < -180.0) d += 360.0;
        return d;
    }
}