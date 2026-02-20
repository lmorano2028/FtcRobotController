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

@Autonomous(name = "BlueAutoCloseRange15BallOdo_FASTFEED", group = "Blue")
public class BlueAutoCloseRange15BallOdo_FASTFEED extends OpMode {

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
    // Mirror of red goal (142,142) across X=72 on a 144" field => blue goal ~ (2,142)
    private static final double BLUE_GOAL_PX = 2;
    private static final double BLUE_GOAL_PY = 142;
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
    private static final double NO_TAG_START_TIMEOUT_SEC = 0.25;
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
    // Gate reload wait (uses variable below)
    // ==========================================================
    private static final double GATE_RELOAD_WAIT_SEC = 1.750;

    private final com.qualcomm.robotcore.util.ElapsedTime gateReloadWaitTimer =
            new com.qualcomm.robotcore.util.ElapsedTime();

    // ==========================================================
    // Gate position wait after arriving at gate start pose
    // ==========================================================
    private static final double GATE_POS_WAIT_SEC = 0.15;
    private final com.qualcomm.robotcore.util.ElapsedTime gatePosWaitTimer =
            new com.qualcomm.robotcore.util.ElapsedTime();

    // ==========================================================
    // PATH STATES
    // ==========================================================
    public enum PathState {
        DRIVE_START_TO_SHOT,
        SHOOT_1,

        DRIVE_SHOT_TO_RELOAD1_START,
        DRIVE_RELOAD1_START_TO_END,
        DRIVE_RELOAD1_END_TO_SHOT,
        SHOOT_2,

        DRIVE_SHOT_TO_RELOAD2_START,
        DRIVE_RELOAD2_START_TO_END,
        DRIVE_RELOAD2_END_TO_SHOT,
        SHOOT_3,

        DRIVE_SHOT_TO_GATERELOAD_START,
        WAIT_GATEPOS_0P5S,
        DRIVE_GATERELOAD_START_TO_RELOAD,
        WAIT_GATERELOAD_1P5S,

        DRIVE_GATERELOAD_TO_SHOT,
        SHOOT_4,

        DRIVE_SHOT_TO_RELOAD3_START,
        DRIVE_RELOAD3_START_TO_END,
        DRIVE_RELOAD3_END_TO_FINALSHOT,
        SHOOT_FINAL
    }

    private PathState pathState;

    // ==========================================================
    // POSES (BLUE)
    // ==========================================================
    private final Pose startPose = new Pose(20.571428571428577, 122.84909456740445, Math.toRadians(138));
    private final Pose shotPose  = new Pose(47.88732394366197,   95.68611670020124, Math.toRadians(180));

    private final Pose reload1Start = new Pose(44.000, 88.000, Math.toRadians(180));
    private final Pose reload1End   = new Pose(28.000, 88.000, Math.toRadians(180));

    private final Pose reload2Start = new Pose(50.000, 66.500, Math.toRadians(180));
    private final Pose reload2End   = new Pose(29.000, 66.500, Math.toRadians(180));

    // ===== Gate reload points (VARIABLES) =====
    private final Pose gateReloadStartPose   = new Pose(16.500, 61.000, Math.toRadians(155)); // openGatePos
    private final Pose gateReloadControlPose = new Pose(22.000, 57.000, Math.toRadians(180)); // control point; heading not used
    private final Pose gateReloadPose        = new Pose(16.000, 53.000, Math.toRadians(135));

    private final Pose reload3Start = new Pose(50.000, 42.500, Math.toRadians(180));
    private final Pose reload3End   = new Pose(23.000, 42.500, Math.toRadians(180));

    private final Pose finalShotPose = new Pose(55.000, 106.000, Math.toRadians(180));

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
                .addPath(new BezierLine(
                        startPose,
                        shotPose
                ))
                .setLinearHeadingInterpolation(startPose.getHeading(), shotPose.getHeading())
                .build();

        shotToReload1Start = follower.pathBuilder()
                .addPath(new BezierLine(
                        shotPose,
                        reload1Start
                ))
                // all former 0 headings -> 180 for blue
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        reload1StartToEnd = follower.pathBuilder()
                .addPath(new BezierLine(
                        reload1Start,
                        reload1End
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        reload1EndToShot = follower.pathBuilder()
                .addPath(new BezierLine(
                        reload1End,
                        shotPose
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shotToReload2Start = follower.pathBuilder()
                .addPath(new BezierLine(
                        shotPose,
                        reload2Start
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        reload2StartToEnd = follower.pathBuilder()
                .addPath(new BezierLine(
                        reload2Start,
                        reload2End
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        reload2EndToShot = follower.pathBuilder()
                .addPath(new BezierLine(
                        reload2End,
                        shotPose
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // shot -> gateReloadStartPose
        shotToGateReloadStart = follower.pathBuilder()
                .addPath(new BezierLine(
                        shotPose,
                        gateReloadStartPose
                ))
                // former (0,0) -> (180,180) per your rule
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
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
                // former end heading 0 -> 180
                .setLinearHeadingInterpolation(gateReloadPose.getHeading(), Math.toRadians(180))
                .build();

        shotToReload3Start = follower.pathBuilder()
                .addPath(new BezierLine(
                        shotPose,
                        reload3Start
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        reload3StartToEnd = follower.pathBuilder()
                .addPath(new BezierLine(
                        reload3Start,
                        reload3End
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        reload3EndToFinalShot = follower.pathBuilder()
                .addPath(new BezierLine(
                        reload3End,
                        finalShotPose
                ))
                // former start 0 -> 180; end was 90 but blue final is 180
                .setLinearHeadingInterpolation(Math.toRadians(180), finalShotPose.getHeading())
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
                runShootStateThenAdvance(PathState.DRIVE_SHOT_TO_RELOAD1_START);
                break;

            case DRIVE_SHOT_TO_RELOAD1_START:
                driveCollect(shotToReload1Start, reload1Start, PathState.DRIVE_RELOAD1_START_TO_END);
                break;

            case DRIVE_RELOAD1_START_TO_END:
                driveCollect(reload1StartToEnd, reload1End, PathState.DRIVE_RELOAD1_END_TO_SHOT);
                break;

            case DRIVE_RELOAD1_END_TO_SHOT:
                driveCollect(reload1EndToShot, shotPose, PathState.SHOOT_2);
                break;

            case SHOOT_2:
                runShootStateThenAdvance(PathState.DRIVE_SHOT_TO_RELOAD2_START);
                break;

            case DRIVE_SHOT_TO_RELOAD2_START:
                driveCollect(shotToReload2Start, reload2Start, PathState.DRIVE_RELOAD2_START_TO_END);
                break;

            case DRIVE_RELOAD2_START_TO_END:
                driveCollect(reload2StartToEnd, reload2End, PathState.DRIVE_RELOAD2_END_TO_SHOT);
                break;

            case DRIVE_RELOAD2_END_TO_SHOT:
                driveCollect(reload2EndToShot, shotPose, PathState.SHOOT_3);
                break;

            case SHOOT_3:
                runShootStateThenAdvance(PathState.DRIVE_SHOT_TO_GATERELOAD_START);
                break;

            case DRIVE_SHOT_TO_GATERELOAD_START:
                driveCollect(shotToGateReloadStart, gateReloadStartPose, PathState.WAIT_GATEPOS_0P5S);
                break;

            case WAIT_GATEPOS_0P5S:
                shooterSys.update();
                startTurretSettle(getAutoAimTurretCmd());

                // opening the gate at gate pos, intake OFF
                intake1.setPower(0.0);
                intake2.setPower(0.0);
                flicker.setPower(0.0);
                flipper.setPosition(0.662);

                if (gatePosWaitTimer.seconds() >= GATE_POS_WAIT_SEC) {
                    setPathState(PathState.DRIVE_GATERELOAD_START_TO_RELOAD);
                }
                break;

            case DRIVE_GATERELOAD_START_TO_RELOAD:
                driveCollect(gateReloadStartToReload, gateReloadPose, PathState.WAIT_GATERELOAD_1P5S);
                break;

            case WAIT_GATERELOAD_1P5S:
                shooterSys.update();
                startTurretSettle(getAutoAimTurretCmd());

                intake1.setPower(-1.0);
                intake2.setPower(0.4);
                flicker.setPower(0.0);
                flipper.setPosition(0.662);

                if (gateReloadWaitTimer.seconds() >= GATE_RELOAD_WAIT_SEC) {
                    setPathState(PathState.DRIVE_GATERELOAD_TO_SHOT);
                }
                break;

            case DRIVE_GATERELOAD_TO_SHOT:
                driveNoCollect(gateReloadToShot, shotPose, PathState.SHOOT_4);
                break;

            case SHOOT_4:
                runShootStateThenAdvance(PathState.DRIVE_SHOT_TO_RELOAD3_START);
                break;

            case DRIVE_SHOT_TO_RELOAD3_START:
                driveCollect(shotToReload3Start, reload3Start, PathState.DRIVE_RELOAD3_START_TO_END);
                break;

            case DRIVE_RELOAD3_START_TO_END:
                driveCollect(reload3StartToEnd, reload3End, PathState.DRIVE_RELOAD3_END_TO_FINALSHOT);
                break;

            case DRIVE_RELOAD3_END_TO_FINALSHOT:
                driveNoCollect(reload3EndToFinalShot, finalShotPose, PathState.SHOOT_FINAL);
                break;

            case SHOOT_FINAL:
            default:
                runShootStateThenHold();
                break;
        }
    }

    // ==========================================================
    // DRIVE helper (COLLECT): start once, require “saw busy” before completing
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
    // DRIVE helper (NO COLLECT): intake OFF while driving
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

        if (newState == PathState.WAIT_GATEPOS_0P5S) {
            gatePosWaitTimer.reset();
        }

        if (newState == PathState.WAIT_GATERELOAD_1P5S) {
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

        shooterSys = new ShooterSubsystem(
                limelight, shooter, hood,
                0,
                -1, BLUE_GOAL_TAG_ID,
                8.0,
                180, 15.5022,
                new double[]{16, 32, 48, 64, 80, 120},
                new double[]{0.310, 0.520, 0.720, 0.730, 0.780, 0.790},
                new double[]{1020, 1055, 1120, 1200, 1260, 1440},
                new double[]{1040, 1080, 1130, 1215, 1280, 1470},
                new double[]{1050, 1090, 1150, 1230, 1300, 1500},
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