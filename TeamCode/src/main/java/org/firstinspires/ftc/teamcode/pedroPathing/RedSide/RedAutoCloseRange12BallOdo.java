package org.firstinspires.ftc.teamcode.pedroPathing.RedSide;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
@Autonomous(name = "RedAutoCloseRange12BallOdo_FASTFEED", group = "Pedro")
public class RedAutoCloseRange12BallOdo extends OpMode {
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

    // turret settle gate (uses its OWN timer so it can't be stomped by pathTimer)
    private static final double TURRET_SETTLE_SEC = 0.15;
    private boolean turretSettleStarted = false;
    private double turretTargetPos = TURRET_START_POS;
    private final com.qualcomm.robotcore.util.ElapsedTime turretSettleTimer = new com.qualcomm.robotcore.util.ElapsedTime();

    // ==========================================================
    // ODOMETRY + VISION TRIM TURRET AIM (PEDRO POSE, RED ONLY)
    // ==========================================================
    private static final double RED_GOAL_PX = 142;
    private static final double RED_GOAL_PY = 142;

    private static final int RED_GOAL_TAG_ID = 24;

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
    private final com.qualcomm.robotcore.util.ElapsedTime loopTimer = new com.qualcomm.robotcore.util.ElapsedTime();

    private boolean visionHasGood = false;
    private double visionTrimDegCW = 0.0;
    private double visionTrimDegCW_raw = 0.0;
    private final com.qualcomm.robotcore.util.ElapsedTime visionLastGoodTimer = new com.qualcomm.robotcore.util.ElapsedTime();
    private final com.qualcomm.robotcore.util.ElapsedTime visionDecayTimer = new com.qualcomm.robotcore.util.ElapsedTime();

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
    private final com.qualcomm.robotcore.util.ElapsedTime readyLatchTimer = new com.qualcomm.robotcore.util.ElapsedTime();

    // ==========================================================
    // "never deadlocks" gate when tag is missing
    // ==========================================================
    private static final double NO_TAG_START_TIMEOUT_SEC = 0.35;
    private final com.qualcomm.robotcore.util.ElapsedTime shootGateTimer = new com.qualcomm.robotcore.util.ElapsedTime();

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
    private static final double DRIVE_INTAKE2_PWR =  0.10;

    // ==========================================================
    // FAST THROUGHOUT FEEDER
    // ==========================================================
    private static final class ThroughputFeeder {
        private static final double INTAKE1_PWR = -1.0;
        private static final double INTAKE2_PWR =  1.0;
        private static final double FLICKER_PWR =  1.0;

        private static final double FLIP_DOWN = 0.662;
        private static final double FLIP_UP   = 0.36;

        private static final double FLICKER_DELAY_SEC = 1.10;
        private static final double BURST_TOTAL_SEC   = 1.35;

        private static final double FLIP_DOWN_BEFORE_STOP_SEC = 0.10;
        private static final double FLIP_DOWN_START_SEC =
                Math.max(FLICKER_DELAY_SEC, BURST_TOTAL_SEC - FLIP_DOWN_BEFORE_STOP_SEC);

        private final DcMotorEx intake1, intake2, flicker;
        private final Servo flipper;
        private final com.qualcomm.robotcore.util.ElapsedTime t = new com.qualcomm.robotcore.util.ElapsedTime();
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
    // Drive-state guards (THIS is the fix)
    // ==========================================================
    private boolean driveSawBusy = false;
    private boolean driveStartedThisState = false;

    // ==========================================================
    // PATH STATES
    // ==========================================================
    public enum PathState {
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,

        DRIVE_SHOOTPOS_RELOAD3POS,
        DRIVE_THIRD_RELOAD_SEQUENCE,

        DRIVE_RELOAD_TO_SHOT,
        SHOOT_POST_RELOAD,

        DRIVE_SHOT_TO_RELOAD2_START,
        DRIVE_RELOAD2_SEQUENCE,

        DRIVE_RELOAD2_TO_OPEN_GATE,
        DRIVE_OPEN_GATE_TO_SHOTPOS,
        SHOOT_POST_RELOAD_2,

        DRIVE_SHOT_TO_RELOAD3_START,
        DRIVE_RELOAD3_SEQUENCE,

        DRIVE_RELOAD3_TO_SHOTPOS,
        SHOOT_FINAL
    }

    private PathState pathState;

    // ==========================================================
    // POSES + PATHS
    // ==========================================================
    private final Pose startPose = new Pose(123.42857142857142, 122.84909456740445, Math.toRadians(42));
    private final Pose shootPose = new Pose(96.11267605633803,  95.68611670020124,  Math.toRadians(42));
    private final Pose finalShotPose = new Pose(89, 104, Math.toRadians(0));

    private PathChain driveStartPosShootPos, driveShootPosReloadPos, driveReloadThree, driveReloadPosToShootPos;
    private PathChain driveShotToReload2Start, driveReload2;
    private PathChain driveReload2ToOpenGate, driveOpenGateToShotPos;
    private PathChain driveShotToReload3Start, driveReload3, driveReload3ToShotPos;

    // ======= Hardware =======
    private Limelight3A limelight;
    private DcMotorEx shooter;
    private DcMotorEx flicker;
    private DcMotorEx intake1;
    private DcMotorEx intake2;
    private Servo hood;
    private Servo flipper;

    private ShooterSubsystem shooterSys;

    private boolean didStartCommands = false;

    // early exit tolerance
    private static final double END_TOL_IN = 2.0;

    // ======================================================================
    // BUILD PATHS
    // ======================================================================
    public void buildPaths() {
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(123.429, 122.849, Math.toRadians(42)),
                        new Pose(96.113,  95.686,  Math.toRadians(42))
                ))
                .setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(42))
                .build();

        driveShootPosReloadPos = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(96.113, 95.686, Math.toRadians(42)),
                        new Pose(95.000, 89.000, Math.toRadians(0))
                ))
                .setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(0))
                .build();

        driveReloadThree = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(95.000, 89.000,  Math.toRadians(0)),
                        new Pose(122.000, 89.000, Math.toRadians(0))
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        driveReloadPosToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(122.000, 89.000, Math.toRadians(0)),
                        new Pose(96.113,  95.686, Math.toRadians(0))
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setReversed()
                .build();

        driveShotToReload2Start = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(96.113, 95.686, Math.toRadians(0)),
                        new Pose(95.000, 66.000, Math.toRadians(0))
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        driveReload2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(95.000, 66.000,  Math.toRadians(0)),
                        new Pose(123.000, 66.000, Math.toRadians(0))
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        driveReload2ToOpenGate = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(121.000, 66.000, Math.toRadians(0)),
                        new Pose(123.000, 70.000, Math.toRadians(0))
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        driveOpenGateToShotPos = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(123.000, 70.000, Math.toRadians(0)),
                        new Pose(96.113,  95.686, Math.toRadians(0))
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        driveShotToReload3Start = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(96.113, 95.686, Math.toRadians(0)),
                        new Pose(95.000, 41.000, Math.toRadians(0))
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        driveReload3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(95.000, 41.000,  Math.toRadians(0)),
                        new Pose(123.000, 41.000, Math.toRadians(0))
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        driveReload3ToShotPos = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(123.000, 41.000, Math.toRadians(0)),
                        new Pose(89.000,  104.000, Math.toRadians(90))
                ))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
    }

    // ==========================================================
    // STATE MACHINE
    // ==========================================================
    public void statePathUpdate() {
        switch (pathState) {

            case DRIVE_STARTPOS_SHOOT_POS:
                // FIX: stay in drive state until the follower actually runs the path
                driveCollect(driveStartPosShootPos, new Pose(96.113, 95.686), PathState.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:
                runShootStateThenAdvance(PathState.DRIVE_SHOOTPOS_RELOAD3POS);
                break;

            case DRIVE_SHOOTPOS_RELOAD3POS:
                driveCollect(driveShootPosReloadPos, new Pose(95.000, 89.000), PathState.DRIVE_THIRD_RELOAD_SEQUENCE);
                break;

            case DRIVE_THIRD_RELOAD_SEQUENCE:
                driveCollect(driveReloadThree, new Pose(122.000, 89.000), PathState.DRIVE_RELOAD_TO_SHOT);
                break;

            case DRIVE_RELOAD_TO_SHOT:
                driveCollect(driveReloadPosToShootPos, new Pose(96.113, 95.686), PathState.SHOOT_POST_RELOAD);
                break;

            case SHOOT_POST_RELOAD:
                runShootStateThenAdvance(PathState.DRIVE_SHOT_TO_RELOAD2_START);
                break;

            case DRIVE_SHOT_TO_RELOAD2_START:
                driveCollect(driveShotToReload2Start, new Pose(95.000, 66.000), PathState.DRIVE_RELOAD2_SEQUENCE);
                break;

            case DRIVE_RELOAD2_SEQUENCE:
                driveCollect(driveReload2, new Pose(123.000, 66.000), PathState.DRIVE_RELOAD2_TO_OPEN_GATE);
                break;

            case DRIVE_RELOAD2_TO_OPEN_GATE:
                driveCollect(driveReload2ToOpenGate, new Pose(126.000, 70.000), PathState.DRIVE_OPEN_GATE_TO_SHOTPOS);
                break;

            case DRIVE_OPEN_GATE_TO_SHOTPOS:
                driveCollect(driveOpenGateToShotPos, new Pose(96.113, 95.686), PathState.SHOOT_POST_RELOAD_2);
                break;

            case SHOOT_POST_RELOAD_2:
                runShootStateThenAdvance(PathState.DRIVE_SHOT_TO_RELOAD3_START);
                break;

            case DRIVE_SHOT_TO_RELOAD3_START:
                driveCollect(driveShotToReload3Start, new Pose(95.000, 41.000), PathState.DRIVE_RELOAD3_SEQUENCE);
                break;

            case DRIVE_RELOAD3_SEQUENCE:
                driveCollect(driveReload3, new Pose(123.000, 41.000), PathState.DRIVE_RELOAD3_TO_SHOTPOS);
                break;

            case DRIVE_RELOAD3_TO_SHOTPOS:
                driveCollect(driveReload3ToShotPos, new Pose(89.000, 104.000), PathState.SHOOT_FINAL);
                break;

            case SHOOT_FINAL:
            default:
                runShootStateThenHold();
                break;
        }
    }

    // ==========================================================
    // DRIVE helper (FIXED: start once, require "saw busy" before completing)
    // ==========================================================
    private void driveCollect(PathChain path, Pose endPose, PathState next) {

        if (!driveStartedThisState) {
            follower.followPath(path, true);
            driveStartedThisState = true;
        }

        // record that follower actually started executing
        if (follower.isBusy()) driveSawBusy = true;

        shooterSys.update();
        startTurretSettle(getAutoAimTurretCmd());

        // collection while driving
        intake1.setPower(DRIVE_INTAKE1_PWR);
        intake2.setPower(DRIVE_INTAKE2_PWR);
        flicker.setPower(0.0);
        flipper.setPosition(0.662);

        updateReadyLatchNear(shootPose);

        // Early exit when close, BUT ONLY AFTER we've seen busy once
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

        if (follower.isBusy()) {
            feeder.stop();
            feederStarted = false;
            stopFeedHardware();
            return;
        }

        if (!isTurretSettled()) {
            feeder.stop();
            feederStarted = false;
            stopFeedHardware();
            return;
        }

        boolean tagNow = shooterSys.isTagSeen();
        boolean readyStable = shooterSys.isShooterReadyStable();
        boolean timeoutNoTag = shootGateTimer.seconds() >= NO_TAG_START_TIMEOUT_SEC;

        boolean okToStart = (tagNow && readyStable) || isReadyLatched() || (timeoutNoTag && readyStable);

        if (!feederStarted) {
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

        if (follower.isBusy() || !isTurretSettled()) {
            feeder.stop();
            feederStarted = false;
            stopFeedHardware();
            return;
        }

        boolean tagNow = shooterSys.isTagSeen();
        boolean readyStable = shooterSys.isShooterReadyStable();
        boolean timeoutNoTag = shootGateTimer.seconds() >= NO_TAG_START_TIMEOUT_SEC;

        boolean okToStart = (tagNow && readyStable) || isReadyLatched() || (timeoutNoTag && readyStable);

        if (!feederStarted) {
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

        // FIX: reset drive guards on every state change
        driveSawBusy = false;
        driveStartedThisState = false;
    }

    // ==========================================================
    // INIT / START / LOOP / STOP
    // ==========================================================
    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
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
        PIDFCoefficients pidf = new PIDFCoefficients(180, 0, 0, 15.202);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        shooterSys = new ShooterSubsystem(
                limelight, shooter, hood,
                0,
                -1, 24,
                8.0,
                265, 16.53,
                new double[]{16, 32, 48, 64, 80, 120},
                new double[]{0.310, 0.530, 0.730, 0.750, 0.780, 0.790},
                new double[]{1020, 1045, 1120, 1210, 1260, 1440},
                new double[]{1040, 1060, 1130, 1230, 1280, 1470},
                new double[]{1050, 1070, 1150, 1250, 1300, 1500},
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

        return Math.hypot(RED_GOAL_PX - turretX, RED_GOAL_PY - turretY);
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

        double bearingDegField = Math.toDegrees(Math.atan2(RED_GOAL_PY - turretY, RED_GOAL_PX - turretX));
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
            if (id == RED_GOAL_TAG_ID) { chosen = t; break; }
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
