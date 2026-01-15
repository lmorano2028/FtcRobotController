package org.firstinspires.ftc.teamcode.pedroPathing.BlueSide;

import com.pedropathing.follower.Follower;
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
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.LoaderSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.ShooterSubsystem;

import java.util.List;

@Autonomous(name = "BlueAutoCloseRange", group = "Pedro")
public class BlueAutoCloseRangeRecordTesting extends OpMode {

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

    // turret settle gate
    private static final double TURRET_SETTLE_SEC = 0.20;
    private boolean turretSettleStarted = false;
    private double turretTargetPos = TURRET_START_POS;

    // ==========================================================
    // ODOMETRY + VISION TRIM TURRET AIM (PEDRO POSE, BLUE ONLY)
    // ==========================================================
    private static final double BLUE_GOAL_PX = 0;//original 16.3575
    private static final double BLUE_GOAL_PY = 144;//original 130.3727

    private static final int BLUE_GOAL_TAG_ID = 20;

    private static final double TURRET_HOME    = 0.50;
    private static final double POS_PER_DEG_CW = 0.007643;   // tuned
    private static final double SERVO_MIN_SAFE = 0.10;
    private static final double SERVO_MAX_SAFE = 0.90;

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
    // Pre-shoot readiness latch (pre-check while driving; never fire until stopped)
    // ==========================================================
    private static final double READY_LATCH_HOLD_SEC = 0.50;
    private static final double PRECHECK_DIST_IN = 10.0;

    private boolean readyLatched = false;
    private final com.qualcomm.robotcore.util.ElapsedTime readyLatchTimer = new com.qualcomm.robotcore.util.ElapsedTime();

    // ==========================================================
    // PoseStorage seeding (Pedro -> FTC) for TeleOp
    // ==========================================================
    private static final double FIELD_HALF_IN = 72.0;
    private static final double PEDRO_TO_FTC_HEADING_OFFSET_DEG = +90.0;

    private static double pedroToFtcX(double pedroY) { return FIELD_HALF_IN - pedroY; }
    private static double pedroToFtcY(double pedroX) { return pedroX - FIELD_HALF_IN; }

    // ==========================================================

    // ===== NEW PATH SET (YOUR PROVIDED SEQUENCE) =====
    public static class Paths {
        public PathChain StarttoFirstShot;
        public PathChain FirstShotto2ndReloadStart;
        public PathChain SecondReloadStarttoEnd;
        public PathChain SecondReloadto2ndShot;
        public PathChain SecondShottoGateOpenReload;
        public PathChain GateOpenReloadto3rdShot;

        public Paths(Follower follower) {
            StarttoFirstShot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20.571, 122.849, Math.toRadians(138)),
                                    new Pose(47.88,  96.000, Math.toRadians(138))
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(138))
                    .build();

            FirstShotto2ndReloadStart = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.500, 80.000, Math.toRadians(180)),
                                    new Pose(45.000, 63.000, Math.toRadians(180))
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            SecondReloadStarttoEnd = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(45.000, 63.000, Math.toRadians(180)),
                                    new Pose(24.000, 63.000, Math.toRadians(180))
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            SecondReloadto2ndShot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.000, 63.000, Math.toRadians(180)),
                                    new Pose(56.454, 80.375, Math.toRadians(143.5))
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143.5))
                    .build();

            SecondShottoGateOpenReload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.454, 80.375, Math.toRadians(143.5)),
                                    new Pose(10.450, 61.188, Math.toRadians(143.5))
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(143.5))
                    .build();

            GateOpenReloadto3rdShot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(10.450, 61.188, Math.toRadians(143.5)),
                                    new Pose(56.500, 80.000, Math.toRadians(143.5))
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(143.5))
                    .build();
        }
    }

    private Paths paths;

    // ====== Poses for readiness checks ======
    private final Pose startPose = new Pose(20.571, 122.849, Math.toRadians(138));
    private final Pose firstShotPose = new Pose(56.500, 80.000, Math.toRadians(138));
    private final Pose secondShotPose = new Pose(56.454, 80.375, Math.toRadians(143.5));
    private final Pose thirdShotPose = new Pose(56.500, 80.000, Math.toRadians(143.5));

    // ======= STATE MACHINE (updated to match new path order) =======
    public enum PathState {
        DRIVE_START_TO_FIRST_SHOT,
        SHOOT_FIRST,

        DRIVE_FIRST_SHOT_TO_RELOAD2_START,
        DRIVE_RELOAD2_SWEEP,
        RELOAD2_DECOMPRESS,

        DRIVE_RELOAD2_TO_SECOND_SHOT,
        SHOOT_SECOND,

        DRIVE_SECOND_SHOT_TO_GATEOPEN_RELOAD,

        // ===== NEW: 2s reload pause at GateOpenReload =====
        PAUSE_GATEOPEN_RELOAD,

        DRIVE_GATEOPEN_RELOAD_TO_THIRD_SHOT,
        SHOOT_THIRD
    }

    private PathState pathState;

    private Limelight3A limelight;
    private DcMotorEx shooter;
    private DcMotorEx flicker;
    private DcMotorEx intake1;
    private DcMotorEx intake2;
    private Servo hood;
    private Servo flipper;

    private IntakeSubsystem intake;
    private LoaderSubsystem loader;
    private ShooterSubsystem shooterSys;

    // Reuse your existing volley flags (kept same style)
    private boolean startedVolley = false;
    private boolean finishedVolley = false;

    private boolean endLoadStarted = false;

    private static final double PRIME_FEED_SEC  = 0.45;
    private static final double SETTLE_HOLD_SEC = 0.25;

    private static final double RELOAD_DECOMPRESS_SEC = 0.12;
    private boolean reload2DecompressStarted = false;

    // Volley feed assist (kept)
    private boolean volleyAssistActive = false;
    private void setVolleyAssist(boolean on) {
        volleyAssistActive = on;
        if (intake != null) intake.setEnabled(on);
    }

    // ===== NEW: ensure no init movement; do first commands in start() only =====
    private boolean didStartCommands = false;

    // ===== NEW: GateOpen reload pause time =====
    private static final double GATEOPEN_RELOAD_PAUSE_SEC = 2.0;

    @Override
    public void init() {
        pathState = PathState.DRIVE_START_TO_FIRST_SHOT;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        shooter   = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);
        flicker   = hardwareMap.get(DcMotorEx.class, FLICKER_NAME);
        intake1   = hardwareMap.get(DcMotorEx.class, INTAKE1_NAME);
        intake2   = hardwareMap.get(DcMotorEx.class, INTAKE2_NAME);
        hood      = hardwareMap.get(Servo.class, HOOD_NAME);
        flipper   = hardwareMap.get(Servo.class, FLIPPER_NAME);

        turretLeft  = hardwareMap.get(Servo.class, TURRET_LEFT_NAME);
        turretRight = hardwareMap.get(Servo.class, TURRET_RIGHT_NAME);

        // ===== CHANGED: NO SERVO COMMANDS IN INIT =====
        turretTargetPos = TURRET_START_POS;
        turretSettleStarted = false;

        lastTurretCmd = clamp(TURRET_START_POS, SERVO_MIN_SAFE, SERVO_MAX_SAFE);
        loopTimer.reset();

        visionTrimDegCW = 0.0;
        visionTrimDegCW_raw = 0.0;
        visionHasGood = false;
        visionLastGoodTimer.reset();
        visionDecayTimer.reset();

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidf = new PIDFCoefficients(265, 0, 0, 16.53);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        intake = new IntakeSubsystem(intake1, -0.8, -1.0);

        // KEEP: safe to command OFF in init to prevent accidental motion from prior opmode
        intake.setEnabled(false);

        // ===== CHANGED: Increased recover/hold powers (intake2 + flicker) =====
        loader = new LoaderSubsystem(
                intake2, flicker, flipper, intake,
                0.662, 0.39,
                0.24, 0.12,
                1.0, 1.0, 150,
                -0.15, 40,
                120
        );

        shooterSys = new ShooterSubsystem(
                limelight, shooter, hood,
                0,
                20, 24,
                8.0,
                265, 16.53,
                new double[]{24,48,80,120},
                new double[]{0.350,0.525,0.790,0.790},
                new double[]{970,1070,1150,1400},
                new double[]{1000,1100,1180,1440},
                new double[]{1030,1130,1210,1470},
                110
        );

        shooterSys.startVision();

        // ===== CHANGED: DO NOT ENABLE SHOOTER IN INIT (prevents hood movement) =====
        shooterSys.setEnabled(false);

        startedVolley = false;
        finishedVolley = false;
        endLoadStarted = false;

        reload2DecompressStarted = false;

        resetReadyLatch();

        // Build new paths
        paths = new Paths(follower);

        follower.setPose(startPose);

        setVolleyAssist(false);

        didStartCommands = false;

        telemetry.addLine("Ready. Waiting for start.");
        telemetry.update();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);

        // ===== NEW: FIRST AND ONLY SERVO POSITION COMMANDS HAPPEN HERE (AFTER START) =====
        if (!didStartCommands) {
            applyTurret(TURRET_START_POS);
            turretTargetPos = TURRET_START_POS;
            turretSettleStarted = false;

            flipper.setPosition(0.662);

            // ===== CHANGED: ENABLE SHOOTER ONLY AFTER START =====
            shooterSys.setEnabled(true);

            didStartCommands = true;
        }
    }

    @Override
    public void loop() {
        follower.update();
        shooterSys.update();

        updateVisionTrim();

        // PoseStorage seeding (unchanged)
        Pose p = follower.getPose();
        double pedroX = p.getX();
        double pedroY = p.getY();
        double pedroHeadingDeg = Math.toDegrees(p.getHeading());

        PoseStorage.valid = true;
        PoseStorage.xIn = pedroToFtcX(pedroY);
        PoseStorage.yIn = pedroToFtcY(pedroX);
        PoseStorage.headingDeg = wrapDeg180(pedroHeadingDeg + PEDRO_TO_FTC_HEADING_OFFSET_DEG);

        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("turret target", "%.3f", turretTargetPos);
        telemetry.addData("turret settled", isTurretSettled());

        telemetry.addData("Goal(Pedro)", "(%.2f, %.2f)", BLUE_GOAL_PX, BLUE_GOAL_PY);
        telemetry.addData("Vision TagSeen", tagSeen);
        telemetry.addData("Vision TagId", tagId);
        telemetry.addData("Vision tx(deg)", Double.isNaN(txDeg) ? "N/A" : String.format("%.2f", txDeg));
        telemetry.addData("TrimRaw CW(deg)", "%.2f", visionTrimDegCW_raw);
        telemetry.addData("TrimFilt CW(deg)", "%.2f", visionTrimDegCW);

        telemetry.addData("Seed FTC X(in)", "%.2f", PoseStorage.xIn);
        telemetry.addData("Seed FTC Y(in)", "%.2f", PoseStorage.yIn);
        telemetry.addData("Seed FTC H(deg)", "%.1f", PoseStorage.headingDeg);

        telemetry.addData("VolleyAssist", volleyAssistActive);
    }

    // ==========================================================
    // NEW STATE FLOW using your new paths, but SAME shoot routine
    // ==========================================================
    private void statePathUpdate() {
        switch (pathState) {

            case DRIVE_START_TO_FIRST_SHOT:
                follower.followPath(paths.StarttoFirstShot, true);
                resetReadyLatch();
                setVolleyAssist(false);
                setPathState(PathState.SHOOT_FIRST);
                break;

            case SHOOT_FIRST:
                if (runShootRoutine(firstShotPose)) {
                    follower.followPath(paths.FirstShotto2ndReloadStart, true);
                    intake.setEnabled(true);
                    setPathState(PathState.DRIVE_FIRST_SHOT_TO_RELOAD2_START);
                }
                break;

            case DRIVE_FIRST_SHOT_TO_RELOAD2_START:
                if (follower.isBusy()) {
                    intake.setEnabled(true);
                } else {
                    follower.followPath(paths.SecondReloadStarttoEnd, true);
                    intake.setEnabled(true);
                    setPathState(PathState.DRIVE_RELOAD2_SWEEP);
                }
                break;

            case DRIVE_RELOAD2_SWEEP:
                if (follower.isBusy()) {
                    intake.setEnabled(true);
                } else {
                    intake.setEnabled(false);
                    reload2DecompressStarted = false;
                    setPathState(PathState.RELOAD2_DECOMPRESS);
                }
                break;

            case RELOAD2_DECOMPRESS:
                if (!reload2DecompressStarted) {
                    reload2DecompressStarted = true;
                    pathTimer.resetTimer();
                    loader.decompress();
                }
                if (pathTimer.getElapsedTimeSeconds() >= RELOAD_DECOMPRESS_SEC) {
                    loader.hold();
                    follower.followPath(paths.SecondReloadto2ndShot, true);
                    resetReadyLatch();
                    setPathState(PathState.DRIVE_RELOAD2_TO_SECOND_SHOT);
                }
                break;

            case DRIVE_RELOAD2_TO_SECOND_SHOT:
                shooterSys.update();
                startTurretSettle(getAutoAimTurretCmd());

                if (follower.isBusy()) {
                    updateReadyLatchNear(secondShotPose);
                    intake.setEnabled(false);
                    loader.hold();
                } else {
                    // arrive: shoot again BEFORE next path (as requested)
                    setVolleyAssist(false);
                    startedVolley = false;
                    finishedVolley = false;
                    endLoadStarted = false;
                    loader.stopAll();

                    setPathState(PathState.SHOOT_SECOND);
                }
                break;

            case SHOOT_SECOND:
                if (runShootRoutine(secondShotPose)) {
                    follower.followPath(paths.SecondShottoGateOpenReload, true);
                    intake.setEnabled(true);
                    setPathState(PathState.DRIVE_SECOND_SHOT_TO_GATEOPEN_RELOAD);
                }
                break;

            case DRIVE_SECOND_SHOT_TO_GATEOPEN_RELOAD:
                if (follower.isBusy()) {
                    intake.setEnabled(true);
                } else {
                    // ===== NEW: 2 second pause here with IntakeOneMotor on =====
                    pathTimer.resetTimer();
                    intake.setEnabled(true);
                    setPathState(PathState.PAUSE_GATEOPEN_RELOAD);
                }
                break;

            case PAUSE_GATEOPEN_RELOAD:
                // Wait 2 seconds at GateOpenReload while IntakeOneMotor is ON
                intake.setEnabled(true);

                if (pathTimer.getElapsedTimeSeconds() >= GATEOPEN_RELOAD_PAUSE_SEC) {
                    follower.followPath(paths.GateOpenReloadto3rdShot, true);
                    resetReadyLatch();
                    setPathState(PathState.DRIVE_GATEOPEN_RELOAD_TO_THIRD_SHOT);
                }
                break;

            case DRIVE_GATEOPEN_RELOAD_TO_THIRD_SHOT:
                shooterSys.update();
                startTurretSettle(getAutoAimTurretCmd());

                if (follower.isBusy()) {
                    updateReadyLatchNear(thirdShotPose);
                    intake.setEnabled(false);
                    loader.hold();
                } else {
                    setVolleyAssist(false);
                    startedVolley = false;
                    finishedVolley = false;
                    endLoadStarted = false;
                    loader.stopAll();

                    setPathState(PathState.SHOOT_THIRD);
                }
                break;

            case SHOOT_THIRD:
                // Final shoot after GateOpenReloadto3rdShot (as requested)
                runShootRoutine(thirdShotPose);
                // stay here (no further path)
                break;

            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }

    /**
     * SAME shooting behavior you already had:
     * - update shooter
     * - turret settle gate
     * - ready latch ok
     * - optional end-load prime/feed/hold/decompress
     * - then loader.startFourShot / updateFourShot
     *
     * Returns true when volley finished.
     */
    private boolean runShootRoutine(Pose shotPoseRef) {
        shooterSys.update();
        startTurretSettle(getAutoAimTurretCmd());

        // If still driving (shouldn't be in these states, but safe)
        if (follower.isBusy()) {
            if (!volleyAssistActive) intake.setEnabled(false);
            loader.hold();
            updateReadyLatchNear(shotPoseRef);
            return false;
        }

        if (!isTurretSettled()) {
            if (!volleyAssistActive) intake.setEnabled(false);
            loader.hold();
            return false;
        }

        // Prime/load at shot position (kept same style as your END_LOAD states)
        boolean tagSeenEL = shooterSys.isTagSeen();
        boolean readyEL = shooterSys.isShooterReadyStable();

        if (!endLoadStarted) {
            endLoadStarted = true;
            pathTimer.resetTimer();
        }

        // Require tag+ready during prime phase (same as your earlier END_LOAD logic)
        if (!tagSeenEL || !readyEL) {
            if (!volleyAssistActive) intake.setEnabled(false);
            loader.hold();
            return false;
        }

        double t = pathTimer.getElapsedTimeSeconds();
        if (t < PRIME_FEED_SEC) {
            if (!volleyAssistActive) intake.setEnabled(true);
            loader.feedOn();
            return false;
        } else if (t < (PRIME_FEED_SEC + SETTLE_HOLD_SEC)) {
            if (!volleyAssistActive) intake.setEnabled(false);
            loader.hold();
            return false;
        } else {
            // ===== CHANGED: Removed prime-phase decompress so decompress is only shot 1 =====
            loader.hold();
            if (!volleyAssistActive) intake.setEnabled(false);
        }

        // Now do the four-shot volley (same gating as before)
        boolean tagSeen = shooterSys.isTagSeen();
        boolean readyStable = shooterSys.isShooterReadyStable();
        boolean okToStart = (tagSeen && readyStable) || isReadyLatched();

        if (!startedVolley) loader.hold();

        if (!startedVolley && shooterSys.isEnabled() && okToStart) {
            startedVolley = true;
            finishedVolley = false;
            shooterSys.resetReadyStable();
            loader.startFourShot();
        }

        if (startedVolley && !finishedVolley) {
            loader.updateFourShot(shooterSys.isShooterReadyStable());

            if (!volleyAssistActive && loader.getState() == LoaderSubsystem.SeqState.SHOT1_RECOVER) {
                setVolleyAssist(true);
            }

            if (loader.getState() == LoaderSubsystem.SeqState.DONE) {
                finishedVolley = true;
            }
        }

        if (finishedVolley) {
            setVolleyAssist(false);
            loader.stopAll();
            // reset for next shoot call
            startedVolley = false;
            finishedVolley = false;
            endLoadStarted = false;
            return true;
        }

        return false;
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
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
            pathTimer.resetTimer();
        }
    }

    private boolean isTurretSettled() {
        return turretSettleStarted && pathTimer.getElapsedTimeSeconds() >= TURRET_SETTLE_SEC;
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
        double dx = a.getX() - a.getX();
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

    // ==========================================================
    // AutoAim turret command from Pedro pose + LL tx trim
    // ==========================================================
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

    // ==========================================================
    // Vision trim update (BLUE TAG 20 ONLY)
    // ==========================================================
    private void updateVisionTrim() {
        tagSeen = false;
        tagId = -1;
        camX_in = Double.NaN;
        camZ_in = Double.NaN;
        camBearingDeg = Double.NaN;
        txDeg = Double.NaN;

        if (limelight == null) {
            handleVisionLost();
            return;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            handleVisionLost();
            return;
        }

        try { txDeg = result.getTx(); } catch (Exception ignored) { txDeg = Double.NaN; }

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) {
            handleVisionLost();
            return;
        }

        LLResultTypes.FiducialResult chosen = null;
        for (LLResultTypes.FiducialResult t : tags) {
            int id = (int) t.getFiducialId();
            if (id == BLUE_GOAL_TAG_ID) {
                chosen = t;
                break;
            }
        }
        if (chosen == null) {
            handleVisionLost();
            return;
        }

        Pose3D pose = null;
        try { pose = chosen.getCameraPoseTargetSpace(); } catch (Exception ignored) {}

        if (pose == null) {
            handleVisionLost();
            return;
        }

        Position p = pose.getPosition();

        double xIn = DistanceUnit.INCH.fromUnit(p.unit, p.x);
        double zIn = DistanceUnit.INCH.fromUnit(p.unit, p.z);
        double zAbs = Math.abs(zIn);

        if (zAbs < VISION_MIN_Z_IN || zAbs > VISION_MAX_Z_IN) {
            handleVisionLost();
            return;
        }

        if (Double.isNaN(txDeg) || Math.abs(txDeg) > VISION_MAX_ABS_TX_DEG) {
            handleVisionLost();
            return;
        }

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

        if (msSince <= VISION_HOLD_MS) {
            return;
        }

        double t = clamp(visionDecayTimer.milliseconds() / (double) VISION_DECAY_MS, 0.0, 1.0);
        visionTrimDegCW = (1.0 - t) * visionTrimDegCW;

        if (t >= 1.0) {
            visionTrimDegCW = 0.0;
            visionTrimDegCW_raw = 0.0;
            visionHasGood = false;
        }
    }

    // ==========================================================
    // Small math/servo helpers
    // ==========================================================
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
