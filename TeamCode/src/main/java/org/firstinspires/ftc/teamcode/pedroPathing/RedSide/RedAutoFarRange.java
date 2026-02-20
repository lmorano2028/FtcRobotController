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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.FieldTransform;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.LoaderSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.ShooterSubsystem;

import java.util.List;

@Autonomous(name = "RedAutoFarRange", group = "Red")
public class RedAutoFarRange extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private static final String LIMELIGHT_NAME = "limelight";
    private static final String SHOOTER_NAME   = "ShooterMotor";
    private static final String FLICKER_NAME   = "Outertake";
    private static final String INTAKE1_NAME   = "intakeOneMotor";
    private static final String INTAKE2_NAME   = "intakeTwoMotor";
    private static final String HOOD_NAME      = "Shooter hood";
    private static final String FLIPPER_NAME   = "fingler";

    // ======= TURRET AIMING (ODO + LL tx TRIM) =======
    private static final String TURRET_LEFT_NAME  = "LRotation";
    private static final String TURRET_RIGHT_NAME = "RRotation";
    private static final boolean MIRROR_RIGHT = false;
    private static final double TURRET_START_POS  = 0.54;

    private Servo turretLeft, turretRight;

    private static final double TURRET_SETTLE_SEC = 0.20;
    private boolean turretSettleStarted = false;
    private double turretTargetPos = TURRET_START_POS;
    private final com.qualcomm.robotcore.util.ElapsedTime turretSettleTimer = new com.qualcomm.robotcore.util.ElapsedTime();

    // ==========================================================
    // ODOMETRY + VISION TRIM TURRET AIM (PEDRO POSE, RED ONLY)
    // ==========================================================
    private static final double RED_GOAL_PX = 136;
    private static final double RED_GOAL_PY = 142;
    private static final int RED_GOAL_TAG_ID = 24;

    private static final double TURRET_HOME    = 0.50;
    private static final double POS_PER_DEG_CW = 0.007643;
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

    // (kept from your code)
    private static final double TURRET_BIAS_DEG_CW = -1.00;

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
    // Option B: "never deadlocks" gate when tag is missing
    // ==========================================================
    private static final double NO_TAG_START_TIMEOUT_SEC = 0.70;
    private final com.qualcomm.robotcore.util.ElapsedTime shootGateTimer = new com.qualcomm.robotcore.util.ElapsedTime();

    // ======= NEW: waits you requested (kept) =======
    private static final double RELOAD_START_WAIT_SEC = 0.25;
    private static final double SHOOTPOSE_WAIT_SEC    = 0.50;

    // ======= POSES (RED) =======
    private final Pose startPose = new Pose(84.000, 10.000, Math.toRadians(0));
    private final Pose shootPose = new Pose(89.000, 15.000, Math.toRadians(25));

    private final Pose firstReloadStart = new Pose(103.000, 35.000, Math.toRadians(0));
    private final Pose firstReloadEnd   = new Pose(125.0,   35.0,  Math.toRadians(0));

    private final Pose endPose = new Pose(92, 35, Math.toRadians(90));

    // ======= PATH CHAINS =======
    private PathChain driveStartToShootPose;

    private PathChain _3rdshotto3rdreloadstart;
    private PathChain _3rdreloadstarttoend;
    private PathChain _1stReloadToShootPos;

    private PathChain _ShotTo2ndReload;
    private PathChain _2ndReloadAgain;
    private PathChain _2ndReloadFinalRound;

    private PathChain _2ndReloadToShootPose;

    private PathChain driveOffLine;

    // Hardware
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

    // Volley control
    private boolean startedVolleyStart = false;
    private boolean finishedVolleyStart = false;
    private boolean endLoadStartedStart = false;

    private boolean startedVolley1 = false;
    private boolean finishedVolley1 = false;
    private boolean endLoadStarted = false;

    private boolean startedVolley2 = false;
    private boolean finishedVolley2 = false;
    private boolean endLoadStarted2 = false;

    private static final double PRIME_FEED_SEC  = 0.45;
    private static final double SETTLE_HOLD_SEC = 0.25;

    private static final double RELOAD_DECOMPRESS_SEC = 0.12;
    private boolean reload3DecompressStarted = false;
    private boolean reload2DecompressStarted = false;

    // Volley assist
    private boolean volleyAssistActive = false;
    private void setVolleyAssist(boolean on) {
        volleyAssistActive = on;
        if (intake != null) intake.setEnabled(on);
    }

    // ======= NEW: no init movement; do first servo+enable commands in start() only =======
    private boolean didStartCommands = false;

    // ==========================================================
    // States (UNCHANGED)
    // ==========================================================
    public enum PathState {
        DRIVE_START_TO_SHOOTPOSE,
        WAIT_AT_SHOOTPOSE,
        END_LOAD_AT_START,
        SHOOT_VOLLEY_START,

        DRIVE_3RDSHOT_TO_3RDRELOADSTART,
        WAIT_AT_RELOAD1_START,
        DRIVE_3RDRELOADSTART_TO_END,
        RELOAD3_DECOMPRESS,

        DRIVE_1STRELOAD_TO_SHOOTPOS,
        END_LOAD_AT_SHOOT_POS,
        SHOOT_VOLLEY_1,

        DRIVE_SHOT_TO_2NDRELOAD_START,
        WAIT_AT_RELOAD2_START,
        DRIVE_2NDRELOAD_AGAIN,
        DRIVE_2NDRELOAD_FINALROUND,
        RELOAD2_DECOMPRESS,

        DRIVE_2NDRELOAD_TO_SHOOTPOSE,
        END_LOAD_AT_SHOOT_POS_2,
        SHOOT_VOLLEY_2,

        FINAL_SHOT_TO_ENDPOSE,
        DONE
    }

    PathState pathState;

    // ==========================================================
    // PoseStorage force-write helper (matches RedAutoCloseRange12BallOdo)
    // ==========================================================
    private void writePoseStorageNow() {
        if (follower == null) return;

        Pose p = follower.getPose();
        double pedroX = p.getX();
        double pedroY = p.getY();
        double pedroHeadingDeg = Math.toDegrees(p.getHeading());

        FieldTransform.writePoseStorageFromPedro(
                pedroX,
                pedroY,
                pedroHeadingDeg
        );
    }

    // ==========================================================
    // Paths (UNCHANGED)
    // ==========================================================
    public void buildPaths() {

        driveStartToShootPose = follower.pathBuilder().addPath(
                        new BezierLine(startPose, shootPose)
                ).setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        _3rdshotto3rdreloadstart = follower.pathBuilder().addPath(
                        new BezierLine(startPose, firstReloadStart)
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        _3rdreloadstarttoend = follower.pathBuilder().addPath(
                        new BezierLine(firstReloadStart, firstReloadEnd)
                ).setTangentHeadingInterpolation()
                .build();

        _1stReloadToShootPos = follower.pathBuilder().addPath(
                        new BezierLine(firstReloadEnd, shootPose)
                ).setLinearHeadingInterpolation(Math.toRadians(0), shootPose.getHeading())
                .build();

        _ShotTo2ndReload = follower.pathBuilder().addPath(
                        new BezierLine(shootPose, new Pose(131.000, 15.000))
                ).setLinearHeadingInterpolation(shootPose.getHeading(), Math.toRadians(0))
                .build();

        _2ndReloadAgain = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(131.000, 15.000), new Pose(124.000, 11.000))
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        _2ndReloadFinalRound = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(124.000, 11.000), new Pose(131.000, 11.000))
                ).setTangentHeadingInterpolation()
                .build();

        _2ndReloadToShootPose = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(131.000, 11.000), shootPose)
                ).setLinearHeadingInterpolation(Math.toRadians(0), shootPose.getHeading())
                .build();

        driveOffLine = follower.pathBuilder().addPath(
                        new BezierLine(shootPose, endPose)
                ).setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    // ==========================================================
    // State machine (converted to CloseRange ODO+LL trim + ShooterSubsystem gating)
    // ==========================================================
    public void statePathUpdate() {
        switch (pathState) {

            case DRIVE_START_TO_SHOOTPOSE:
                // follower.followPath kicked in start()
                resetReadyLatch();
                if (follower.isBusy()) {
                    if (!volleyAssistActive) intake.setEnabled(false);
                    loader.hold();
                } else {
                    pathTimer.resetTimer();
                    setPathState(PathState.WAIT_AT_SHOOTPOSE);
                }
                break;

            case WAIT_AT_SHOOTPOSE:
                if (pathTimer.getElapsedTimeSeconds() < SHOOTPOSE_WAIT_SEC) {
                    if (!volleyAssistActive) intake.setEnabled(false);
                    loader.hold();
                    break;
                }
                endLoadStartedStart = false;
                startedVolleyStart = false;
                finishedVolleyStart = false;
                resetReadyLatch();
                setPathState(PathState.END_LOAD_AT_START);
                break;

            case END_LOAD_AT_START: {
                shooterSys.update();
                startTurretSettle(getAutoAimTurretCmd());

                if (follower.isBusy()) {
                    if (!volleyAssistActive) intake.setEnabled(false);
                    loader.hold();
                    updateReadyLatchNear(shootPose);
                    break;
                }

                if (!isTurretSettled()) {
                    if (!volleyAssistActive) intake.setEnabled(false);
                    loader.hold();
                    break;
                }

                boolean ready = shooterSys.isShooterReadyStable();

                if (!endLoadStartedStart) {
                    endLoadStartedStart = true;
                    pathTimer.resetTimer();
                }

                if (!ready) {
                    if (!volleyAssistActive) intake.setEnabled(false);
                    loader.hold();
                    break;
                }

                double t = pathTimer.getElapsedTimeSeconds();
                if (t < PRIME_FEED_SEC) {
                    if (!volleyAssistActive) intake.setEnabled(true);
                    loader.feedOn();
                } else if (t < (PRIME_FEED_SEC + SETTLE_HOLD_SEC)) {
                    if (!volleyAssistActive) intake.setEnabled(false);
                    loader.hold();
                } else {
                    loader.decompress();
                    if (!volleyAssistActive) intake.setEnabled(false);

                    shooterSys.resetReadyStable();
                    startedVolleyStart = false;
                    finishedVolleyStart = false;

                    setPathState(PathState.SHOOT_VOLLEY_START);
                }
                break;
            }

            case SHOOT_VOLLEY_START: {
                shooterSys.update();
                startTurretSettle(getAutoAimTurretCmd());

                if (finishedVolleyStart) {
                    setVolleyAssist(false);
                    loader.stopAll();

                    follower.followPath(_3rdshotto3rdreloadstart, true);
                    intake.setEnabled(true);
                    setPathState(PathState.DRIVE_3RDSHOT_TO_3RDRELOADSTART);
                    break;
                }

                if (!isTurretSettled()) {
                    loader.hold();
                    break;
                }

                boolean tagSeenNow = shooterSys.isTagSeen();
                boolean readyStable = shooterSys.isShooterReadyStable();
                boolean timeoutNoTag = shootGateTimer.seconds() >= NO_TAG_START_TIMEOUT_SEC;
                boolean okToStart = (tagSeenNow && readyStable) || isReadyLatched() || (timeoutNoTag && readyStable);

                if (!startedVolleyStart) loader.hold();

                if (!startedVolleyStart && shooterSys.isEnabled() && okToStart) {
                    startedVolleyStart = true;
                    finishedVolleyStart = false;
                    shooterSys.resetReadyStable();
                    loader.startFourShot();
                }

                if (startedVolleyStart && !finishedVolleyStart) {
                    loader.updateFourShot(shooterSys.isShooterReadyStable());

                    if (!volleyAssistActive && loader.getState() == LoaderSubsystem.SeqState.SHOT1_RECOVER) {
                        setVolleyAssist(true);
                    }

                    if (loader.getState() == LoaderSubsystem.SeqState.DONE) finishedVolleyStart = true;
                }
                break;
            }

            case DRIVE_3RDSHOT_TO_3RDRELOADSTART:
                if (follower.isBusy()) {
                    intake.setEnabled(true);
                } else {
                    pathTimer.resetTimer();
                    setPathState(PathState.WAIT_AT_RELOAD1_START);
                }
                break;

            case WAIT_AT_RELOAD1_START:
                if (pathTimer.getElapsedTimeSeconds() < RELOAD_START_WAIT_SEC) {
                    intake.setEnabled(true);
                    break;
                }
                follower.followPath(_3rdreloadstarttoend, true);
                intake.setEnabled(true);
                setPathState(PathState.DRIVE_3RDRELOADSTART_TO_END);
                break;

            case DRIVE_3RDRELOADSTART_TO_END:
                if (follower.isBusy()) {
                    intake.setEnabled(true);
                } else {
                    intake.setEnabled(false);
                    reload3DecompressStarted = false;
                    setPathState(PathState.RELOAD3_DECOMPRESS);
                }
                break;

            case RELOAD3_DECOMPRESS:
                if (!reload3DecompressStarted) {
                    reload3DecompressStarted = true;
                    pathTimer.resetTimer();
                    loader.decompress();
                }
                if (pathTimer.getElapsedTimeSeconds() >= RELOAD_DECOMPRESS_SEC) {
                    loader.hold();

                    follower.followPath(_1stReloadToShootPos, true);
                    resetReadyLatch();

                    endLoadStarted = false;
                    startedVolley1 = false;
                    finishedVolley1 = false;
                    setVolleyAssist(false);

                    setPathState(PathState.DRIVE_1STRELOAD_TO_SHOOTPOS);
                }
                break;

            case DRIVE_1STRELOAD_TO_SHOOTPOS:
                shooterSys.update();
                startTurretSettle(getAutoAimTurretCmd());

                if (follower.isBusy()) {
                    updateReadyLatchNear(shootPose);
                    intake.setEnabled(false);
                    loader.hold();
                } else {
                    endLoadStarted = false;
                    loader.stopAll();
                    setVolleyAssist(false);
                    setPathState(PathState.END_LOAD_AT_SHOOT_POS);
                }
                break;

            case END_LOAD_AT_SHOOT_POS: {
                shooterSys.update();
                startTurretSettle(getAutoAimTurretCmd());

                if (!isTurretSettled()) {
                    if (!volleyAssistActive) intake.setEnabled(false);
                    loader.hold();
                    break;
                }

                boolean readyEL = shooterSys.isShooterReadyStable();

                if (!endLoadStarted) {
                    endLoadStarted = true;
                    pathTimer.resetTimer();
                }

                if (!readyEL) {
                    if (!volleyAssistActive) intake.setEnabled(false);
                    loader.hold();
                    break;
                }

                double t = pathTimer.getElapsedTimeSeconds();
                if (t < PRIME_FEED_SEC) {
                    if (!volleyAssistActive) intake.setEnabled(true);
                    loader.feedOn();
                } else if (t < (PRIME_FEED_SEC + SETTLE_HOLD_SEC)) {
                    if (!volleyAssistActive) intake.setEnabled(false);
                    loader.hold();
                } else {
                    loader.decompress();
                    if (!volleyAssistActive) intake.setEnabled(false);

                    shooterSys.resetReadyStable();
                    startedVolley1 = false;
                    finishedVolley1 = false;

                    setPathState(PathState.SHOOT_VOLLEY_1);
                }
                break;
            }

            case SHOOT_VOLLEY_1: {
                shooterSys.update();
                startTurretSettle(getAutoAimTurretCmd());

                if (finishedVolley1) {
                    setVolleyAssist(false);
                    loader.stopAll();

                    follower.followPath(_ShotTo2ndReload, true);
                    intake.setEnabled(true);
                    setPathState(PathState.DRIVE_SHOT_TO_2NDRELOAD_START);
                    break;
                }

                if (!isTurretSettled()) {
                    loader.hold();
                    break;
                }

                boolean tagSeenNow = shooterSys.isTagSeen();
                boolean readyStable = shooterSys.isShooterReadyStable();
                boolean timeoutNoTag = shootGateTimer.seconds() >= NO_TAG_START_TIMEOUT_SEC;
                boolean okToStart = (tagSeenNow && readyStable) || isReadyLatched() || (timeoutNoTag && readyStable);

                if (!startedVolley1) loader.hold();

                if (!startedVolley1 && shooterSys.isEnabled() && okToStart) {
                    startedVolley1 = true;
                    finishedVolley1 = false;
                    shooterSys.resetReadyStable();
                    loader.startFourShot();
                }

                if (startedVolley1 && !finishedVolley1) {
                    loader.updateFourShot(shooterSys.isShooterReadyStable());

                    if (!volleyAssistActive && loader.getState() == LoaderSubsystem.SeqState.SHOT1_RECOVER) {
                        setVolleyAssist(true);
                    }

                    if (loader.getState() == LoaderSubsystem.SeqState.DONE) finishedVolley1 = true;
                }
                break;
            }

            case DRIVE_SHOT_TO_2NDRELOAD_START:
                if (follower.isBusy()) {
                    intake.setEnabled(true);
                } else {
                    pathTimer.resetTimer();
                    setPathState(PathState.WAIT_AT_RELOAD2_START);
                }
                break;

            case WAIT_AT_RELOAD2_START:
                if (pathTimer.getElapsedTimeSeconds() < RELOAD_START_WAIT_SEC) {
                    intake.setEnabled(true);
                    break;
                }
                follower.followPath(_2ndReloadAgain, true);
                intake.setEnabled(true);
                setPathState(PathState.DRIVE_2NDRELOAD_AGAIN);
                break;

            case DRIVE_2NDRELOAD_AGAIN:
                if (follower.isBusy()) {
                    intake.setEnabled(true);
                } else {
                    follower.followPath(_2ndReloadFinalRound, true);
                    intake.setEnabled(true);
                    setPathState(PathState.DRIVE_2NDRELOAD_FINALROUND);
                }
                break;

            case DRIVE_2NDRELOAD_FINALROUND:
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

                    follower.followPath(_2ndReloadToShootPose, true);
                    resetReadyLatch();

                    endLoadStarted2 = false;
                    startedVolley2 = false;
                    finishedVolley2 = false;
                    setVolleyAssist(false);

                    setPathState(PathState.DRIVE_2NDRELOAD_TO_SHOOTPOSE);
                }
                break;

            case DRIVE_2NDRELOAD_TO_SHOOTPOSE:
                shooterSys.update();
                startTurretSettle(getAutoAimTurretCmd());

                if (follower.isBusy()) {
                    intake.setEnabled(false);
                    loader.hold();
                    updateReadyLatchNear(shootPose);
                } else {
                    endLoadStarted2 = false;
                    loader.stopAll();
                    setVolleyAssist(false);
                    setPathState(PathState.END_LOAD_AT_SHOOT_POS_2);
                }
                break;

            case END_LOAD_AT_SHOOT_POS_2: {
                shooterSys.update();
                startTurretSettle(getAutoAimTurretCmd());

                if (!isTurretSettled()) {
                    if (!volleyAssistActive) intake.setEnabled(false);
                    loader.hold();
                    break;
                }

                boolean readyEL2 = shooterSys.isShooterReadyStable();

                if (!endLoadStarted2) {
                    endLoadStarted2 = true;
                    pathTimer.resetTimer();
                }

                if (!readyEL2) {
                    if (!volleyAssistActive) intake.setEnabled(false);
                    loader.hold();
                    break;
                }

                double t = pathTimer.getElapsedTimeSeconds();
                if (t < PRIME_FEED_SEC) {
                    if (!volleyAssistActive) intake.setEnabled(true);
                    loader.feedOn();
                } else if (t < (PRIME_FEED_SEC + SETTLE_HOLD_SEC)) {
                    if (!volleyAssistActive) intake.setEnabled(false);
                    loader.hold();
                } else {
                    loader.decompress();
                    if (!volleyAssistActive) intake.setEnabled(false);

                    shooterSys.resetReadyStable();
                    startedVolley2 = false;
                    finishedVolley2 = false;

                    setPathState(PathState.SHOOT_VOLLEY_2);
                }
                break;
            }

            case SHOOT_VOLLEY_2: {
                shooterSys.update();
                startTurretSettle(getAutoAimTurretCmd());

                if (finishedVolley2) {
                    startTurretSettle(0.5);
                    setVolleyAssist(false);
                    loader.stopAll();

                    follower.followPath(driveOffLine, true);
                    setPathState(PathState.FINAL_SHOT_TO_ENDPOSE);
                    break;
                }

                if (!isTurretSettled()) {
                    loader.hold();
                    break;
                }

                boolean tagSeenNow = shooterSys.isTagSeen();
                boolean readyStable = shooterSys.isShooterReadyStable();
                boolean timeoutNoTag = shootGateTimer.seconds() >= NO_TAG_START_TIMEOUT_SEC;
                boolean okToStart = (tagSeenNow && readyStable) || isReadyLatched() || (timeoutNoTag && readyStable);

                if (!startedVolley2) loader.hold();

                if (!startedVolley2 && shooterSys.isEnabled() && okToStart) {
                    startedVolley2 = true;
                    finishedVolley2 = false;
                    shooterSys.resetReadyStable();
                    loader.startFourShot();
                }

                if (startedVolley2 && !finishedVolley2) {
                    loader.updateFourShot(shooterSys.isShooterReadyStable());

                    if (!volleyAssistActive && loader.getState() == LoaderSubsystem.SeqState.SHOT1_RECOVER) {
                        setVolleyAssist(true);
                    }

                    if (loader.getState() == LoaderSubsystem.SeqState.DONE) finishedVolley2 = true;
                }
                break;
            }

            case FINAL_SHOT_TO_ENDPOSE:
                if (!follower.isBusy()) {
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                // final stationary; keep turret calm and seed pose once more
                startTurretSettle(0.5);
                writePoseStorageNow();
                break;

            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        shootGateTimer.reset();
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_START_TO_SHOOTPOSE;

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

        // ===== NO SERVO MOVEMENT IN INIT =====
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

        intake = new IntakeSubsystem(intake1, -1.0, -1.0);
        intake.setEnabled(false);

        loader = new LoaderSubsystem(
                intake2, flicker, flipper, intake,
                0.662, 0.39,
                0.12, 0.05,
                1.0, 1.0, 150,
                -0.15, 40,
                120
        );

        // ===== ShooterSubsystem (same operating model as CloseRange12BallOdo) =====
        shooterSys = new ShooterSubsystem(
                limelight, shooter, hood,
                0,
                -1, 24,
                8.0,
                180, 15.5022,
                new double[]{16, 32, 48, 64, 80, 120},
                new double[]{0.310, 0.530, 0.730, 0.750, 0.780, 0.790},
                new double[]{1020, 1045, 1110, 1180, 1260, 1480},
                new double[]{1040, 1060, 1120, 1200, 1280, 1490},
                new double[]{1050, 1070, 1130, 1210, 1300, 1550},
                110
        );


        shooterSys.startVision();
        shooterSys.setEnabled(false); // no motion in init

        endLoadStartedStart = false;
        startedVolleyStart = false;
        finishedVolleyStart = false;

        endLoadStarted = false;
        startedVolley1 = false;
        finishedVolley1 = false;

        endLoadStarted2 = false;
        startedVolley2 = false;
        finishedVolley2 = false;

        reload3DecompressStarted = false;
        reload2DecompressStarted = false;

        resetReadyLatch();

        buildPaths();
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

        if (!didStartCommands) {
            applyTurret(TURRET_START_POS);
            turretTargetPos = TURRET_START_POS;
            turretSettleStarted = false;
            turretSettleTimer.reset();

            flipper.setPosition(0.662);

            // shooter runs from start (and stays on through drive legs)
            shooterSys.setEnabled(true);

            didStartCommands = true;
        }

        // Kick off the initial path immediately
        follower.followPath(driveStartToShootPose, true);
    }

    @Override
    public void loop() {
        follower.update();

        // ===== Option B: feed odometry distance to shooter (vision used when available; odom when not) =====
        shooterSys.setExternalDistanceIn(getAutoAimShooterDistanceIn());

        shooterSys.update();

        updateVisionTrim();

        Pose p = follower.getPose();
        double pedroX = p.getX();
        double pedroY = p.getY();
        double pedroHeadingDeg = Math.toDegrees(p.getHeading());

        // ===== single source of truth for Pedro -> FTC PoseStorage seeding =====
        FieldTransform.writePoseStorageFromPedro(pedroX, pedroY, pedroHeadingDeg);

        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());

        telemetry.addData("turret target", "%.3f", turretTargetPos);
        telemetry.addData("turret settled", isTurretSettled());

        telemetry.addData("Goal(Pedro)", "(%.2f, %.2f)", RED_GOAL_PX, RED_GOAL_PY);
        telemetry.addData("Vision TagSeen", tagSeen);
        telemetry.addData("Vision TagId", tagId);
        telemetry.addData("Vision tx(deg)", Double.isNaN(txDeg) ? "N/A" : String.format("%.2f", txDeg));
        telemetry.addData("TrimRaw CW(deg)", "%.2f", visionTrimDegCW_raw);
        telemetry.addData("TrimFilt CW(deg)", "%.2f", visionTrimDegCW);

        telemetry.addData("Seed FTC X(in)", "%.2f", PoseStorage.xIn);
        telemetry.addData("Seed FTC Y(in)", "%.2f", PoseStorage.yIn);
        telemetry.addData("Seed FTC H(deg)", "%.1f", PoseStorage.headingDeg);

        telemetry.addData("VolleyAssist", volleyAssistActive);
        telemetry.addData("Shooter TagSeen", shooterSys.isTagSeen());
        telemetry.addData("Shooter Dist(in)", "%.2f", shooterSys.getShooterDistanceIn());
        telemetry.addData("Shooter RPM Tgt", "%.0f", shooterSys.getRpmTargetCmd());
        telemetry.addData("Shooter HoodCmd", "%.3f", shooterSys.getHoodCmd());
    }

    @Override
    public void stop() {
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
    // Ready latch helpers (matches CloseRange: must be READY + TAG near shot pose)
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

    // ==========================================================
    // AutoAim shooter distance from Pedro pose (matches turret reference point)
    // ==========================================================
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

        double bearingDegField = Math.toDegrees(Math.atan2(RED_GOAL_PY - turretY, RED_GOAL_PX - turretX));
        bearingDegField = wrapDeg180(bearingDegField);

        double headingDeg = wrapDeg180(Math.toDegrees(headingRad));

        double relDegCCW = wrapDeg180(bearingDegField - headingDeg);
        double turretDegCW_odo = -relDegCCW;

        double turretDegCW_total = wrapDeg180(turretDegCW_odo + visionTrimDegCW + TURRET_BIAS_DEG_CW);

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
    // Vision trim update (RED TAG 24 ONLY)
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
            if (id == RED_GOAL_TAG_ID) {
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