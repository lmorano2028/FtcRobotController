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
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.LoaderSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.ShooterSubsystem;

import java.util.List;

@Autonomous
public class BlueAutoCloseRange12Ball extends OpMode {
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
    private static final double TURRET_SHOT1_POS  = 0.54;   // (kept, unused now)
    private static final double TURRET_SHOT23_POS = 0.836;  // (kept, unused now)
    private static final double TURRET_SHOT4_POS  = 0.1;    // (kept, unused now)

    private Servo turretLeft, turretRight;

    // NEW: turret settle gate (ensures turret is at target BEFORE we rev/feed/shoot)
    private static final double TURRET_SETTLE_SEC = 0.20;
    private boolean turretSettleStarted = false;
    private double turretTargetPos = TURRET_START_POS;

    // ==========================================================
    // ODOMETRY + VISION TRIM TURRET AIM (PEDRO POSE, BLUE ONLY)
    // ==========================================================
    private static final double BLUE_GOAL_PX = 0; // original 16.3575
    private static final double BLUE_GOAL_PY = 144; //original 130.3727

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

    public enum PathState {
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,

        DRIVE_SHOOTPOS_RELOAD3POS,
        DRIVE_THIRD_RELOAD_SEQUENCE,
        RELOAD_DECOMPRESS,
        DRIVE_RELOAD_TO_SHOT,
        END_LOAD_AT_SHOOT_POS,
        SHOOT_POST_RELOAD,

        DRIVE_SHOT_TO_RELOAD2_START,
        DRIVE_RELOAD2_SEQUENCE,
        RELOAD2_DECOMPRESS,

        // ===== NEW ROUTE AFTER RELOAD2 END (gate path) =====
        DRIVE_RELOAD2_TO_OPEN_GATE,
        DRIVE_GATE_PATH12,
        DRIVE_OPEN_GATE_TO_SHOTPOS,

        // back to your existing flow
        DRIVE_RELOAD2_TO_SHOT,
        END_LOAD_AT_SHOOT_POS_2,
        SHOOT_POST_RELOAD_2,

        DRIVE_SHOT_TO_RELOAD3_START,
        DRIVE_RELOAD3_SEQUENCE,
        RELOAD3_DECOMPRESS,

        // ===== CHANGED: reload3 returns directly to shotpos (Path10) =====
        DRIVE_RELOAD3_TO_SHOTPOS,

        END_LOAD_AT_FINAL_SHOT,
        SHOOT_FINAL
    }

    PathState pathState;

    private final Pose startPose = new Pose(20.571428571428577, 122.84909456740445, Math.toRadians(138));
    private final Pose shootPose = new Pose(47.88732394366197,   95.68611670020124,  Math.toRadians(138));
    private final Pose shootPoseWithoutHeading = new Pose(47.88732394366197, 95.68611670020124, Math.toRadians(180));

    private final Pose thirdReloadStart = new Pose(44, 87, Math.toRadians(180));
    private final Pose thirdReloadEnd   = new Pose(23, 87, Math.toRadians(180));

    private final Pose secondReloadStart = new Pose(48.000, 63.000, Math.toRadians(180));
    private final Pose secondReloadEnd   = new Pose(24.000, 63.000, Math.toRadians(180));
    private final Pose openGatePos = new Pose(23.000, 70.000, Math.toRadians(-90));

    private final Pose thirdReloadStart3 = new Pose(48.000, 41.000, Math.toRadians(180));
    private final Pose thirdReloadEnd3   = new Pose(23.000, 41.000, Math.toRadians(180));

    private final Pose finalShotPose = new Pose(55, 106, Math.toRadians(180));

    private PathChain driveStartPosShootPos, driveShootPosReloadPos, driveReloadThree, driveReloadPosToShootPos;
    private PathChain driveShotToReload2Start, driveReload2;
    private PathChain driveReload2ToOpenGate, driveGatePath12, driveOpenGateToShotPos;
    private PathChain driveShotToReload3Start, driveReload3, driveReload3ToShotPos;

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

    private boolean startedFourShot = false;
    private boolean finishedFourShot = false;

    private boolean endLoadStarted = false;
    private boolean startedFourShot2 = false;
    private boolean finishedFourShot2 = false;

    private boolean endLoadStarted2 = false;
    private boolean startedFourShot3 = false;
    private boolean finishedFourShot3 = false;

    private boolean endLoadFinalStarted = false;
    private boolean startedFinal = false;
    private boolean finishedFinal = false;

    private static final double PRIME_FEED_SEC  = 0.45;
    private static final double SETTLE_HOLD_SEC = 0.25;

    private static final double RELOAD_DECOMPRESS_SEC = 0.12;
    private boolean reloadDecompressStarted = false;
    private boolean reload2DecompressStarted = false;
    private boolean reload3DecompressStarted = false;

    // ==========================================================
    // NEW: Volley feed assist (Intake1 forced ON from after shot1 until volley ends)
    // ==========================================================
    private boolean volleyAssistActive = false;

    private void setVolleyAssist(boolean on) {
        volleyAssistActive = on;
        if (intake != null) intake.setEnabled(on);
    }
    // ==========================================================

    // ===== NEW: no init movement; do first servo+enable commands in start() only =====
    private boolean didStartCommands = false;

    public void buildPaths() {
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosReloadPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, thirdReloadStart))
                .setLinearHeadingInterpolation(shootPose.getHeading(), thirdReloadStart.getHeading())
                .build();

        driveReloadThree = follower.pathBuilder()
                .addPath(new BezierLine(thirdReloadStart, thirdReloadEnd))
                .setLinearHeadingInterpolation(thirdReloadStart.getHeading(), thirdReloadEnd.getHeading())
                .build();

        driveReloadPosToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(thirdReloadEnd, shootPoseWithoutHeading))
                .setLinearHeadingInterpolation(thirdReloadEnd.getHeading(), shootPoseWithoutHeading.getHeading())
                .build();

        driveShotToReload2Start = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, secondReloadStart))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        driveReload2 = follower.pathBuilder()
                .addPath(new BezierLine(secondReloadStart, secondReloadEnd))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        driveReload2ToOpenGate = follower.pathBuilder()
                .addPath(new BezierLine(
                        secondReloadEnd,
                        openGatePos
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-90))
                .build();

        driveGatePath12 = follower.pathBuilder()
                .addPath(new BezierLine(
                        openGatePos,
                        new Pose(32.000, 80.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                .build();

        driveOpenGateToShotPos = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(32.000, 80.000),
                        shootPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180))
                .build();

        driveShotToReload3Start = follower.pathBuilder()
                .addPath(new BezierLine(
                        shootPose,
                        thirdReloadStart3
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        driveReload3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        thirdReloadStart3,
                        thirdReloadEnd3
                ))
                .setTangentHeadingInterpolation()
                .build();

        driveReload3ToShotPos = follower.pathBuilder()
                .addPath(new BezierLine(
                        thirdReloadEnd3,
                        finalShotPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();
    }

    public void statePathUpdate() {
        switch(pathState) {

            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                resetReadyLatch();
                setVolleyAssist(false);
                setPathState(PathState.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD: {
                shooterSys.update();
                startTurretSettle(getAutoAimTurretCmd());

                if (follower.isBusy()) {
                    if (!volleyAssistActive) intake.setEnabled(false);
                    loader.hold();
                    updateReadyLatchNear(shootPose);
                    break;
                }

                if (!isTurretSettled()) {
                    loader.hold();
                    if (!volleyAssistActive) intake.setEnabled(false);
                    break;
                }

                boolean tagSeen = shooterSys.isTagSeen();
                boolean readyStable = shooterSys.isShooterReadyStable();
                boolean okToStart = (tagSeen && readyStable) || isReadyLatched();

                if (!startedFourShot && shooterSys.isEnabled() && okToStart) {
                    startedFourShot = true;
                    finishedFourShot = false;
                    shooterSys.resetReadyStable();
                    loader.startFourShot();
                }

                if (startedFourShot && !finishedFourShot) {
                    loader.updateFourShot(shooterSys.isShooterReadyStable());

                    if (!volleyAssistActive && loader.getState() == LoaderSubsystem.SeqState.SHOT1_RECOVER) {
                        setVolleyAssist(true);
                    }

                    if (loader.getState() == LoaderSubsystem.SeqState.DONE) finishedFourShot = true;
                }

                if (finishedFourShot) {
                    setVolleyAssist(false);
                    follower.followPath(driveShootPosReloadPos, true);
                    intake.setEnabled(true);
                    setPathState(PathState.DRIVE_SHOOTPOS_RELOAD3POS);
                }
                break;
            }

            case DRIVE_SHOOTPOS_RELOAD3POS:
                if (follower.isBusy()) intake.setEnabled(true);
                else {
                    follower.followPath(driveReloadThree, true);
                    intake.setEnabled(true);
                    setPathState(PathState.DRIVE_THIRD_RELOAD_SEQUENCE);
                }
                break;

            case DRIVE_THIRD_RELOAD_SEQUENCE:
                if (follower.isBusy()) intake.setEnabled(true);
                else {
                    intake.setEnabled(false);
                    reloadDecompressStarted = false;
                    setPathState(PathState.RELOAD_DECOMPRESS);
                }
                break;

            case RELOAD_DECOMPRESS:
                if (!reloadDecompressStarted) {
                    reloadDecompressStarted = true;
                    pathTimer.resetTimer();
                    loader.decompress();
                }
                if (pathTimer.getElapsedTimeSeconds() >= RELOAD_DECOMPRESS_SEC) {
                    loader.hold();
                    follower.followPath(driveReloadPosToShootPos, true);
                    resetReadyLatch();
                    setPathState(PathState.DRIVE_RELOAD_TO_SHOT);
                }
                break;

            case DRIVE_RELOAD_TO_SHOT:
                shooterSys.update();
                startTurretSettle(getAutoAimTurretCmd());

                if (follower.isBusy()) {
                    updateReadyLatchNear(shootPose);
                } else {
                    shooterSys.update();
                    endLoadStarted = false;
                    startedFourShot2 = false;
                    finishedFourShot2 = false;
                    loader.stopAll();
                    setVolleyAssist(false);
                    setPathState(PathState.END_LOAD_AT_SHOOT_POS);

                    startTurretSettle(getAutoAimTurretCmd());
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

                // ===== CHANGED: relaxed gate — no “tag must be seen right now” =====
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
                    startedFourShot2 = false;
                    finishedFourShot2 = false;
                    setPathState(PathState.SHOOT_POST_RELOAD);
                }
                break;
            }

            case SHOOT_POST_RELOAD: {
                shooterSys.update();
                startTurretSettle(getAutoAimTurretCmd());

                if (!isTurretSettled()) {
                    loader.hold();
                    break;
                }

                boolean tagSeen2 = shooterSys.isTagSeen();
                boolean readyStable2 = shooterSys.isShooterReadyStable();
                boolean okToStart2 = (tagSeen2 && readyStable2) || isReadyLatched();

                if (!startedFourShot2) loader.hold();

                if (!startedFourShot2 && shooterSys.isEnabled() && okToStart2) {
                    startedFourShot2 = true;
                    finishedFourShot2 = false;
                    shooterSys.resetReadyStable();
                    loader.startFourShot();
                }

                if (startedFourShot2 && !finishedFourShot2) {
                    loader.updateFourShot(shooterSys.isShooterReadyStable());

                    if (!volleyAssistActive && loader.getState() == LoaderSubsystem.SeqState.SHOT1_RECOVER) {
                        setVolleyAssist(true);
                    }

                    if (loader.getState() == LoaderSubsystem.SeqState.DONE) finishedFourShot2 = true;
                }

                if (finishedFourShot2) {
                    setVolleyAssist(false);
                    loader.stopAll();
                    follower.followPath(driveShotToReload2Start, true);
                    intake.setEnabled(true);
                    setPathState(PathState.DRIVE_SHOT_TO_RELOAD2_START);
                }
                break;
            }

            case DRIVE_SHOT_TO_RELOAD2_START:
                if (follower.isBusy()) intake.setEnabled(true);
                else {
                    follower.followPath(driveReload2, true);
                    intake.setEnabled(true);
                    setPathState(PathState.DRIVE_RELOAD2_SEQUENCE);
                }
                break;

            case DRIVE_RELOAD2_SEQUENCE:
                if (follower.isBusy()) intake.setEnabled(true);
                else {
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
                    follower.followPath(driveReload2ToOpenGate, true);
                    setPathState(PathState.DRIVE_RELOAD2_TO_OPEN_GATE);
                }
                break;

            case DRIVE_RELOAD2_TO_OPEN_GATE:
                if (follower.isBusy()) {
                    intake.setEnabled(false);
                    loader.hold();
                } else {
                    follower.followPath(driveGatePath12, true);
                    setPathState(PathState.DRIVE_GATE_PATH12);
                }
                break;

            case DRIVE_GATE_PATH12:
                if (follower.isBusy()) {
                    intake.setEnabled(false);
                    loader.hold();
                } else {
                    follower.followPath(driveOpenGateToShotPos, true);
                    resetReadyLatch();
                    setPathState(PathState.DRIVE_OPEN_GATE_TO_SHOTPOS);
                }
                break;

            case DRIVE_OPEN_GATE_TO_SHOTPOS:
                shooterSys.update();
                startTurretSettle(getAutoAimTurretCmd());

                if (follower.isBusy()) {
                    intake.setEnabled(false);
                    loader.hold();
                    updateReadyLatchNear(shootPose);
                } else {
                    endLoadStarted2 = false;
                    startedFourShot3 = false;
                    finishedFourShot3 = false;
                    loader.stopAll();
                    setVolleyAssist(false);
                    setPathState(PathState.END_LOAD_AT_SHOOT_POS_2);

                    startTurretSettle(getAutoAimTurretCmd());
                }
                break;

            case DRIVE_RELOAD2_TO_SHOT:
                setPathState(PathState.END_LOAD_AT_SHOOT_POS_2);
                break;

            case END_LOAD_AT_SHOOT_POS_2: {
                shooterSys.update();
                startTurretSettle(getAutoAimTurretCmd());

                if (!isTurretSettled()) {
                    if (!volleyAssistActive) intake.setEnabled(false);
                    loader.hold();
                    break;
                }

                boolean readyEL = shooterSys.isShooterReadyStable();

                if (!endLoadStarted2) {
                    endLoadStarted2 = true;
                    pathTimer.resetTimer();
                }

                // ===== CHANGED: relaxed gate — no “tag must be seen right now” =====
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
                    startedFourShot3 = false;
                    finishedFourShot3 = false;
                    setPathState(PathState.SHOOT_POST_RELOAD_2);
                }
                break;
            }

            case SHOOT_POST_RELOAD_2: {
                shooterSys.update();
                startTurretSettle(getAutoAimTurretCmd());

                if (!isTurretSettled()) {
                    loader.hold();
                    break;
                }

                boolean tagSeen3 = shooterSys.isTagSeen();
                boolean readyStable3 = shooterSys.isShooterReadyStable();
                boolean okToStart3 = (tagSeen3 && readyStable3) || isReadyLatched();

                if (!startedFourShot3) loader.hold();

                if (!startedFourShot3 && shooterSys.isEnabled() && okToStart3) {
                    startedFourShot3 = true;
                    finishedFourShot3 = false;
                    shooterSys.resetReadyStable();
                    loader.startFourShot();
                }

                if (startedFourShot3 && !finishedFourShot3) {
                    loader.updateFourShot(shooterSys.isShooterReadyStable());

                    if (!volleyAssistActive && loader.getState() == LoaderSubsystem.SeqState.SHOT1_RECOVER) {
                        setVolleyAssist(true);
                    }

                    if (loader.getState() == LoaderSubsystem.SeqState.DONE) finishedFourShot3 = true;
                }

                if (finishedFourShot3) {
                    setVolleyAssist(false);
                    loader.stopAll();
                    follower.followPath(driveShotToReload3Start, true);
                    intake.setEnabled(true);
                    setPathState(PathState.DRIVE_SHOT_TO_RELOAD3_START);
                }
                break;
            }

            case DRIVE_SHOT_TO_RELOAD3_START:
                if (follower.isBusy()) intake.setEnabled(true);
                else {
                    follower.followPath(driveReload3, true);
                    intake.setEnabled(true);
                    setPathState(PathState.DRIVE_RELOAD3_SEQUENCE);
                }
                break;

            case DRIVE_RELOAD3_SEQUENCE:
                if (follower.isBusy()) intake.setEnabled(true);
                else {
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
                    follower.followPath(driveReload3ToShotPos, true);
                    resetReadyLatch();
                    setPathState(PathState.DRIVE_RELOAD3_TO_SHOTPOS);
                }
                break;

            case DRIVE_RELOAD3_TO_SHOTPOS:
                shooterSys.update();
                startTurretSettle(getAutoAimTurretCmd());

                if (follower.isBusy()) {
                    if (!volleyAssistActive) intake.setEnabled(false);
                    loader.hold();
                    updateReadyLatchNear(finalShotPose);
                } else {
                    endLoadFinalStarted = false;
                    startedFinal = false;
                    finishedFinal = false;
                    loader.stopAll();
                    setVolleyAssist(false);

                    setPathState(PathState.END_LOAD_AT_FINAL_SHOT);
                }
                break;

            case END_LOAD_AT_FINAL_SHOT: {
                shooterSys.update();
                startTurretSettle(getAutoAimTurretCmd());

                if (!isTurretSettled()) {
                    if (!volleyAssistActive) intake.setEnabled(false);
                    loader.hold();
                    break;
                }

                boolean readyF = shooterSys.isShooterReadyStable();

                if (!endLoadFinalStarted) {
                    endLoadFinalStarted = true;
                    pathTimer.resetTimer();
                }

                if (!readyF) {
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
                    startedFinal = false;
                    finishedFinal = false;

                    setPathState(PathState.SHOOT_FINAL);
                }
                break;
            }

            case SHOOT_FINAL: {
                shooterSys.update();

                if (finishedFinal) {
                    startTurretSettle(0.5);
                    setVolleyAssist(false);
                    loader.stopAll();
                    break;
                }

                startTurretSettle(getAutoAimTurretCmd());

                if (!isTurretSettled()) {
                    loader.hold();
                    break;
                }

                boolean tagSeenF2 = shooterSys.isTagSeen();
                boolean readyF2 = shooterSys.isShooterReadyStable();
                boolean okToStartF = readyF2 || ((tagSeenF2 && readyF2) || isReadyLatched());

                if (!startedFinal) loader.hold();

                if (!startedFinal && shooterSys.isEnabled() && okToStartF) {
                    startedFinal = true;
                    finishedFinal = false;

                    shooterSys.resetReadyStable();
                    loader.startFourShot();
                }

                if (startedFinal && !finishedFinal) {
                    loader.updateFourShot(shooterSys.isShooterReadyStable());

                    if (!volleyAssistActive && loader.getState() == LoaderSubsystem.SeqState.SHOT1_RECOVER) {
                        setVolleyAssist(true);
                    }

                    if (loader.getState() == LoaderSubsystem.SeqState.DONE) finishedFinal = true;
                }

                break;
            }

            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

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

        // ===== CHANGED: NO SERVO MOVEMENT IN INIT =====
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
        intake.setEnabled(false);

        loader = new LoaderSubsystem(
                intake2, flicker, flipper, intake,
                0.662, 0.39,
                0.12, 0.05,
                1.0, 1.0, 150,
                -0.15, 40,
                120
        );

        shooterSys = new ShooterSubsystem(
                limelight, shooter, hood,
                0,
                20, -1,
                8.0,
                265, 16.53,
                new double[]{24,48,80,120},
                new double[]{0.350,0.525,0.790,0.790},
                new double[]{950,1050,1130,1380},
                new double[]{960,1060,1140,1400},
                new double[]{990,1090,1170,1430},
                110
        );

        shooterSys.startVision();

        // ===== CHANGED: DO NOT ENABLE SHOOTER IN INIT (prevents hood movement) =====
        shooterSys.setEnabled(false);

        startedFourShot = false;
        finishedFourShot = false;

        endLoadStarted = false;
        startedFourShot2 = false;
        finishedFourShot2 = false;

        endLoadStarted2 = false;
        startedFourShot3 = false;
        finishedFourShot3 = false;

        endLoadFinalStarted = false;
        startedFinal = false;
        finishedFinal = false;

        reloadDecompressStarted = false;
        reload2DecompressStarted = false;
        reload3DecompressStarted = false;

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

        // ===== CHANGED: first movement commands happen ONLY after start =====
        if (!didStartCommands) {
            applyTurret(TURRET_START_POS);
            turretTargetPos = TURRET_START_POS;
            turretSettleStarted = false;

            flipper.setPosition(0.662);

            // ===== CHANGED: shooter runs from start (and stays running through drive legs) =====
            shooterSys.setEnabled(true);

            didStartCommands = true;
        }
    }

    @Override
    public void loop() {
        follower.update();
        shooterSys.update();

        updateVisionTrim();

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
        telemetry.addData("Shooter TagSeen", shooterSys.isTagSeen());
        telemetry.addData("Shooter Dist(in)", "%.2f", shooterSys.getShooterDistanceIn());
        telemetry.addData("Shooter RPM Tgt", "%.0f", shooterSys.getRpmTargetCmd());
        telemetry.addData("Shooter HoodCmd", "%.3f", shooterSys.getHoodCmd());

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
    // Ready latch helpers (local only)
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
    // Small math/servo helpers (local only)
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
