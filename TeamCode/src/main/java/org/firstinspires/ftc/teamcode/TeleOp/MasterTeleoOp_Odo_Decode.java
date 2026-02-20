package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.FieldTransform;

import java.util.List;

/**
 * ONE-GAMEPAD MASTER TELEOP
 *
 * Alliance Select (INIT Only):
 *  dpad_left  = RED
 *  dpad_right = BLUE
 *
 * Drive:
 *  left stick    = translate
 *  right stick x = rotate
 *
 * CONTROL CHANGES (per latest decision):
 *  dpad_left  = reset IMU yaw
 *  dpad_down  = toggle reverse eject (intake1+intake2 reverse so balls exit intake)
 *
 * Intake/Shooter:
 *  A = IntakeOneMotor toggle
 *  Y = Shooter toggle
 *  X = 4-shot sequence (requires shooter READY stable)
 *  B = STOP: intake1+intake2+flicker OFF, flipper DOWN (0.662). Shooter stays per Y toggle.
 *
 * Turret Aim:
 *  RB = Enable/Disable Auto Aim (toggle)
 *  LB = Turret Home (hold)
 *
 * Key behavior changes:
 *  - READY feedback via RGB light (no rumble)
 *  - If tag becomes unseen: HOLD last known rpm/hood (do not zero them)
 *
 * NEW:
 *  - Two independent RGB lights:
 *      shooterlight (servo port 3): orange/blue/off for shooter ready + volley state
 *      IRlight      (servo port 4): green/off based on IR beam break sensor
 *  - IR beam break sensor:
 *      beamIntake (DigitalChannel)
 *      SOLID GREEN while broken, FLASH GREEN for 1.5s when broken->clear, OFF otherwise
 *
 * CRITICAL ODO RULE:
 *  - Seed Pinpoint at START from PoseStorage if valid:
 *      pinpoint.setPosition(FieldTransform.poseStorageToPinpointPose2D());
 *  - In loop: pinpoint.update() exactly once
 *  - Convert Pinpoint Pose2D -> FTC inches/deg ONLY via FieldTransform:
 *      xIn = FieldTransform.pinpointPoseToFtcXIn(pp)
 *      yIn = FieldTransform.pinpointPoseToFtcYIn(pp)
 *      headingDeg = FieldTransform.pinpointPoseToFtcHeadingDeg(pp)
 *  - All distance-to-goal math uses FTC inches.
 */
@TeleOp(name="MasterTeleOp_Odo_Decode", group="Tele-Master")
public class MasterTeleoOp_Odo_Decode extends OpMode {

    // ======= DRIVE CONFIG NAMES =======
    private static final String DRIVE_FR_NAME = "FR";
    private static final String DRIVE_FL_NAME = "FL";
    private static final String DRIVE_BR_NAME = "BR";
    private static final String DRIVE_BL_NAME = "BL";
    private static final String IMU_NAME      = "imu";

    // ======= SHOTASSIST CONFIG NAMES =======
    private static final String LIMELIGHT_NAME = "limelight";
    private static final String SHOOTER_NAME   = "ShooterMotor";
    private static final String FLICKER_NAME   = "Outertake";
    private static final String INTAKE1_NAME   = "intakeOneMotor";
    private static final String INTAKE2_NAME   = "intakeTwoMotor";
    private static final String HOOD_NAME      = "Shooter hood";
    private static final String FLIPPER_NAME   = "fingler";

    // ======= RGB LIGHT NAMES (DECOUPLED) =======
    private static final String SHOOTER_LIGHT_NAME = "shooterlight"; // Servo port 3
    private static final String IR_LIGHT_NAME      = "IRlight";      // Servo port 4

    // ======= IR BEAM SENSOR NAME =======
    private static final String BEAM_INTAKE_NAME = "beamIntake";

    // ======= RGB LIGHT POSITIONS (goBILDA chart FTC positions) =======
    private static final double LIGHT_OFF    = 0.000; // Off (500us)
    private static final double LIGHT_ORANGE = 0.333; // Orange (1200us)
    private static final double LIGHT_BLUE   = 0.611; // Blue (1700us)
    private static final double LIGHT_GREEN  = 0.500; // Green (1500us) for IR light

    // IR flash timing (match test)
    private static final double IR_FLASH_DURATION_SEC = 1.5;
    private static final double IR_FLASH_PERIOD_SEC   = 0.15;

    // ======= TURRET / PINPOINT NAMES =======
    private static final String PINPOINT_NAME = "pinpoint";
    private static final String SERVO_L_NAME  = "LRotation";
    private static final String SERVO_R_NAME  = "RRotation";

    // ======= LIMELIGHT / PIPELINE =======
    private static final int APRILTAG_PIPELINE_INDEX = 0;
    private static final int BLUE_GOAL_ID = 20;
    private static final int RED_GOAL_ID  = 24;

    // ======= DISTANCE OFFSETS =======
    private static final double CAMERA_TO_FLYWHEEL_IN = 8.0;

    // ======= SHOT MAP TABLE (YOUR VALUES) =======
    private static final double[] DIST_IN  = { 16, 32, 48, 64, 80, 120 };
    private static final double[] HOOD_POS = { 0.310, 0.530, 0.730, 0.750, 0.780, 0.790 };
    private static final double[] RPM_MIN  = { 1020, 1045, 1140, 1210, 1260, 1440 };
    private static final double[] RPM_TGT  = { 1040, 1060, 1150, 1230, 1280, 1470 };
    private static final double[] RPM_MAX  = { 1050, 1070, 1160, 1250, 1300, 1500 };//previously 1030,1130,1210,1470

    // ======= HARDWARE =======
    private DcMotor driveFR, driveFL, driveBR, driveBL;
    private IMU imu;

    private Limelight3A limelight;

    private DcMotorEx shooter, flicker, intake1, intake2;
    private Servo hood, flipper;

    private GoBildaPinpointDriver odo;
    private Servo turretL, turretR;

    // Two RGB lights
    private Servo shooterlight;
    private Servo IRlight;

    // IR beam break sensor
    private DigitalChannel beamIntake;

    // ======= FLIPPER POSITIONS =======
    private static final double FLIP_DOWN = 0.662;
    private static final double FLIP_UP   = 0.41;

    // ======= INTAKES =======
    private static final double INTAKE1_PWR = 1.0;
    private static final double INTAKE1_BURST_PWR = 1.0;

    // Reverse eject mode power (full reverse)
    private static final double EJECT_PWR = -1.0;

    // ======= FEED / HOLD / DECOMPRESS =======
    private static final double HOLD_PWR_INTAKE2 = 0.12;
    private static final double HOLD_PWR_FLICKER = 0.05;

    private static final double FEED_PWR_INTAKE2 = 1.0;
    private static final double FEED_PWR_FLICKER = 1.0;

    // ======= THROUGHPUT TUNING (UPDATED) =======
    private static final int FEED_MS = 150;            // was 150
    private static final int SHOT12_SPACING_MS = 130;  // was 220
    private static final int MIN_RECOVER_MS  = 70;     // was 120

    // FIX #2: reduce recovery delay only for Ball 3 (SHOT2_HOLD -> SHOT3_FEED)
    private static final int BALL3_RECOVER_MS = 0;

    // Combine DECOMPRESS + RECOVER into one short HOLD phase (no reverse decompress)
    private static final int HOLD_MS_AFTER_FEED = 55;  // new short hold window

    // Turn off extra hold drag during volley
    private static final double VOLLEY_HOLD_PWR_INTAKE2 = 0.4; // new (was 0.12 during recover)
    private static final double VOLLEY_HOLD_PWR_FLICKER = 0.1; // new (was 0.05 during recover)

    // ======= READY STABLE =======
    private static final int READY_STABLE_MS = 90;

    // ======= LATCHES =======
    private boolean intake1Enabled = false;
    private boolean shooterEnabled = false;
    private boolean intake1BurstOverride = false;

    // Reverse eject toggle (NOW dpad_down)
    private boolean reverseEjectEnabled = false;

    // ======= STATE MACHINE =======
    private enum SeqState {
        IDLE,
        SHOT1_FEED, SHOT1_HOLD,
        SHOT2_FEED, SHOT2_HOLD,
        SHOT3_FEED, SHOT3_HOLD,
        SHOT4_FEED, SHOT4_HOLD,
        DONE,
        ABORTED
    }
    private SeqState state = SeqState.IDLE;

    private final ElapsedTime seqTimer = new ElapsedTime();
    private final ElapsedTime readyStableTimer = new ElapsedTime();

    // Edge detection
    private boolean prevA=false, prevB=false, prevX=false, prevY=false;
    private boolean prevDpadDown=false;  // reverse eject toggle
    private boolean prevRB=false;

    // ======= IR LIGHT FLASH STATE =======
    private boolean prevIntakeBroken = false;
    private boolean irFlashing = false;
    private final ElapsedTime irFlashTimer = new ElapsedTime();
    private final ElapsedTime irFlashToggleTimer = new ElapsedTime();
    private boolean irFlashOn = false;

    // ======= “LIVE” SHOT ASSIST OUTPUTS =======
    private double shooterDistIn = Double.NaN;

    // P and F values
    double F = 15.5022;//prev 16.53
    double P = 180;//prev 205.5

    // ======= Commands / last-known holding =======
    private double hoodCmd = HOOD_POS[0];
    private double rpmMinCmd = 0;
    private double rpmTgtCmd = 0;
    private double rpmMaxCmd = 0;

    private boolean haveLastShotSolution = false;
    private double lastHoodCmd = HOOD_POS[0];
    private double lastRpmMinCmd = 0;
    private double lastRpmTgtCmd = 0;
    private double lastRpmMaxCmd = 0;

    // FIX: HOLD-LAST should also hold the distance used/displayed
    private double lastShooterDistIn = Double.NaN;

    // Track whether we saw a tag this frame
    private boolean sawTagThisFrame = false;

    // Prevent hood motion before START
    private boolean teleopStarted = false;

    // =========================
    // #4 NEW: Stabilize distance before mapping (filter + rpm slew + hood deadband/slew)
    // =========================
    private final Median3Filter shotDistMedian = new Median3Filter();

    // ===== NEW: separate LPFs for RPM (smoother) vs Hood (quicker) =====
    private final LowPassFilter shotDistLpfRpm  = new LowPassFilter(0.30); // keep smoother for RPM
    private final LowPassFilter shotDistLpfHood = new LowPassFilter(0.80); // prev 0.65 quicker for hood response

    private boolean haveLastFilteredShotDist = false;
    private double lastFilteredShotDistIn = Double.NaN;

    private final SlewLimiter rpmSlew = new SlewLimiter();
    private final HoodController hoodStabilizer = new HoodController();

    private static final double RPM_SLEW_PER_LOOP = 70.0;
    private static final double HOOD_DEADBAND = 0.0015;//prev 0.003
    private static final double HOOD_MAX_STEP = 0.015;//prev 0.006

    // =========================
    // TURRET AIM (from your turret teleop)
    // =========================

    // (FTC field inches)
    private static final double RED_GOAL_X  = -58.3727;//previously -58.3727
    private static final double RED_GOAL_Y  = 63.0000;//previously 70.00 and 64.6425

    private static final double BLUE_GOAL_X = -58.3727;//previously -58.3727
    private static final double BLUE_GOAL_Y = -55.625;//previously -70.00

    private static final double TURRET_HOME    = 0.50;
    private static final double POS_PER_DEG_CW = 0.005368;//was 0.007643
    private static final double SERVO_MIN_SAFE = 0.10;
    private static final double SERVO_MAX_SAFE = 0.90;

    private static final double POD_X_OFFSET_MM = 82.55;
    private static final double POD_Y_OFFSET_MM = -95.25;

    private static final double TURRET_FWD_OFFSET_IN  = -4.0;
    private static final double TURRET_LEFT_OFFSET_IN =  0.0;

    // ======= TURRET STABILITY TUNING (ANTI-HUNT) =======
    private static final double BEARING_DEADBAND_DEG = 0.90;   // was 0.75 (reduces long-range chatter)
    private static final int    TURRET_SETTLE_LOOPS  = 3;      // latch when inside deadband for N loops
    private static final double TURRET_UNLATCH_MULT  = 1.4;    // must exceed deadband*mult to "wake up"

    private static final double SERVO_SLEW_PER_SEC   = 2.8;//was 1.2
    private static final double MAX_STEP_PER_LOOP    = 0.045;
    private static final double KP_DEG_TO_SERVO      = 0.0046; // base gain (distance-scaled at runtime)

    private static final double VISION_X_SIGN = -1.0;
    private static final double VISION_MIN_Z_IN = 12.0;
    private static final double VISION_MAX_Z_IN = 160.0;
    private static final double VISION_MAX_TRIM_DEG = 10.0;
    private static final double VISION_LPF_ALPHA = 0.35;
    private static final int VISION_HOLD_MS = 250;
    private static final int VISION_DECAY_MS = 900;
    private static final double VISION_MAX_ABS_TX_DEG = 8.0;

    private boolean allianceIsRed = true;
    private boolean autoAimEnabled = true;

    // ======= NEW: Alliance turret offset (apply only after START) =======
    private static final double TURRET_ALLIANCE_OFFSET_DEG = 0.00;
    private double turretAllianceOffsetDegCW = 0.0;

    private double lastTurretCmd = TURRET_HOME;
    private final ElapsedTime turretLoopTimer = new ElapsedTime();

    private boolean visionTrimEnabled = true;
    private boolean visionHasGood = false;
    private double visionTrimDegCW = 0.0;
    private double visionTrimDegCW_raw = 0.0;
    private final ElapsedTime visionLastGoodTimer = new ElapsedTime();
    private final ElapsedTime visionDecayTimer = new ElapsedTime();

    private boolean turretTagSeen = false;
    private int turretTagId = -1;
    private double txDeg = Double.NaN;

    // =========================================================
    // STEP 3: Heading-rate feedforward (locks while rotating / translating)
    // =========================================================
    private double prevHeadingDegFTC = 0.0;
    private boolean havePrevHeading = false;

    private static final double TURRET_FF_GAIN = 0.035;   // deg turret CW per (deg robot/sec)
    private static final double TURRET_FF_MAX_DEG = 2.50; // clamp for safety

    // Telemetry taps for Step 3
    private double lastHeadingRateDegPerSec = 0.0;
    private double lastTurretFFDegCW = 0.0;

    // =========================================================
    // STEP 4: Translation-based LOS feedforward (dummy steps -> implemented)
    // =========================================================
    private static final double TRANS_LOOKAHEAD_SEC = 0.10;    // start 0.08-0.12
    private static final double TRANS_MIN_SPEED_IN_S = 4.0;    // start 3-6
    private static final double TRANS_MAX_FF_DEG = 6.0;        // clamp
    private static final double TRANS_FF_STRENGTH = 1.0;       // unitless scalar

    private double lastTurretXIn = 0.0;
    private double lastTurretYIn = 0.0;
    private boolean haveLastTurretPos = false;

    // Step 4 telemetry taps
    private double lastTurretVInPerSec = 0.0;
    private double lastLosRateDegPerSec = 0.0;
    private double lastTransFfDegCW = 0.0;
    private boolean lastTransFfActive = false;

    // =========================================================
    // Pose-based shot assist + vision-range override
    // =========================================================
    private static final double POSE_MIN_VALID_IN = 12.0;
    private static final double POSE_MAX_VALID_IN = 160.0;
    private static final double POSE_MAX_STEP_IN_PER_LOOP = 30.0;     // prev 12 freeze if jump exceeds this
    private static final double POSE_TO_FLYWHEEL_OFFSET_IN = 0.0;      // tune if needed (start 0)

    // Vision range gating
    private static final double VISION_DIST_MAX_STEP_IN_PER_LOOP = 30.0; // prev 12 freeze if tag distance jumps too much

    private double lastPoseDistIn = Double.NaN;
    private double lastVisionDistIn = Double.NaN;

    // =========================================================
    // NEW: ODO CACHE (odo.update() ONCE PER LOOP)
    // =========================================================
    private double cachedRobotX = 0.0;      // FTC inches
    private double cachedRobotY = 0.0;      // FTC inches
    private double cachedRobotXPin = 0.0;   // Pinpoint inches (for telemetry only)
    private double cachedRobotYPin = 0.0;   // Pinpoint inches (for telemetry only)

    private double cachedHeadingDegFTC = 0.0;  // FTC deg, wrapped [-180,180]

    private double cachedGoalX = 0.0;       // FTC inches
    private double cachedGoalY = 0.0;       // FTC inches

    // Shooter distance sources (for telemetry)
    private boolean usingVisionRange = false;
    private double visionZInAbs = Double.NaN;
    private double distPoseIn = Double.NaN;
    private double distVisionIn = Double.NaN;

    // Turret anti-hunt state (latch)
    private int turretSettleCount = 0;
    private boolean turretLatched = false;
    private double lastTurretErrDegCW = 0.0;
    private double lastTurretKpUsed = KP_DEG_TO_SERVO;

    // ======= Track whether PoseStorage was valid during INIT (for telemetry) =======
    private boolean poseStorageWasValidInInit = false;

    @Override
    public void init() {
        // ===== DRIVE =====
        driveFR = hardwareMap.get(DcMotor.class, DRIVE_FR_NAME);
        driveFL = hardwareMap.get(DcMotor.class, DRIVE_FL_NAME);
        driveBR = hardwareMap.get(DcMotor.class, DRIVE_BR_NAME);
        driveBL = hardwareMap.get(DcMotor.class, DRIVE_BL_NAME);

        driveFL.setDirection(DcMotorSimple.Direction.REVERSE);
        driveBL.setDirection(DcMotorSimple.Direction.REVERSE);
        driveFR.setDirection(DcMotorSimple.Direction.FORWARD);
        driveBR.setDirection(DcMotorSimple.Direction.FORWARD);

        driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU init
        imu = hardwareMap.get(IMU.class, IMU_NAME);
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(parameters);

        // ===== LIMELIGHT =====
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
        limelight.start();

        // ===== SHOTASSIST =====
        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);
        flicker = hardwareMap.get(DcMotorEx.class, FLICKER_NAME);
        intake1 = hardwareMap.get(DcMotorEx.class, INTAKE1_NAME);
        intake2 = hardwareMap.get(DcMotorEx.class, INTAKE2_NAME);
        hood    = hardwareMap.get(Servo.class, HOOD_NAME);
        flipper = hardwareMap.get(Servo.class, FLIPPER_NAME);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        // PIDF set ONCE (do not re-set every loop)
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, 0, 0, F));

        flicker.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake1.setDirection(DcMotor.Direction.REVERSE);

        flicker.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // NOTE: per request, NO robot movements in INIT (no hood/flipper/turret positioning, no stopping motors)

        // ===== PINPOINT + TURRET =====
        odo = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);
        turretL = hardwareMap.get(Servo.class, SERVO_L_NAME);
        turretR = hardwareMap.get(Servo.class, SERVO_R_NAME);

        odo.setOffsets(POD_X_OFFSET_MM, POD_Y_OFFSET_MM, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        // We DO NOT seed from PoseStorage in INIT.
        // The single most important operational step is seeding at START.
        poseStorageWasValidInInit = PoseStorage.valid;

        // Safe baseline reset (will be overridden at START if PoseStorage.valid)
        odo.resetPosAndIMU();
        odo.update();

        // Vision trim state init (no movement)
        visionTrimDegCW = 0.0;
        visionTrimDegCW_raw = 0.0;
        visionHasGood = false;
        visionLastGoodTimer.reset();
        visionDecayTimer.reset();

        haveLastShotSolution = false;
        lastHoodCmd = hoodCmd;
        lastRpmMinCmd = rpmMinCmd;
        lastRpmTgtCmd = rpmTgtCmd;
        lastRpmMaxCmd = rpmMaxCmd;
        lastShooterDistIn = shooterDistIn;

        // #4: reset filters/slew so INIT-loop history doesn't bias START
        shotDistMedian.reset();
        shotDistLpfRpm.reset();
        shotDistLpfHood.reset();
        rpmSlew.reset();
        hoodStabilizer.reset();
        haveLastFilteredShotDist = false;
        lastFilteredShotDistIn = Double.NaN;

        turretLoopTimer.reset();

        // ===== STEP 3 init =====
        havePrevHeading = false;
        prevHeadingDegFTC = 0.0;
        lastHeadingRateDegPerSec = 0.0;
        lastTurretFFDegCW = 0.0;

        // ===== STEP 4 init =====
        haveLastTurretPos = false;
        lastTurretXIn = 0.0;
        lastTurretYIn = 0.0;
        lastTurretVInPerSec = 0.0;
        lastLosRateDegPerSec = 0.0;
        lastTransFfDegCW = 0.0;
        lastTransFfActive = false;

        // ===== RGB LIGHTS (DECOUPLED) =====
        shooterlight = hardwareMap.get(Servo.class, SHOOTER_LIGHT_NAME);
        IRlight      = hardwareMap.get(Servo.class, IR_LIGHT_NAME);

        shooterlight.setPosition(LIGHT_OFF);
        IRlight.setPosition(LIGHT_OFF);

        // ===== IR BEAM SENSOR =====
        beamIntake = hardwareMap.get(DigitalChannel.class, BEAM_INTAKE_NAME);
        beamIntake.setMode(DigitalChannel.Mode.INPUT);

        prevIntakeBroken = false;
        irFlashing = false;
        irFlashTimer.reset();
        irFlashToggleTimer.reset();
        irFlashOn = false;

        teleopStarted = false;

        // ===== NEW: ensure no alliance offset until START =====
        turretAllianceOffsetDegCW = 0.0;

        lastPoseDistIn = Double.NaN;
        lastVisionDistIn = Double.NaN;

        turretSettleCount = 0;
        turretLatched = false;

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("MasterTeleOp One-Gamepad READY");
        telemetry.addLine("INIT: dpad_left=RED, dpad_right=BLUE");
        telemetry.addLine("Drive: LS translate, RSX rotate");
        telemetry.addLine("dpad_left reset yaw | dpad_down toggle reverse eject");
        telemetry.addLine("A intake toggle | Y shooter toggle | X 4-shot | B stop intake+flicker+flipper down");
        telemetry.addLine("RB(toggle) turret autoaim | LB turret home");
        telemetry.addLine("IR: beamIntake -> IRlight GREEN (solid/flash)");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Alliance select in INIT only
        if (gamepad1.dpad_left)  allianceIsRed = true;
        if (gamepad1.dpad_right) allianceIsRed = false;

        // ODO UPDATE ONCE
        updateOdoCacheOnce();

        updateFromLimelightAndComputeShot_HoldLast(limelight.getLatestResult());

        // IR light update also allowed in init_loop (no mechanical movement)
        updateIRLightFromBeam();

        telemetry.addLine("=== INIT ===");
        telemetry.addData("Alliance", allianceIsRed ? "RED" : "BLUE");
        telemetry.addData("HaveLastShotSolution", haveLastShotSolution);
        telemetry.addData("Last RPM tgt", "%.0f", lastRpmTgtCmd);
        telemetry.addData("Last Hood", "%.3f", lastHoodCmd);
        telemetry.addData("beamIntake(raw)", beamIntake.getState());
        telemetry.addData("IntakeBroken", !beamIntake.getState());

        telemetry.addLine("=== PoseStorage (will seed at START) ===");
        telemetry.addData("PoseStorage.valid", PoseStorage.valid);
        telemetry.addData("PoseStorage x,y,h", "%.1f, %.1f, %.1f", PoseStorage.xIn, PoseStorage.yIn, PoseStorage.headingDeg);

        telemetry.addLine("--- Odo Cache ---");
        telemetry.addData("Pinpoint X,Y(in) [tele]", "%.1f, %.1f", cachedRobotXPin, cachedRobotYPin);
        telemetry.addData("FTC X,Y(in)", "%.1f, %.1f", cachedRobotX, cachedRobotY);
        telemetry.addData("FTC Heading(deg)", "%.1f", cachedHeadingDegFTC);

        telemetry.addLine("--- Shot Dist ---");
        telemetry.addData("UsingVisionRange", usingVisionRange);
        telemetry.addData("VisionZAbs(in)", Double.isNaN(visionZInAbs) ? "N/A" : String.format("%.1f", visionZInAbs));
        telemetry.addData("DistVision(in)", Double.isNaN(distVisionIn) ? "N/A" : String.format("%.1f", distVisionIn));
        telemetry.addData("DistPose(in)", Double.isNaN(distPoseIn) ? "N/A" : String.format("%.1f", distPoseIn));

        telemetry.update();
    }

    @Override
    public void start() {
        teleopStarted = true;

        // ===================== CRITICAL: Seed Pinpoint from PoseStorage at START =====================
        if (PoseStorage.valid) {
            odo.setPosition(FieldTransform.poseStorageToPinpointPose2D());
            // Optional immediate read so the first loop telemetry shows the seeded pose
            odo.update();

            // FIX: Clear pose/vision step-gates after seeding so the first TeleOp frame can't freeze on a "jump"
            lastPoseDistIn = Double.NaN;
            lastVisionDistIn = Double.NaN;

            // Clear after seeding so it doesn't get reused accidentally
            PoseStorage.clear();
        }
        // ============================================================================================

        // #4: reset filters/slew at START so any init_loop vision doesn't bias first shots
        shotDistMedian.reset();
        shotDistLpfRpm.reset();
        shotDistLpfHood.reset();
        rpmSlew.reset();
        hoodStabilizer.reset();
        haveLastFilteredShotDist = false;
        lastFilteredShotDistIn = Double.NaN;

        // ===== apply alliance turret offset ONLY after START =====
        turretAllianceOffsetDegCW = allianceIsRed ? +TURRET_ALLIANCE_OFFSET_DEG : -TURRET_ALLIANCE_OFFSET_DEG;

        // Per request: initial mechanical positioning happens only upon START
        hood.setPosition(hoodCmd);
        flipper.setPosition(FLIP_DOWN);

        stopAllIntakeAndFlickerAndFlipperDown();
        stopShooter();

        lastTurretCmd = TURRET_HOME;
        setTurretCmd(lastTurretCmd);

        turretSettleCount = 0;
        turretLatched = false;

        // ===== STEP 3: seed heading history at START (prevents spike) =====
        prevHeadingDegFTC = cachedHeadingDegFTC;
        havePrevHeading = true;
        lastHeadingRateDegPerSec = 0.0;
        lastTurretFFDegCW = 0.0;

        // ===== STEP 4: seed turret position history at START (prevents spike) =====
        // (Uses current cached pose; turretX/Y will be computed in first runTurretAiming loop)
        haveLastTurretPos = false;
        lastTurretVInPerSec = 0.0;
        lastLosRateDegPerSec = 0.0;
        lastTransFfDegCW = 0.0;
        lastTransFfActive = false;

        shooterlight.setPosition(LIGHT_OFF);

        // IR light state reset at start (still driven by beam immediately after)
        IRlight.setPosition(LIGHT_OFF);
        prevIntakeBroken = !beamIntake.getState();
        irFlashing = false;
        irFlashTimer.reset();
        irFlashToggleTimer.reset();
        irFlashOn = false;
    }

    @Override
    public void loop() {
        // ===================== DRIVE =====================
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // UPDATED: Reset yaw is now dpad_left (IMU USED FOR DRIVE ONLY)
        if (gamepad1.dpad_left) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX *= 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double fl = (rotY + rotX + rx) / denominator;
        double bl = (rotY - rotX + rx) / denominator;
        double fr = (rotY - rotX - rx) / denominator;
        double br = (rotY + rotX - rx) / denominator;

        driveFL.setPower(fl);
        driveBL.setPower(bl);
        driveFR.setPower(fr);
        driveBR.setPower(br);

        // Read Limelight once
        LLResult ll = limelight.getLatestResult();

        // ===================== ODO UPDATE ONCE (PINPOINT IS SOURCE OF TRUTH FOR AIM) =====================
        updateOdoCacheOnce();

        // ===================== EDGE DETECT =====================
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean xBtn = gamepad1.x;
        boolean yBtn = gamepad1.y;

        boolean aPressed = a && !prevA;
        boolean bPressed = b && !prevB;
        boolean xPressed = xBtn && !prevX;
        boolean yPressed = yBtn && !prevY;

        // UPDATED: reverse eject toggle is now dpad_down
        boolean dpadDown = gamepad1.dpad_down;
        boolean dpadDownPressed = dpadDown && !prevDpadDown;

        boolean rb = gamepad1.right_bumper;
        boolean rbPressed = rb && !prevRB;

        // ===================== SHOT SOLUTION (VISION RANGE FIRST, ELSE POSE, HOLD LAST) =====================
        updateFromLimelightAndComputeShot_HoldLast(ll);

        // ===================== IR LIGHT (beamIntake -> IRlight) =====================
        updateIRLightFromBeam();

        // ===================== SHOOTERLIGHT (orange/blue/off only) =====================
        // Blue during 4-shot sequence
        // Orange when shooter ready stable
        // Off otherwise (including shooter disabled)
        if (!shooterEnabled) {
            shooterlight.setPosition(LIGHT_OFF);
        } else if (state != SeqState.IDLE && state != SeqState.DONE && state != SeqState.ABORTED) {
            shooterlight.setPosition(LIGHT_BLUE);
        } else {
            boolean readyNow = isShooterReadyStable();
            if (readyNow) shooterlight.setPosition(LIGHT_ORANGE);
            else shooterlight.setPosition(LIGHT_OFF);
        }

        // ===================== Reverse eject toggle (dpad_down) =====================
        if (dpadDownPressed) {
            reverseEjectEnabled = !reverseEjectEnabled;
        }

        // ===================== Intake1 toggle (A) =====================
        if (aPressed) {
            intake1Enabled = !intake1Enabled;
        }

        // ===================== Shooter toggle (Y) =====================
        if (yPressed) {
            shooterEnabled = !shooterEnabled;
            resetReadyStable();
            if (!shooterEnabled) stopShooter();
        }

        // ===================== B behavior (STOP intake/flicker/flipper down) =====================
        if (bPressed) {
            intake1Enabled = false;
            intake1BurstOverride = false;
            reverseEjectEnabled = false;
            stopAllIntakeAndFlickerAndFlipperDown();
        }

        // ===================== Apply intake powers =====================
        if (reverseEjectEnabled) {
            intake1.setPower(EJECT_PWR);
            intake2.setPower(EJECT_PWR);
        } else {
            if (intake1BurstOverride) intake1.setPower(INTAKE1_BURST_PWR);
            else intake1.setPower(intake1Enabled ? INTAKE1_PWR : 0.0);

            if (state == SeqState.IDLE || state == SeqState.ABORTED) {
                flicker.setPower(0);
                intake2.setPower(0);
            }
        }

        // ===================== Shooter command =====================
        if (shooterEnabled && haveLastShotSolution && lastRpmTgtCmd > 0) {
            shooter.setVelocity(lastRpmTgtCmd);
        } else {
            stopShooter();
        }

        // ===================== Start 4-shot sequence (X) =====================
        if (xPressed && state == SeqState.IDLE) {
            if (shooterEnabled && isShooterReadyStable()) {
                startFeed(SeqState.SHOT1_FEED);
            }
        }

        // ===================== Run 4-shot state machine =====================
        switch (state) {
            case IDLE:
                intake1BurstOverride = false;
                flipper.setPosition(FLIP_DOWN);
                if (!reverseEjectEnabled) {
                    flicker.setPower(0);
                    intake2.setPower(0);
                }
                break;

            case SHOT1_FEED:
                if (seqTimer.milliseconds() >= FEED_MS) startHold(SeqState.SHOT1_HOLD);
                break;

            case SHOT1_HOLD:
                if (seqTimer.milliseconds() >= SHOT12_SPACING_MS && isShooterReadySoft()) startFeed(SeqState.SHOT2_FEED);
                break;

            case SHOT2_FEED:
                if (seqTimer.milliseconds() >= FEED_MS) startHold(SeqState.SHOT2_HOLD);
                break;

            case SHOT2_HOLD:
                // FIX #2: shorter recovery only before Ball 3
                if (seqTimer.milliseconds() >= BALL3_RECOVER_MS && isShooterReadySoft()) startFeed(SeqState.SHOT3_FEED);
                break;

            case SHOT3_FEED:
                if (seqTimer.milliseconds() >= FEED_MS) startHold(SeqState.SHOT3_HOLD);
                break;

            case SHOT3_HOLD:
                if (seqTimer.milliseconds() >= MIN_RECOVER_MS && isShooterReadySoft()) startFeed(SeqState.SHOT4_FEED);
                break;

            case SHOT4_FEED:
                if (seqTimer.milliseconds() >= FEED_MS) startHold(SeqState.SHOT4_HOLD);
                break;

            case SHOT4_HOLD:
                if (seqTimer.milliseconds() >= MIN_RECOVER_MS && isShooterReadySoft()) {
                    stopSequenceMotors();
                    intake1BurstOverride = false;
                    flipper.setPosition(FLIP_DOWN);
                    state = SeqState.DONE;
                    seqTimer.reset();
                }
                break;

            case DONE:
                stopSequenceMotors();
                intake1BurstOverride = false;
                flipper.setPosition(FLIP_DOWN);
                state = SeqState.IDLE;
                break;

            case ABORTED:
                flipper.setPosition(FLIP_DOWN);
                break;
        }

        // ===================== TURRET CONTROLS =====================
        if (rbPressed) autoAimEnabled = !autoAimEnabled;

        if (gamepad1.left_bumper) {
            lastTurretCmd = TURRET_HOME;
            setTurretCmd(lastTurretCmd);
            turretSettleCount = 0;
            turretLatched = false;

            // Step 4: reset pos history when you home (prevents a dt spike jump)
            haveLastTurretPos = false;
            lastTransFfActive = false;
        } else {
            runTurretAiming(ll);
        }

        // ===================== TELEMETRY (DIAGNOSTIC) =====================
        double vShooter = shooter.getVelocity();
        boolean readyNow = isShooterReadyStable();

        telemetry.addLine("=== One-Pad Master TeleOp ===");
        telemetry.addData("Alliance (init)", allianceIsRed ? "RED" : "BLUE");

        telemetry.addLine("--- Drive ---");
        telemetry.addData("IMU Yaw(deg) [drive only]", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        telemetry.addLine("--- Odo (Pinpoint) ---");
        telemetry.addData("Pinpoint X,Y(in) [tele]", "%.1f, %.1f", cachedRobotXPin, cachedRobotYPin);
        telemetry.addData("FTC X,Y(in)", "%.1f, %.1f", cachedRobotX, cachedRobotY);
        telemetry.addData("FTC Heading(deg)", "%.1f", cachedHeadingDegFTC);

        telemetry.addLine("--- Shot Assist ---");
        telemetry.addData("SawTagThisFrame", sawTagThisFrame);
        telemetry.addData("UsingVisionRange", usingVisionRange);
        telemetry.addData("VisionZAbs(in)", Double.isNaN(visionZInAbs) ? "N/A" : String.format("%.1f", visionZInAbs));
        telemetry.addData("DistVision(in)", Double.isNaN(distVisionIn) ? "N/A" : String.format("%.1f", distVisionIn));
        telemetry.addData("DistPose(in)", Double.isNaN(distPoseIn) ? "N/A" : String.format("%.1f", distPoseIn));
        telemetry.addData("ShooterDistUsed(in)", Double.isNaN(shooterDistIn) ? "N/A" : String.format("%.1f", shooterDistIn));

        telemetry.addLine("--- Intake/Shooter ---");
        telemetry.addData("Intake1Enabled(A)", intake1Enabled);
        telemetry.addData("ReverseEject(dpad_down)", reverseEjectEnabled);
        telemetry.addData("ShooterEnabled(Y)", shooterEnabled);
        telemetry.addData("HaveLastShotSolution", haveLastShotSolution);
        telemetry.addData("RPM tgt (held)", "%.0f", lastRpmTgtCmd);
        telemetry.addData("RPM window", "%.0f..%.0f", lastRpmMinCmd, lastRpmMaxCmd);
        telemetry.addData("ShooterVel", "%.0f", vShooter);
        telemetry.addData("READY (shooterlight)", readyNow);
        telemetry.addData("4Shot State", state);

        telemetry.addLine("--- IR Beam ---");
        telemetry.addData("beamIntake(raw)", beamIntake.getState());
        telemetry.addData("IntakeBroken", !beamIntake.getState());
        telemetry.addData("IR Flushing", irFlashing);
        telemetry.addData("IR FlashTimer(s)", "%.2f", irFlashTimer.seconds());

        telemetry.addLine("--- Turret ---");
        telemetry.addData("AutoAim (RB)", autoAimEnabled);
        telemetry.addData("TurretCmd", "%.3f", lastTurretCmd);
        telemetry.addData("AllianceOffsetDegCW", "%.2f", turretAllianceOffsetDegCW);
        telemetry.addData("VisionTagSeen", turretTagSeen);
        telemetry.addData("tx(deg)", Double.isNaN(txDeg) ? "N/A" : String.format("%.2f", txDeg));
        telemetry.addData("TurretErrDegCW", "%.2f", lastTurretErrDegCW);
        telemetry.addData("TurretLatched", turretLatched);
        telemetry.addData("KpUsed", "%.5f", lastTurretKpUsed);
        telemetry.addData("WantedGoalTag", allianceIsRed ? RED_GOAL_ID : BLUE_GOAL_ID);
        telemetry.addData("ChosenGoalTag", turretTagId);
        telemetry.addData("TagMatch", (turretTagId == (allianceIsRed ? RED_GOAL_ID : BLUE_GOAL_ID)));

        // STEP 3 telemetry
        telemetry.addData("HeadingRate(deg/s)", "%.1f", lastHeadingRateDegPerSec);
        telemetry.addData("TurretFF(degCW)", "%.2f", lastTurretFFDegCW);

        // STEP 4 telemetry
        telemetry.addData("TurretV(in/s)", "%.1f", lastTurretVInPerSec);
        telemetry.addData("LOSRate(deg/s)", "%.1f", lastLosRateDegPerSec);
        telemetry.addData("TransFF(degCW)", "%.2f", lastTransFfDegCW);
        telemetry.addData("TransFFActive", lastTransFfActive);

        // ======= Fiducial debug (IDs present) =======
        int fidCount = 0;
        String ids = "";
        if (ll != null && ll.isValid() && ll.getFiducialResults() != null) {
            fidCount = ll.getFiducialResults().size();
            StringBuilder sb = new StringBuilder();
            for (LLResultTypes.FiducialResult t : ll.getFiducialResults()) {
                sb.append((int) t.getFiducialId()).append(" ");
            }
            ids = sb.toString();
        }
        telemetry.addData("Fids", "%d [%s]", fidCount, ids);

        telemetry.update();

        // ===================== Save prev =====================
        prevA=a; prevB=b; prevX=xBtn; prevY=yBtn;
        prevDpadDown = dpadDown;
        prevRB = rb;
    }

    // =========================================================
    // ODO CACHE: call odo.update() ONCE, compute FTC pose ONLY via FieldTransform
    // =========================================================
    private void updateOdoCacheOnce() {
        // EXACTLY ONCE per loop
        odo.update();

        Pose2D pp = odo.getPosition();

        // FTC pose MUST come from FieldTransform (single source of truth)
        cachedRobotX = FieldTransform.pinpointPoseToFtcXIn(pp);
        cachedRobotY = FieldTransform.pinpointPoseToFtcYIn(pp);
        cachedHeadingDegFTC = FieldTransform.pinpointPoseToFtcHeadingDeg(pp);

        // Pinpoint pose (telemetry only): convert mm -> inches (NO axis/sign transforms here)
        cachedRobotXPin = FieldTransform.mmToIn(pp.getX(DistanceUnit.MM));
        cachedRobotYPin = FieldTransform.mmToIn(pp.getY(DistanceUnit.MM));

        cachedGoalX = allianceIsRed ? RED_GOAL_X : BLUE_GOAL_X;
        cachedGoalY = allianceIsRed ? RED_GOAL_Y : BLUE_GOAL_Y;
    }

    // =========================================================
    // IR Light behavior (beamIntake -> IRlight)
    // =========================================================
    private void updateIRLightFromBeam() {
        // With pull-up wiring: getState() == true means CLEAR (not broken)
        // Beam broken pulls line LOW => getState() == false
        boolean intakeBroken = !beamIntake.getState();

        // Edge detect: broken -> clear (ball passed the beam)
        boolean intakeClearedEvent = (prevIntakeBroken && !intakeBroken);
        if (intakeClearedEvent) startIRFlash();

        // Priority: SOLID GREEN while broken
        if (intakeBroken) {
            irFlashing = false; // cancel flash while currently broken
            IRlight.setPosition(LIGHT_GREEN);
        } else if (irFlashing) {
            // Run flash for 1.5s
            if (irFlashTimer.seconds() >= IR_FLASH_DURATION_SEC) {
                irFlashing = false;
                IRlight.setPosition(LIGHT_OFF);
            } else {
                // Toggle flash on/off
                if (irFlashToggleTimer.seconds() >= IR_FLASH_PERIOD_SEC) {
                    irFlashToggleTimer.reset();
                    irFlashOn = !irFlashOn;
                }
                IRlight.setPosition(irFlashOn ? LIGHT_GREEN : LIGHT_OFF);
            }
        } else {
            IRlight.setPosition(LIGHT_OFF);
        }

        prevIntakeBroken = intakeBroken;
    }

    private void startIRFlash() {
        irFlashing = true;
        irFlashTimer.reset();
        irFlashToggleTimer.reset();
        irFlashOn = true;
    }

    // =========================================================
    // ShotAssist:
    //  - VISION RANGE FIRST (zIn) when goal tag is good
    //  - else pose distance-to-goal (FTC inches)
    //  - hold-last if invalid/jumpy
    //  - NO vision pose correction (safest)
    // =========================================================
    private void updateFromLimelightAndComputeShot_HoldLast(LLResult result) {
        boolean gotNewSolution = false;
        sawTagThisFrame = false;

        usingVisionRange = false;
        visionZInAbs = Double.NaN;
        distVisionIn = Double.NaN;
        distPoseIn = Double.NaN;

        // -------- Try VISION RANGE (preferred) --------
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            if (tags != null && !tags.isEmpty()) {
                LLResultTypes.FiducialResult chosen = null;
                for (LLResultTypes.FiducialResult t : tags) {
                    int id = (int) t.getFiducialId();
                    if ((allianceIsRed && id == RED_GOAL_ID) || (!allianceIsRed && id == BLUE_GOAL_ID)) {
                        chosen = t;
                        break;
                    }
                }
                if (chosen != null) {
                    Pose3D camPoseTarget = null;
                    try { camPoseTarget = chosen.getCameraPoseTargetSpace(); } catch (Exception ignored) {}
                    if (camPoseTarget != null) {
                        sawTagThisFrame = true;

                        Position p = camPoseTarget.getPosition();
                        double zIn = DistanceUnit.INCH.fromUnit(p.unit, p.z);
                        double zAbs = Math.abs(zIn);

                        double tx = Double.NaN;
                        try { tx = result.getTx(); } catch (Exception ignored) {}

                        if (zAbs >= VISION_MIN_Z_IN && zAbs <= VISION_MAX_Z_IN
                                && !Double.isNaN(tx) && Math.abs(tx) <= VISION_MAX_ABS_TX_DEG) {

                            visionZInAbs = zAbs;

                            // Convert camera->tag range to flywheel->goal range
                            // (vision range is authoritative for shot solution)
                            double visionDistRaw = zAbs + CAMERA_TO_FLYWHEEL_IN;
                            distVisionIn = visionDistRaw;

                            boolean visionValid = visionDistRaw >= POSE_MIN_VALID_IN && visionDistRaw <= POSE_MAX_VALID_IN;

                            if (visionValid && !Double.isNaN(lastVisionDistIn)) {
                                if (Math.abs(visionDistRaw - lastVisionDistIn) > VISION_DIST_MAX_STEP_IN_PER_LOOP) {
                                    visionValid = false;
                                }
                            }

                            if (visionValid) {
                                lastVisionDistIn = visionDistRaw;
                                usingVisionRange = true;

                                // ===== NEW: median once, then split filters =====
                                double visionDistMed = shotDistMedian.update(visionDistRaw);

                                double distForHood = shotDistLpfHood.update(visionDistMed);
                                double distForRpm  = shotDistLpfRpm.update(visionDistMed);

                                // Keep displayed "ShooterDistUsed" as HOOD distance (what hood is using)
                                shooterDistIn = distForHood;

                                // Interpolate from SPLIT distances
                                double newHoodRaw = interp(DIST_IN, HOOD_POS, distForHood);
                                double newMinRaw  = interp(DIST_IN, RPM_MIN,  distForRpm);
                                double newTgtRaw  = interp(DIST_IN, RPM_TGT,  distForRpm);
                                double newMaxRaw  = interp(DIST_IN, RPM_MAX,  distForRpm);

                                // #4: RPM slew limit (slew target; keep window width)
                                double newTgt = rpmSlew.update(newTgtRaw, RPM_SLEW_PER_LOOP);
                                double halfWin = 0.5 * Math.max(0.0, (newMaxRaw - newMinRaw));
                                double newMin = Math.max(0.0, newTgt - halfWin);
                                double newMax = newTgt + halfWin;

                                // #4: Hood deadband + slew
                                double newHood = hoodStabilizer.update(newHoodRaw, HOOD_DEADBAND, HOOD_MAX_STEP);

                                hoodCmd = newHood;
                                rpmMinCmd = newMin;
                                rpmTgtCmd = newTgt;
                                rpmMaxCmd = newMax;

                                lastShooterDistIn = shooterDistIn;
                                lastHoodCmd = newHood;
                                lastRpmMinCmd = newMin;
                                lastRpmTgtCmd = newTgt;
                                lastRpmMaxCmd = newMax;

                                // Keep legacy taps meaningful
                                lastFilteredShotDistIn = shooterDistIn;
                                haveLastFilteredShotDist = true;

                                haveLastShotSolution = true;
                                gotNewSolution = true;

                                if (teleopStarted) hood.setPosition(lastHoodCmd);
                                return; // vision wins; do not compute pose-based this frame
                            }
                        }
                    }
                }
            }
        }

        // -------- Else POSE DISTANCE (fallback) --------
        // MUST be FTC inches
        double distPoseRaw = Math.hypot(cachedGoalX - cachedRobotX, cachedGoalY - cachedRobotY) + POSE_TO_FLYWHEEL_OFFSET_IN;
        distPoseIn = distPoseRaw;

        boolean poseValid = !Double.isNaN(distPoseRaw) && distPoseRaw >= POSE_MIN_VALID_IN && distPoseRaw <= POSE_MAX_VALID_IN;

        // Freeze if distance jumps too much (prevents crazy RPM from a pose glitch)
        if (poseValid && !Double.isNaN(lastPoseDistIn)) {
            if (Math.abs(distPoseRaw - lastPoseDistIn) > POSE_MAX_STEP_IN_PER_LOOP) {
                poseValid = false;
            }
        }

        if (poseValid) {
            lastPoseDistIn = distPoseRaw;

            // ===== NEW: median once, then split filters =====
            double distPoseMed = shotDistMedian.update(distPoseRaw);

            double distForHood = shotDistLpfHood.update(distPoseMed);
            double distForRpm  = shotDistLpfRpm.update(distPoseMed);

            // Keep displayed "ShooterDistUsed" as HOOD distance (what hood is using)
            shooterDistIn = distForHood;

            double newHoodRaw = interp(DIST_IN, HOOD_POS, distForHood);
            double newMinRaw  = interp(DIST_IN, RPM_MIN,  distForRpm);
            double newTgtRaw  = interp(DIST_IN, RPM_TGT,  distForRpm);
            double newMaxRaw  = interp(DIST_IN, RPM_MAX,  distForRpm);

            // #4: RPM slew limit (slew target; keep window width)
            double newTgt = rpmSlew.update(newTgtRaw, RPM_SLEW_PER_LOOP);
            double halfWin = 0.5 * Math.max(0.0, (newMaxRaw - newMinRaw));
            double newMin = Math.max(0.0, newTgt - halfWin);
            double newMax = newTgt + halfWin;

            // #4: Hood deadband + slew
            double newHood = hoodStabilizer.update(newHoodRaw, HOOD_DEADBAND, HOOD_MAX_STEP);

            hoodCmd = newHood;
            rpmMinCmd = newMin;
            rpmTgtCmd = newTgt;
            rpmMaxCmd = newMax;

            lastShooterDistIn = shooterDistIn;
            lastHoodCmd = newHood;
            lastRpmMinCmd = newMin;
            lastRpmTgtCmd = newTgt;
            lastRpmMaxCmd = newMax;

            // Keep legacy taps meaningful
            lastFilteredShotDistIn = shooterDistIn;
            haveLastFilteredShotDist = true;

            haveLastShotSolution = true;
            gotNewSolution = true;

            if (teleopStarted) hood.setPosition(lastHoodCmd);
        }

        // HOLD LAST if invalid/jumpy
        if (!gotNewSolution && haveLastShotSolution) {
            shooterDistIn = lastShooterDistIn;

            hoodCmd = lastHoodCmd;
            rpmMinCmd = lastRpmMinCmd;
            rpmTgtCmd = lastRpmTgtCmd;
            rpmMaxCmd = lastRpmMaxCmd;

            if (teleopStarted) hood.setPosition(lastHoodCmd);
        }

        // If nothing ever computed
        if (!haveLastShotSolution) {
            shooterDistIn = Double.NaN;
            hoodCmd = HOOD_POS[0];
            if (teleopStarted) hood.setPosition(hoodCmd);
            rpmMinCmd = 0;
            rpmTgtCmd = 0;
            rpmMaxCmd = 0;
        }
    }

    private static double interp(double[] x, double[] y, double xi) {
        if (x.length != y.length || x.length == 0) return 0;
        if (xi <= x[0]) return y[0];
        if (xi >= x[x.length - 1]) return y[y.length - 1];

        int hi = 1;
        while (hi < x.length && xi > x[hi]) hi++;
        int lo = hi - 1;

        double x0 = x[lo], x1 = x[hi];
        double y0 = y[lo], y1 = y[hi];

        double t = (xi - x0) / (x1 - x0);
        return y0 + t * (y1 - y0);
    }

    // ===================== Sequence helpers =====================
    private void startFeed(SeqState feedState) {
        if (feedState == SeqState.SHOT3_FEED || feedState == SeqState.SHOT4_FEED) flipper.setPosition(FLIP_UP);
        else flipper.setPosition(FLIP_DOWN);

        intake2.setPower(FEED_PWR_INTAKE2);
        flicker.setPower(FEED_PWR_FLICKER);
        seqTimer.reset();
        state = feedState;

        // Stop reproving READY after every tiny phase:
        // Only reset stable timing when we actually FEED into the flywheel.
        resetReadyStable();
    }

    // Combined DECOMPRESS + RECOVER into one short HOLD phase (no reverse decompress)
    // Turn off extra hold drag during volley (set both holds to 0.0)
    private void startHold(SeqState holdState) {
        // Preserve your intake1 burst behavior for first shot timing
        if (holdState == SeqState.SHOT1_HOLD) intake1BurstOverride = true;

        flipper.setPosition(FLIP_DOWN);

        intake2.setPower(VOLLEY_HOLD_PWR_INTAKE2);
        flicker.setPower(VOLLEY_HOLD_PWR_FLICKER);

        seqTimer.reset();
        state = holdState;

        // IMPORTANT: do NOT resetReadyStable() here
    }

    private void stopSequenceMotors() {
        intake2.setPower(0);
        flicker.setPower(0);
        flipper.setPosition(FLIP_DOWN);
    }

    private void stopAllIntakeAndFlickerAndFlipperDown() {
        intake1.setPower(0);
        intake2.setPower(0);
        flicker.setPower(0);
        flipper.setPosition(FLIP_DOWN);
        state = SeqState.IDLE;
    }

    // ===================== Ready / shooter =====================
    private boolean isShooterReadyStable() {
        if (!shooterEnabled || !haveLastShotSolution || lastRpmTgtCmd <= 0) {
            resetReadyStable();
            return false;
        }

        double v = shooter.getVelocity();
        boolean inWindow = (v >= lastRpmMinCmd && v <= lastRpmMaxCmd);

        if (inWindow) return readyStableTimer.milliseconds() >= READY_STABLE_MS;

        resetReadyStable();
        return false;
    }

    // READY as a soft check (not hard gate)
    // - returns true if fully stable OR "close enough" (above min) to keep cadence moving
    private boolean isShooterReadySoft() {
        if (!shooterEnabled || !haveLastShotSolution || lastRpmTgtCmd <= 0) return false;

        double v = shooter.getVelocity();

        // hard fail only if we're meaningfully below min (keeps you from dumping shots when wheel is dead)
        if (v < (lastRpmMinCmd - 25.0)) return false;

        // if stable, great; if not stable but above min, allow progression (soft gate)
        return isShooterReadyStable() || (v >= lastRpmMinCmd);
    }

    private void resetReadyStable() { readyStableTimer.reset(); }

    private void stopShooter() { shooter.setPower(0); }

    // =========================================================
    // Turret aiming loop (PINPOINT HEADING ONLY)
    //  - uses cached FTC pose/heading (computed once via FieldTransform)
    //  - anti-hunt latch inside deadband
    //  - distance-based gain reduction at long range
    //  - STEP 3: heading-rate feedforward (locks while moving)
    //  - STEP 4: translation-based LOS feedforward (prevents strafe trailing)
    // =========================================================
    private void runTurretAiming(LLResult ll) {
        double dt = turretLoopTimer.seconds();
        turretLoopTimer.reset();
        dt = clamp(dt, 0.005, 0.050);

        updateVisionTrim(ll);

        if (!autoAimEnabled) {
            setTurretCmd(lastTurretCmd);
            return;
        }

        // ===================== STEP 3: heading-rate feedforward =====================
        double headingRateDegPerSec = 0.0;
        if (havePrevHeading) {
            double dHeading = wrapDeg180(cachedHeadingDegFTC - prevHeadingDegFTC);
            headingRateDegPerSec = dHeading / dt;
        }
        prevHeadingDegFTC = cachedHeadingDegFTC;
        havePrevHeading = true;

        double ffDegCW = clamp(
                TURRET_FF_GAIN * headingRateDegPerSec,
                -TURRET_FF_MAX_DEG,
                +TURRET_FF_MAX_DEG
        );

        // store for telemetry
        lastHeadingRateDegPerSec = headingRateDegPerSec;
        lastTurretFFDegCW = ffDegCW;
        // ==========================================================================

        double headingForAimDeg = cachedHeadingDegFTC; // FTC heading of robot forward

        double headingRad = Math.toRadians(headingForAimDeg);

        double fwdX = Math.cos(headingRad);
        double fwdY = Math.sin(headingRad);

        double leftX = -Math.sin(headingRad);
        double leftY =  Math.cos(headingRad);

        double turretX = cachedRobotX + fwdX * TURRET_FWD_OFFSET_IN + leftX * TURRET_LEFT_OFFSET_IN;
        double turretY = cachedRobotY + fwdY * TURRET_FWD_OFFSET_IN + leftY * TURRET_LEFT_OFFSET_IN;

        double bearingDegField = Math.toDegrees(Math.atan2(cachedGoalY - turretY, cachedGoalX - turretX));
        bearingDegField = wrapDeg180(bearingDegField);

        double relDegCCW = wrapDeg180(bearingDegField - headingForAimDeg);
        double turretDegCW_odo = -relDegCCW;

        // =========================================================
        // STEP 4: Translation-based LOS feedforward
        // =========================================================
        double vx = 0.0;
        double vy = 0.0;

        if (haveLastTurretPos) {
            vx = (turretX - lastTurretXIn) / dt;
            vy = (turretY - lastTurretYIn) / dt;
        }

        lastTurretXIn = turretX;
        lastTurretYIn = turretY;
        haveLastTurretPos = true;

        double speed = Math.hypot(vx, vy);
        lastTurretVInPerSec = speed;

        double losRateDegPerSec = 0.0; // +CCW
        double ffTransDegCW = 0.0;
        boolean ffActive = false;

        if (speed >= TRANS_MIN_SPEED_IN_S) {
            double dx = cachedGoalX - turretX;
            double dy = cachedGoalY - turretY;
            double r2 = (dx * dx) + (dy * dy);

            if (r2 > 1e-6) {
                double losRateRadPerSec = (vx * dy - vy * dx) / r2;
                losRateDegPerSec = Math.toDegrees(losRateRadPerSec);

                ffTransDegCW = clamp(
                        -(losRateDegPerSec * TRANS_LOOKAHEAD_SEC * TRANS_FF_STRENGTH),
                        -TRANS_MAX_FF_DEG,
                        +TRANS_MAX_FF_DEG
                );

                ffActive = true;
            }
        }

        lastLosRateDegPerSec = losRateDegPerSec;
        lastTransFfDegCW = ffTransDegCW;
        lastTransFfActive = ffActive;

        double turretDegCW_total = wrapDeg180(turretDegCW_odo + turretAllianceOffsetDegCW + ffDegCW + ffTransDegCW);

        if (visionTrimEnabled && turretTagSeen && Math.abs(visionTrimDegCW) < 4.0) {
            turretDegCW_total = wrapDeg180(turretDegCW_total + visionTrimDegCW);
        }

        double currentTurretDegCW = (lastTurretCmd - TURRET_HOME) / POS_PER_DEG_CW;
        double errDegCW = wrapDeg180(turretDegCW_total - currentTurretDegCW);
        lastTurretErrDegCW = errDegCW;

        double deadband = BEARING_DEADBAND_DEG;
        double unlatchBand = deadband * TURRET_UNLATCH_MULT;

        if (Math.abs(errDegCW) <= deadband) {
            turretSettleCount++;
            if (turretSettleCount >= TURRET_SETTLE_LOOPS) turretLatched = true;
        } else {
            turretSettleCount = 0;
            if (Math.abs(errDegCW) >= unlatchBand) turretLatched = false;
        }

        if (turretLatched && !ffActive) {
            setTurretCmd(lastTurretCmd);
            return;
        }

        if (Math.abs(errDegCW) < deadband) errDegCW = 0.0;

        double distToGoal = Math.hypot(cachedGoalX - cachedRobotX, cachedGoalY - cachedRobotY);
        double gainScale = clamp(48.0 / Math.max(24.0, distToGoal), 0.75, 1.00);
        double kpUsed = KP_DEG_TO_SERVO * gainScale;
        lastTurretKpUsed = kpUsed;

        double deltaServo = errDegCW * kpUsed;
        double desiredCmd = clamp(lastTurretCmd + deltaServo, SERVO_MIN_SAFE, SERVO_MAX_SAFE);

        double appliedCmd = slewTo(lastTurretCmd, desiredCmd, SERVO_SLEW_PER_SEC, dt);
        appliedCmd = clamp(appliedCmd,
                Math.max(SERVO_MIN_SAFE, lastTurretCmd - MAX_STEP_PER_LOOP),
                Math.min(SERVO_MAX_SAFE, lastTurretCmd + MAX_STEP_PER_LOOP));

        lastTurretCmd = appliedCmd;
        setTurretCmd(lastTurretCmd);
    }

    private void updateVisionTrim(LLResult result) {
        turretTagSeen = false;
        turretTagId = -1;
        txDeg = Double.NaN;
        final int wantedId = allianceIsRed ? RED_GOAL_ID : BLUE_GOAL_ID;

        if (result == null || !result.isValid()) { handleVisionLost(); return; }

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) { handleVisionLost(); return; }

        LLResultTypes.FiducialResult chosen = null;
        for (LLResultTypes.FiducialResult t : tags) {
            int id = (int) t.getFiducialId();
            if (id == wantedId) {
                chosen = t;
                break;
            }
        }
        if (chosen == null) {
            handleVisionLost();
            return;
        }

        try { txDeg = result.getTx(); } catch (Exception ignored) { txDeg = Double.NaN; }

        Pose3D pose = null;
        try { pose = chosen.getCameraPoseTargetSpace(); } catch (Exception ignored) {}
        if (pose == null) { handleVisionLost(); return; }

        Position p = pose.getPosition();
        double zIn = DistanceUnit.INCH.fromUnit(p.unit, p.z);
        double zAbs = Math.abs(zIn);

        if (zAbs < VISION_MIN_Z_IN || zAbs > VISION_MAX_Z_IN) { handleVisionLost(); return; }
        if (Double.isNaN(txDeg) || Math.abs(txDeg) > VISION_MAX_ABS_TX_DEG) { handleVisionLost(); return; }

        turretTagSeen = true;
        turretTagId = (int) chosen.getFiducialId();

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
        turretTagSeen = false;

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

    private void setTurretCmd(double cmd) {
        cmd = clamp(cmd, SERVO_MIN_SAFE, SERVO_MAX_SAFE);
        turretL.setPosition(cmd);
        turretR.setPosition(cmd);
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

    // =========================================================
    // #4 helper classes (embedded for copy/paste)
    // =========================================================
    private static class Median3Filter {
        private double a = Double.NaN, b = Double.NaN, c = Double.NaN;
        public void reset() { a = b = c = Double.NaN; }
        public double update(double x) {
            a = b; b = c; c = x;
            if (Double.isNaN(a) || Double.isNaN(b)) return x;
            return Math.max(Math.min(a, b), Math.min(Math.max(a, b), c));
        }
    }

    private static class LowPassFilter {
        private boolean init = false;
        private double y = 0.0;
        private final double alpha;
        public LowPassFilter(double alpha) { this.alpha = alpha; }
        public void reset() { init = false; y = 0.0; }
        public double update(double x) {
            if (!init) { y = x; init = true; return y; }
            y = y + alpha * (x - y);
            return y;
        }
    }

    private static class SlewLimiter {
        private double last = Double.NaN;
        public void reset() { last = Double.NaN; }
        public double update(double target, double maxDeltaPerCall) {
            if (Double.isNaN(last)) { last = target; return last; }
            double delta = target - last;
            if (delta >  maxDeltaPerCall) delta =  maxDeltaPerCall;
            if (delta < -maxDeltaPerCall) delta = -maxDeltaPerCall;
            last += delta;
            return last;
        }
    }

    private static class HoodController {
        private double last = Double.NaN;
        public void reset() { last = Double.NaN; }
        public double update(double targetPos, double deadband, double maxStep) {
            if (Double.isNaN(last)) { last = targetPos; return last; }
            if (Math.abs(targetPos - last) <= deadband) return last;
            double delta = targetPos - last;
            if (delta >  maxStep) delta =  maxStep;
            if (delta < -maxStep) delta = -maxStep;
            last += delta;
            return last;
        }
    }

    @Override
    public void stop() {
        intake1.setPower(0);
        intake2.setPower(0);
        flicker.setPower(0);
        flipper.setPosition(FLIP_DOWN);
        stopShooter();

        try { limelight.stop(); } catch (Exception ignored) {}

        driveFL.setPower(0);
        driveBL.setPower(0);
        driveFR.setPower(0);
        driveBR.setPower(0);

        if (shooterlight != null) shooterlight.setPosition(LIGHT_OFF);
        if (IRlight != null) IRlight.setPosition(LIGHT_OFF);
    }
}