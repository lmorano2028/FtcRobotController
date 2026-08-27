package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.FieldTransform;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.LoaderSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.PoseStorage;

import java.util.List;

/**
 * ONE-GAMEPAD MASTER TELEOP
 *
 * CHANGE REQUEST IMPLEMENTED (THIS EDIT):
 *  - A toggle now runs:
 *      * intake1 at full power (INTAKE1_PWR)
 *      * intake2 at 0.2 power (INTAKE2_PWR_ON_A)
 *
 * NEW CHANGE (THIS EDIT):
 *  - dpad_up (edge-detected) forces odometry pose to (0,0,90deg)
 *
 * NEW CHANGE (THIS EDIT):
 *  - REMOVED HOOD ARC BOOST (no timed hood delta during throughput sequence)
 *
 * Everything else unchanged.
 */
@TeleOp(name="Henry Tele-Op", group="Tele-Master")
public class HenryTeleOp extends OpMode {

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
    private static final double[] RPM_MIN  = { 1020, 1045, 1140, 1220, 1260, 1490 };
    private static final double[] RPM_TGT  = { 1040, 1060, 1155, 1230, 1280, 1500 };
    private static final double[] RPM_MAX  = { 1050, 1090, 1170, 1250, 1300, 1550 };

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

    // ======= INTAKES (normal toggle A) =======
    private static final double INTAKE1_PWR = 1.0;
    // >>> NEW: intake2 power when A is enabled
    private static final double INTAKE2_PWR_ON_A = 0.15;

    // Reverse eject mode power (full reverse)
    private static final double EJECT_PWR = -1.0;

    // ======= ONE-SHOT THROUGHPUT SEQUENCE (X) =======
    private static final double THR_INTAKE1_PWR = 1.0;
    private static final double THR_INTAKE2_PWR = 1.0;
    private static final double THR_FLICKER_PWR = 1.0;
    private static final double THR_FLIP_DELAY_SEC = 1.5;     // time before flipping up
    private static final double THR_FLIP_UP_HOLD_SEC = 0.35;  // hold flipper UP
    private static final double THR_FLIP_DOWN_SETTLE_SEC = 0.25; // settle DOWN before stopping motors

    private enum ThroughputState { IDLE, FEEDING_DOWN, FLIP_UP, FLIP_DOWN_SETTLE }
    private ThroughputState throughputState = ThroughputState.IDLE;

    // Timer for state transitions (resets per state)
    private final ElapsedTime throughputTimer = new ElapsedTime();
    // Timer for entire sequence since X press (does NOT reset until sequence ends)
    private final ElapsedTime throughputSeqTimer = new ElapsedTime();

    // ======= READY STABLE (NEW: 40ms latch) =======
    private static final int READY_STABLE_MS = 40;
    private final ElapsedTime readyStableTimer = new ElapsedTime();
    private boolean shooterReadyLatched = false;

    // ======= LATCHES =======
    private boolean intake1Enabled = false;
    private boolean intake2Enabled = false; // kept for compatibility, but A now drives intake2 at 0.2 when enabled
    private boolean shooterEnabled = false;

    private LoaderSubsystem loader;
    private boolean dpadRightLast = false;
    private boolean teleVolleyActive = false;

    // Reverse eject toggle (dpad_down)
    private boolean reverseEjectEnabled = false;

    // Edge detection
    private boolean prevA=false, prevB=false, prevX=false, prevY=false;
    private boolean prevDpadDown=false;
    private boolean prevDpadUp=false;
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
    double P = 180;

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

    private double lastShooterDistIn = Double.NaN;

    // Track whether we saw a tag this frame
    private boolean sawTagThisFrame = false;

    // Prevent hood motion before START
    private boolean teleopStarted = false;

    // =========================
    // Distance stabilize (filter + rpm slew + hood deadband/slew)
    // =========================
    private final Median3Filter shotDistMedian = new Median3Filter();
    private final LowPassFilter shotDistLpf = new LowPassFilter(0.30);

    private final SlewLimiter rpmSlew = new SlewLimiter();
    private final HoodController hoodStabilizer = new HoodController();

    private static final double RPM_SLEW_PER_LOOP = 70.0;
    private static final double HOOD_DEADBAND = 0.003;
    private static final double HOOD_MAX_STEP = 0.006;

    // =========================
    // TURRET AIM
    // =========================
    private static final double RED_GOAL_X  = -58.3727;
    private static final double RED_GOAL_Y  = 55.625;

    private static final double BLUE_GOAL_X = -58.3727;
    private static final double BLUE_GOAL_Y = -55.625;

    private static final double TURRET_HOME    = 0.50;
    private static final double POS_PER_DEG_CW = 0.005368;
    private static final double SERVO_MIN_SAFE = 0.10;
    private static final double SERVO_MAX_SAFE = 0.90;

    private static final double POD_X_OFFSET_MM = 82.55;
    private static final double POD_Y_OFFSET_MM = -95.25;

    private static final double TURRET_FWD_OFFSET_IN  = -4.0;
    private static final double TURRET_LEFT_OFFSET_IN =  0.0;

    // ======= TURRET STABILITY TUNING (ANTI-HUNT) =======
    private static final double BEARING_DEADBAND_DEG = 0.90;
    private static final int    TURRET_SETTLE_LOOPS  = 3;
    private static final double TURRET_UNLATCH_MULT  = 1.4;

    private static final double SERVO_SLEW_PER_SEC   = 2.8;
    private static final double MAX_STEP_PER_LOOP    = 0.045;
    private static final double KP_DEG_TO_SERVO      = 0.0046;

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

    // ======= Alliance turret offset (apply only after START) =======
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
    // STEP 3: Heading-rate feedforward
    // =========================================================
    private double prevHeadingDegFTC = 0.0;
    private boolean havePrevHeading = false;

    private static final double TURRET_FF_GAIN = 0.035;
    private static final double TURRET_FF_MAX_DEG = 2.50;

    // Telemetry taps for Step 3
    private double lastHeadingRateDegPerSec = 0.0;
    private double lastTurretFFDegCW = 0.0;

    // =========================================================
    // STEP 4: Translation-based LOS feedforward
    // =========================================================
    private static final double TRANS_LOOKAHEAD_SEC = 0.10;
    private static final double TRANS_MIN_SPEED_IN_S = 4.0;
    private static final double TRANS_MAX_FF_DEG = 6.0;
    private static final double TRANS_FF_STRENGTH = 1.0;

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
    private static final double POSE_MAX_STEP_IN_PER_LOOP = 12.0;
    private static final double POSE_TO_FLYWHEEL_OFFSET_IN = 0.0;

    // Vision range gating
    private static final double VISION_DIST_MAX_STEP_IN_PER_LOOP = 18.0;

    private double lastPoseDistIn = Double.NaN;
    private double lastVisionDistIn = Double.NaN;

    // =========================================================
    // ODO CACHE (odo.update() ONCE PER LOOP)
    // =========================================================
    private double cachedRobotX = 0.0;      // FTC inches
    private double cachedRobotY = 0.0;      // FTC inches
    private double cachedRobotXPin = 0.0;   // Pinpoint inches (telemetry)
    private double cachedRobotYPin = 0.0;   // Pinpoint inches (telemetry)
    private double cachedHeadingDegFTC = 0.0;
    private double cachedGoalX = 0.0;
    private double cachedGoalY = 0.0;

    // Shooter distance sources (telemetry)
    private boolean usingVisionRange = false;
    private double visionZInAbs = Double.NaN;
    private double distPoseIn = Double.NaN;
    private double distVisionIn = Double.NaN;

    // Turret anti-hunt state (latch)
    private int turretSettleCount = 0;
    private boolean turretLatched = false;
    private double lastTurretErrDegCW = 0.0;
    private double lastTurretKpUsed = KP_DEG_TO_SERVO;

    // PoseStorage validity telemetry
    private boolean poseStorageWasValidInInit = false;

    // =========================================================
    // FORCE ODO POSE (Pinpoint) from FTC inches + FTC heading deg
    // =========================================================
    private void forcePoseFtcIn(double xInFtc, double yInFtc, double headingDegFtc) {
        // Pinpoint Pose2D uses DistanceUnit.MM here
        double xMm = xInFtc * 25.4;
        double yMm = yInFtc * 25.4;

        Pose2D newPose = new Pose2D(
                DistanceUnit.MM, xMm, yMm,
                AngleUnit.DEGREES, headingDegFtc
        );

        odo.setPosition(newPose);
        odo.update();

        // Refresh cache immediately so shot-assist / telemetry see the new pose this loop
        updateOdoCacheOnce();

        telemetry.addLine("DPAD_UP: Forced Pose -> X=0 Y=0 H=90");
    }

    private void setHoodCmd(double cmd) {
        cmd = clamp(cmd, 0.0, 1.0);
        hood.setPosition(cmd);
    }

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
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, 0, 0, F));

        flicker.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake1.setDirection(DcMotor.Direction.REVERSE);

        flicker.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        poseStorageWasValidInInit = PoseStorage.valid;

        odo.resetPosAndIMU();
        odo.update();

        // Vision trim state init
        visionTrimDegCW = 0.0;
        visionTrimDegCW_raw = 0.0;
        visionHasGood = false;
        visionLastGoodTimer.reset();
        visionDecayTimer.reset();

        loader = new LoaderSubsystem(
                intake2,
                flicker,
                flipper,
                null,
                0.662,
                0.39,
                0.12,
                0.05,
                1.0,
                1.0,
                150,
                -0.15,
                40,
                120
        );

        haveLastShotSolution = false;
        lastHoodCmd = hoodCmd;
        lastRpmMinCmd = rpmMinCmd;
        lastRpmTgtCmd = rpmTgtCmd;
        lastRpmMaxCmd = rpmMaxCmd;
        lastShooterDistIn = shooterDistIn;

        shotDistMedian.reset();
        shotDistLpf.reset();
        rpmSlew.reset();
        hoodStabilizer.reset();

        turretLoopTimer.reset();

        // ===== STEP 3 init =====
        havePrevHeading = false;
        prevHeadingDegFTC = 0.0;
        lastHeadingRateDegPerSec = 0.0;
        lastTurretFFDegCW = 0.0;

        // ===== STEP 4 init =====
        haveLastTurretPos = false;
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

        // No alliance offset until START
        turretAllianceOffsetDegCW = 0.0;

        lastPoseDistIn = Double.NaN;
        lastVisionDistIn = Double.NaN;

        turretSettleCount = 0;
        turretLatched = false;

        throughputState = ThroughputState.IDLE;
        throughputTimer.reset();
        throughputSeqTimer.reset();

        shooterReadyLatched = false;
        readyStableTimer.reset();

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("MasterTeleOp READY");
        telemetry.addLine("INIT: dpad_left=RED, dpad_right=BLUE");
        telemetry.addLine("Drive: LS translate, RSX rotate");
        telemetry.addLine("dpad_left reset yaw | dpad_down toggle reverse eject");
        telemetry.addLine("dpad_up FORCE POSE -> (0,0,90)");
        telemetry.addLine("A intake1 toggle (intake2 @ 0.2) | Y shooter toggle | X start ONE-SHOT loading | B STOP loading/intakes");
        telemetry.addLine("RB(toggle) turret autoaim | LB turret home");
        telemetry.addLine("IR: beamIntake -> IRlight GREEN (solid/flash)");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_left)  allianceIsRed = true;
        if (gamepad1.dpad_right) allianceIsRed = false;

        updateOdoCacheOnce();
        updateFromLimelightAndComputeShot_HoldLast(limelight.getLatestResult());
        updateIRLightFromBeam();

        telemetry.addLine("=== INIT ===");
        telemetry.addData("Alliance", allianceIsRed ? "RED" : "BLUE");
        telemetry.addData("HaveLastShotSolution", haveLastShotSolution);
        telemetry.addData("Last RPM tgt", "%.0f", lastRpmTgtCmd);
        telemetry.addData("Last Hood", "%.3f", lastHoodCmd);
        telemetry.addData("beamIntake(raw)", beamIntake.getState());
        telemetry.addData("IntakeBroken", !beamIntake.getState());
        telemetry.addData("PoseStorage.valid", PoseStorage.valid);
        telemetry.update();
    }

    @Override
    public void start() {
        teleopStarted = true;

        // Seed Pinpoint at START
        if (PoseStorage.valid) {
            odo.setPosition(FieldTransform.poseStorageToPinpointPose2D());
            odo.update();

            lastPoseDistIn = Double.NaN;
            lastVisionDistIn = Double.NaN;

            PoseStorage.clear();
        }

        shotDistMedian.reset();
        shotDistLpf.reset();
        rpmSlew.reset();
        hoodStabilizer.reset();

        turretAllianceOffsetDegCW = allianceIsRed ? +TURRET_ALLIANCE_OFFSET_DEG : -TURRET_ALLIANCE_OFFSET_DEG;

        setHoodCmd(hoodCmd);

        flipper.setPosition(FLIP_DOWN);

        stopAllFeedAndFlipperDown();
        stopShooter();

        lastTurretCmd = TURRET_HOME;
        setTurretCmd(lastTurretCmd);

        turretSettleCount = 0;
        turretLatched = false;

        prevHeadingDegFTC = cachedHeadingDegFTC;
        havePrevHeading = true;
        lastHeadingRateDegPerSec = 0.0;
        lastTurretFFDegCW = 0.0;

        haveLastTurretPos = false;
        lastTransFfActive = false;

        shooterlight.setPosition(LIGHT_OFF);

        IRlight.setPosition(LIGHT_OFF);
        prevIntakeBroken = !beamIntake.getState();
        irFlashing = false;
        irFlashTimer.reset();
        irFlashToggleTimer.reset();
        irFlashOn = false;

        throughputState = ThroughputState.IDLE;
        throughputTimer.reset();
        throughputSeqTimer.reset();

        shooterReadyLatched = false;
        readyStableTimer.reset();

    }

    @Override
    public void loop() {
        // ===================== DRIVE (ROBOT-CENTRIC) =====================
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        if (gamepad1.dpad_left) imu.resetYaw();

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX *= 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

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

        // ===================== ODO UPDATE ONCE =====================
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

        boolean dpadDown = gamepad1.dpad_down;
        boolean dpadDownPressed = dpadDown && !prevDpadDown;

        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadUpPressed = dpadUp && !prevDpadUp;

        boolean rb = gamepad1.right_bumper;
        boolean rbPressed = rb && !prevRB;

        // ===================== SHOT SOLUTION =====================
        updateFromLimelightAndComputeShot_HoldLast(ll);

        // ===================== IR LIGHT =====================
        updateIRLightFromBeam();

        // ===================== Reverse eject toggle =====================
        if (dpadDownPressed) reverseEjectEnabled = !reverseEjectEnabled;

        // ===================== DPAD_UP = FORCE POSE (0,0,90) =====================
        if (dpadUpPressed) {
            forcePoseFtcIn(0.0, 0.0, 90.0);
        }

        // ===================== Intake1 toggle (A) =====================
        if (aPressed) {
            intake1Enabled = !intake1Enabled;
            intake2Enabled = intake1Enabled; // keep boolean consistent (A drives intake2 as well)
        }

        // ===================== Shooter toggle (Y) =====================
        if (yPressed) {
            shooterEnabled = !shooterEnabled;
            resetReadyStable();
            shooterReadyLatched = false;
            if (!shooterEnabled) stopShooter();
        }
        // ========================= far range shot( dpad right)
        boolean dpadRightNow = gamepad1.dpad_right;
        boolean dpadRightPressed = dpadRightNow && !dpadRightLast;

        if (dpadRightPressed && !teleVolleyActive && loader.isIdle()) {
            teleVolleyActive = true;
            resetReadyStable();
            loader.startFourShot();
        }

        dpadRightLast = dpadRightNow;

        if (teleVolleyActive) {
            loader.updateFourShot(isShooterReadyStable());

            if (loader.getState() == LoaderSubsystem.SeqState.DONE || loader.isIdle()) {
                teleVolleyActive = false;
                loader.stopAll();
            }
        }

        // ===================== X = START ONE-SHOT LOADING SEQUENCE =====================
        if (xPressed) {
            reverseEjectEnabled = false;

            throughputState = ThroughputState.FEEDING_DOWN;
            throughputTimer.reset();

            throughputSeqTimer.reset(); // starts counting from X press
            flipper.setPosition(FLIP_DOWN);
        }

        // ===================== B = STOP LOADING SEQUENCE + FEED MOTORS =====================
        if (bPressed) {

            // Kill ALL loading systems
            throughputState = HenryTeleOp.ThroughputState.IDLE;
            teleVolleyActive = false;
            reverseEjectEnabled = false;

            // Stop loader sequence
            if (loader != null) {
                loader.stopAll();
            }

            // Stop motors
            stopAllFeedAndFlipperDown();

            // Reset readiness so volley doesn't auto-resume
            shooterReadyLatched = false;
            resetReadyStable();
        }

        boolean throughputActive = (throughputState != ThroughputState.IDLE);

        // ===================== Motor power application =====================
        if (reverseEjectEnabled && !throughputActive && !teleVolleyActive) {

            intake1.setPower(EJECT_PWR);
            intake2.setPower(EJECT_PWR);
            flicker.setPower(0.0);
            flipper.setPosition(FLIP_DOWN);

        } else if (teleVolleyActive) {

            // Let loader control intake2 + flicker internally
            // Only drive intake1 here
            intake1.setPower(INTAKE1_PWR);

        } else if (throughputActive) {

            // Existing X-button sequence (UNCHANGED)
            intake1.setPower(THR_INTAKE1_PWR);
            intake2.setPower(THR_INTAKE2_PWR);
            flicker.setPower(THR_FLICKER_PWR);

            double t = throughputTimer.seconds();

            switch (throughputState) {

                case FEEDING_DOWN:
                    flipper.setPosition(FLIP_DOWN);
                    if (t >= THR_FLIP_DELAY_SEC) {
                        throughputState = HenryTeleOp.ThroughputState.FLIP_UP;
                        throughputTimer.reset();
                    }
                    break;

                case FLIP_UP:
                    flipper.setPosition(FLIP_UP);
                    if (t >= THR_FLIP_UP_HOLD_SEC) {
                        throughputState = HenryTeleOp.ThroughputState.FLIP_DOWN_SETTLE;
                        throughputTimer.reset();
                    }
                    break;

                case FLIP_DOWN_SETTLE:
                    flipper.setPosition(FLIP_DOWN);
                    if (t >= THR_FLIP_DOWN_SETTLE_SEC) {
                        throughputState = HenryTeleOp.ThroughputState.IDLE;
                        stopAllFeedAndFlipperDown();
                    }
                    break;

                default:
                    throughputState = HenryTeleOp.ThroughputState.IDLE;
                    stopAllFeedAndFlipperDown();
                    break;
            }

        } else {

            // Normal driving intake behavior
            intake1.setPower(intake1Enabled ? INTAKE1_PWR : 0.0);
            intake2.setPower(intake1Enabled ? INTAKE2_PWR_ON_A : 0.0);

            flicker.setPower(0.0);
            flipper.setPosition(FLIP_DOWN);
        }

        // ===================== Shooter command =====================
        if (shooterEnabled && haveLastShotSolution && lastRpmTgtCmd > 0) {
            shooter.setVelocity(lastRpmTgtCmd);
        } else {
            stopShooter();
        }

        // ===================== SHOOTERLIGHT =====================
        // Blue when throughputActive, else orange when READY, else off.
        boolean readyNow = isShooterReadyStable();
        if (throughputActive) {
            shooterlight.setPosition(LIGHT_BLUE);
        } else {
            shooterlight.setPosition(readyNow ? LIGHT_ORANGE : LIGHT_OFF);
        }

        // ===================== TURRET CONTROLS =====================
        if (rbPressed) autoAimEnabled = !autoAimEnabled;

        if (gamepad1.left_bumper) {
            lastTurretCmd = TURRET_HOME;
            setTurretCmd(lastTurretCmd);
            turretSettleCount = 0;
            turretLatched = false;

            haveLastTurretPos = false;
            lastTransFfActive = false;
        } else {
            runTurretAiming(ll);
        }

        // ===================== TELEMETRY =====================
        double vShooter = shooter.getVelocity();

        telemetry.addLine("=== One-Pad Master TeleOp ===");
        telemetry.addData("Alliance (init)", allianceIsRed ? "RED" : "BLUE");

        telemetry.addLine("--- Intake/Loading ---");
        telemetry.addData("LoadState(X)", throughputState);
        telemetry.addData("StateTimer(s)", "%.2f", throughputTimer.seconds());
        telemetry.addData("SeqTimer(s)", "%.2f", throughputSeqTimer.seconds());
        telemetry.addData("Intake1Enabled(A)", intake1Enabled);
        telemetry.addData("Intake2(A@0.15)", intake1Enabled);
        telemetry.addData("ReverseEject(dpad_down)", reverseEjectEnabled);

        telemetry.addLine("--- Shooter ---");
        telemetry.addData("ShooterEnabled(Y)", shooterEnabled);
        telemetry.addData("HaveLastShotSolution", haveLastShotSolution);
        telemetry.addData("RPM tgt (held)", "%.0f", lastRpmTgtCmd);
        telemetry.addData("ShooterVel", "%.0f", vShooter);
        telemetry.addData("HoodVal(now)", "%.3f", hood.getPosition());
        telemetry.addData("HoodVal(cmd)", "%.3f", lastHoodCmd);
        telemetry.addData("READY(latched)", readyNow);

        telemetry.addLine("--- IR Beam ---");
        telemetry.addData("IntakeBroken", !beamIntake.getState());
        telemetry.addData("IR Flushing", irFlashing);

        telemetry.addLine("--- Turret ---");
        telemetry.addData("AutoAim (RB)", autoAimEnabled);
        telemetry.addData("TurretCmd", "%.3f", lastTurretCmd);
        telemetry.addData("TurretErrDegCW", "%.2f", lastTurretErrDegCW);
        telemetry.addData("TurretLatched", turretLatched);
        telemetry.addData("KpUsed", "%.5f", lastTurretKpUsed);

        telemetry.update();

        // ===================== Save prev =====================
        prevA=a; prevB=b; prevX=xBtn; prevY=yBtn;
        prevDpadDown = dpadDown;
        prevDpadUp = dpadUp;
        prevRB = rb;
    }

    // =========================================================
    // Stop helpers (loading / feed motors)
    // =========================================================
    private void stopAllFeedAndFlipperDown() {
        intake1.setPower(0.0);
        intake2.setPower(0.0);
        flicker.setPower(0.0);
        flipper.setPosition(FLIP_DOWN);
    }

    // =========================================================
    // ODO CACHE
    // =========================================================
    private void updateOdoCacheOnce() {
        odo.update();

        Pose2D pp = odo.getPosition();

        cachedRobotX = FieldTransform.pinpointPoseToFtcXIn(pp);
        cachedRobotY = FieldTransform.pinpointPoseToFtcYIn(pp);
        cachedHeadingDegFTC = FieldTransform.pinpointPoseToFtcHeadingDeg(pp);

        cachedRobotXPin = FieldTransform.mmToIn(pp.getX(DistanceUnit.MM));
        cachedRobotYPin = FieldTransform.mmToIn(pp.getY(DistanceUnit.MM));

        cachedGoalX = allianceIsRed ? RED_GOAL_X : BLUE_GOAL_X;
        cachedGoalY = allianceIsRed ? RED_GOAL_Y : BLUE_GOAL_Y;
    }

    // =========================================================
    // IR Light behavior (beamIntake -> IRlight)
    // =========================================================
    private void updateIRLightFromBeam() {
        boolean intakeBroken = !beamIntake.getState();

        boolean intakeClearedEvent = (prevIntakeBroken && !intakeBroken);
        if (intakeClearedEvent) startIRFlash();

        if (intakeBroken) {
            irFlashing = false;
            IRlight.setPosition(LIGHT_GREEN);
        } else if (irFlashing) {
            if (irFlashTimer.seconds() >= IR_FLASH_DURATION_SEC) {
                irFlashing = false;
                IRlight.setPosition(LIGHT_OFF);
            } else {
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
    // ShotAssist (unchanged except hood command calls -> setHoodCmd)
    // =========================================================
    private void updateFromLimelightAndComputeShot_HoldLast(LLResult result) {
        boolean gotNewSolution = false;
        sawTagThisFrame = false;

        usingVisionRange = false;
        visionZInAbs = Double.NaN;
        distVisionIn = Double.NaN;
        distPoseIn = Double.NaN;

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

                                double visionDistFilt = filterShotDistanceIn(visionDistRaw);
                                shooterDistIn = visionDistFilt;

                                double newHoodRaw = interp(DIST_IN, HOOD_POS, shooterDistIn);
                                double newMinRaw  = interp(DIST_IN, RPM_MIN,  shooterDistIn);
                                double newTgtRaw  = interp(DIST_IN, RPM_TGT,  shooterDistIn);
                                double newMaxRaw  = interp(DIST_IN, RPM_MAX,  shooterDistIn);

                                double newTgt = rpmSlew.update(newTgtRaw, RPM_SLEW_PER_LOOP);
                                double halfWin = 0.5 * Math.max(0.0, (newMaxRaw - newMinRaw));
                                double newMin = Math.max(0.0, newTgt - halfWin);
                                double newMax = newTgt + halfWin;

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

                                haveLastShotSolution = true;
                                gotNewSolution = true;

                                if (teleopStarted) setHoodCmd(lastHoodCmd);
                                return;
                            }
                        }
                    }
                }
            }
        }

        double distPoseRaw = Math.hypot(cachedGoalX - cachedRobotX, cachedGoalY - cachedRobotY) + POSE_TO_FLYWHEEL_OFFSET_IN;
        distPoseIn = distPoseRaw;

        boolean poseValid = !Double.isNaN(distPoseRaw) && distPoseRaw >= POSE_MIN_VALID_IN && distPoseRaw <= POSE_MAX_VALID_IN;

        if (poseValid && !Double.isNaN(lastPoseDistIn)) {
            if (Math.abs(distPoseRaw - lastPoseDistIn) > POSE_MAX_STEP_IN_PER_LOOP) {
                poseValid = false;
            }
        }

        if (poseValid) {
            lastPoseDistIn = distPoseRaw;

            double distPoseFilt = filterShotDistanceIn(distPoseRaw);
            shooterDistIn = distPoseFilt;

            double newHoodRaw = interp(DIST_IN, HOOD_POS, shooterDistIn);
            double newMinRaw  = interp(DIST_IN, RPM_MIN,  shooterDistIn);
            double newTgtRaw  = interp(DIST_IN, RPM_TGT,  shooterDistIn);
            double newMaxRaw  = interp(DIST_IN, RPM_MAX,  shooterDistIn);

            double newTgt = rpmSlew.update(newTgtRaw, RPM_SLEW_PER_LOOP);
            double halfWin = 0.5 * Math.max(0.0, (newMaxRaw - newMinRaw));
            double newMin = Math.max(0.0, newTgt - halfWin);
            double newMax = newTgt + halfWin;

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

            haveLastShotSolution = true;
            gotNewSolution = true;

            if (teleopStarted) setHoodCmd(lastHoodCmd);
        }

        if (!gotNewSolution && haveLastShotSolution) {
            shooterDistIn = lastShooterDistIn;
            hoodCmd = lastHoodCmd;
            rpmMinCmd = lastRpmMinCmd;
            rpmTgtCmd = lastRpmTgtCmd;
            rpmMaxCmd = lastRpmMaxCmd;

            if (teleopStarted) setHoodCmd(lastHoodCmd);
        }

        if (!haveLastShotSolution) {
            shooterDistIn = Double.NaN;
            hoodCmd = HOOD_POS[0];
            if (teleopStarted) setHoodCmd(hoodCmd);
            rpmMinCmd = 0;
            rpmTgtCmd = 0;
            rpmMaxCmd = 0;
        }
    }

    private double filterShotDistanceIn(double distInRaw) {
        double med = shotDistMedian.update(distInRaw);
        return shotDistLpf.update(med);
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

    // ===================== Ready / shooter (40ms latch) =====================
    private boolean isShooterReadyStable() {
        if (!shooterEnabled || !haveLastShotSolution || lastRpmTgtCmd <= 0) {
            shooterReadyLatched = false;
            resetReadyStable();
            return false;
        }

        double v = shooter.getVelocity();
        boolean inWindow = (v >= lastRpmMinCmd && v <= lastRpmMaxCmd);

        if (!inWindow) {
            shooterReadyLatched = false;
            resetReadyStable();
            return false;
        }

        // We are in-window. Latch after 40ms in-window.
        if (!shooterReadyLatched) {
            if (readyStableTimer.milliseconds() >= READY_STABLE_MS) {
                shooterReadyLatched = true;
            }
        }

        // Once latched, stays ready while in-window.
        return shooterReadyLatched;
    }

    private void resetReadyStable() {
        readyStableTimer.reset();
    }

    private void stopShooter() { shooter.setPower(0); }

    // =========================================================
    // Turret aiming loop (unchanged)
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

        lastHeadingRateDegPerSec = headingRateDegPerSec;
        lastTurretFFDegCW = ffDegCW;

        double headingForAimDeg = cachedHeadingDegFTC;
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

        double losRateDegPerSec = 0.0;
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
        if (chosen == null) { handleVisionLost(); return; }

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
    // Helper classes
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
        stopAllFeedAndFlipperDown();
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