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

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.PoseStorage;

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
 */
@TeleOp(name="MasterTeleOp_Decode", group="Test")
public class MasterTeleOp_Decode extends OpMode {

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

    // ======= RGB LIGHT NAME =======
    private static final String STATUS_LIGHT_NAME = "statuslight";

    // ======= RGB LIGHT POSITIONS (goBILDA chart FTC positions) =======
    private static final double LIGHT_OFF    = 0.000; // Off (500us)
    private static final double LIGHT_ORANGE = 0.333; // Orange (1200us)
    private static final double LIGHT_BLUE   = 0.611; // Blue (1700us)

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
    private static final double IN_PER_MM = 1.0 / 25.4;

    // ======= SHOT MAP TABLE (YOUR VALUES) =======
    private static final double[] DIST_IN  = { 24, 48, 80, 120 };
    private static final double[] HOOD_POS = { 0.350, 0.525, 0.790, 0.790 };
    private static final double[] RPM_MIN  = {  970, 1070, 1150, 1400 };
    private static final double[] RPM_TGT  = { 1000, 1100, 1180, 1440 };
    private static final double[] RPM_MAX  = { 1030, 1130, 1210, 1470 };

    // ======= HARDWARE =======
    private DcMotor driveFR, driveFL, driveBR, driveBL;
    private IMU imu;

    private Limelight3A limelight;

    private DcMotorEx shooter, flicker, intake1, intake2;
    private Servo hood, flipper;

    private GoBildaPinpointDriver odo;
    private Servo turretL, turretR;

    // RGB status light
    private Servo statuslight;

    // ======= FLIPPER POSITIONS =======
    private static final double FLIP_DOWN = 0.662;
    private static final double FLIP_UP   = 0.437;

    // ======= INTAKES =======
    private static final double INTAKE1_PWR = 0.8;
    private static final double INTAKE1_BURST_PWR = 1.0;

    // Reverse eject mode power (full reverse)
    private static final double EJECT_PWR = -1.0;

    // ======= FEED / HOLD / DECOMPRESS =======
    private static final double HOLD_PWR_INTAKE2 = 0.12;
    private static final double HOLD_PWR_FLICKER = 0.05;

    private static final double FEED_PWR_INTAKE2 = 1.0;
    private static final double FEED_PWR_FLICKER = 1.0;
    private static final int FEED_MS = 150;

    private static final double DECOMPRESS_PWR_INTAKE2 = -0.15;
    private static final int DECOMPRESS_MS = 40;

    // ======= READY STABLE =======
    private static final int READY_STABLE_MS = 110;
    private static final int MIN_RECOVER_MS  = 120;
    private static final int SHOT12_SPACING_MS = 220;

    // ======= READY RUMBLE (REMOVED) =======
    // private boolean wasReady = false;
    // private static final int READY_RUMBLE_MS = 220;

    // ======= LATCHES =======
    private boolean intake1Enabled = false;
    private boolean shooterEnabled = false;
    private boolean intake1BurstOverride = false;

    // Reverse eject toggle (NOW dpad_down)
    private boolean reverseEjectEnabled = false;

    // ======= STATE MACHINE =======
    private enum SeqState {
        IDLE,
        SHOT1_FEED, SHOT1_DECOMP, SHOT1_RECOVER,
        SHOT2_FEED, SHOT2_DECOMP, SHOT2_RECOVER,
        SHOT3_FEED, SHOT3_DECOMP, SHOT3_RECOVER,
        SHOT4_FEED, SHOT4_DECOMP, SHOT4_RECOVER,
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

    // ======= “LIVE” SHOT ASSIST OUTPUTS =======
    private double shooterDistIn = Double.NaN;

    // P and F values
    double F = 17.73;//prev 16.53
    double P = 266;

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

    // Track whether we saw a tag this frame
    private boolean sawTagThisFrame = false;

    // Prevent hood motion before START
    private boolean teleopStarted = false;

    // =========================
    // TURRET AIM (from your turret teleop)
    // =========================

    private static final double RED_GOAL_X  = -58.3727;
    private static final double RED_GOAL_Y  = +55.6425;

    private static final double BLUE_GOAL_X = -58.3727;
    private static final double BLUE_GOAL_Y = -55.6425;

    private static final double TURRET_HOME    = 0.50;
    private static final double POS_PER_DEG_CW = 0.007643;
    private static final double SERVO_MIN_SAFE = 0.10;
    private static final double SERVO_MAX_SAFE = 0.90;

    private static final double POD_X_OFFSET_MM = 82.55;
    private static final double POD_Y_OFFSET_MM = -95.25;

    private static final double TURRET_FWD_OFFSET_IN  = -4.0;
    private static final double TURRET_LEFT_OFFSET_IN =  0.0;

    private static final double PINPOINT_TO_FTC_X_SIGN = -1.0;
    private static final double PINPOINT_TO_FTC_Y_SIGN = -1.0;

    private static double pinpointXToFtc(double xPin) { return PINPOINT_TO_FTC_X_SIGN * xPin; }
    private static double pinpointYToFtc(double yPin) { return PINPOINT_TO_FTC_Y_SIGN * yPin; }

    private static double ftcXToPinpoint(double xFtc) { return PINPOINT_TO_FTC_X_SIGN * xFtc; }
    private static double ftcYToPinpoint(double yFtc) { return PINPOINT_TO_FTC_Y_SIGN * yFtc; }

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

    private boolean allianceIsRed = true;
    private boolean autoAimEnabled = true;

    private double lastTurretCmd = TURRET_HOME;
    private final ElapsedTime turretLoopTimer = new ElapsedTime();

    private double headingOffsetDeg = 0.0;
    private double seededHeadingDeg = 0.0;

    private final ElapsedTime imuSettleTimer = new ElapsedTime();
    private boolean headingOffsetLocked = false;
    private double lastRawForStability = 0.0;
    private boolean haveLastRaw = false;
    private int stableCount = 0;

    private static final double IMU_MIN_SETTLE_MS = 350;
    private static final double IMU_STABLE_EPS_DEG = 0.6;
    private static final int IMU_STABLE_LOOPS = 6;

    private boolean visionTrimEnabled = true;
    private boolean visionHasGood = false;
    private double visionTrimDegCW = 0.0;
    private double visionTrimDegCW_raw = 0.0;
    private final ElapsedTime visionLastGoodTimer = new ElapsedTime();
    private final ElapsedTime visionDecayTimer = new ElapsedTime();

    private boolean turretTagSeen = false;
    private int turretTagId = -1;
    private double txDeg = Double.NaN;

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

        odo.resetPosAndIMU();
        imuSettleTimer.reset();
        headingOffsetLocked = false;
        haveLastRaw = false;
        stableCount = 0;

        Pose2D seedPose;
        if (PoseStorage.valid) {
            seedPose = new Pose2D(
                    DistanceUnit.INCH,
                    ftcXToPinpoint(PoseStorage.xIn),
                    ftcYToPinpoint(PoseStorage.yIn),
                    AngleUnit.DEGREES,
                    PoseStorage.headingDeg
            );
            seededHeadingDeg = wrapDeg180(PoseStorage.headingDeg);
            PoseStorage.clear();
        } else {
            seedPose = new Pose2D(
                    DistanceUnit.INCH,
                    ftcXToPinpoint(0.0), ftcYToPinpoint(0.0),
                    AngleUnit.DEGREES,
                    0.0
            );
            seededHeadingDeg = 0.0;
        }

        odo.setPosition(seedPose);
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

        turretLoopTimer.reset();

        // ===== RGB STATUS LIGHT =====
        statuslight = hardwareMap.get(Servo.class, STATUS_LIGHT_NAME);
        statuslight.setPosition(LIGHT_OFF);

        teleopStarted = false;

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("MasterTeleOp One-Gamepad READY");
        telemetry.addLine("INIT: dpad_left=RED, dpad_right=BLUE");
        telemetry.addLine("Drive: LS translate, RSX rotate");
        telemetry.addLine("dpad_left reset yaw | dpad_down toggle reverse eject");
        telemetry.addLine("A intake toggle | Y shooter toggle | X 4-shot | B stop intake+flicker+flipper down");
        telemetry.addLine("RB(toggle) turret autoaim | LB turret home");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Alliance select in INIT only
        if (gamepad1.dpad_left)  allianceIsRed = true;
        if (gamepad1.dpad_right) allianceIsRed = false;

        odo.update();
        double raw = getRawHeadingDeg();
        maybeLockHeadingOffset(raw);

        updateFromLimelightAndComputeShot_HoldLast(limelight.getLatestResult());

        telemetry.addLine("=== INIT ===");
        telemetry.addData("Alliance", allianceIsRed ? "RED" : "BLUE");
        telemetry.addData("HeadingOffsetLocked", headingOffsetLocked);
        telemetry.addData("HaveLastShotSolution", haveLastShotSolution);
        telemetry.addData("Last RPM tgt", "%.0f", lastRpmTgtCmd);
        telemetry.addData("Last Hood", "%.3f", lastHoodCmd);
        telemetry.update();
    }

    @Override
    public void start() {
        teleopStarted = true;

        // Per request: initial mechanical positioning happens only upon START
        hood.setPosition(hoodCmd);
        flipper.setPosition(FLIP_DOWN);

        stopAllIntakeAndFlickerAndFlipperDown();
        stopShooter();

        lastTurretCmd = TURRET_HOME;
        setTurretCmd(lastTurretCmd);

        statuslight.setPosition(LIGHT_OFF);
    }

    @Override
    public void loop() {
        // ===================== DRIVE =====================
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // UPDATED: Reset yaw is now dpad_left
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

        // ===================== SHOT SOLUTION (HOLD LAST) =====================
        updateFromLimelightAndComputeShot_HoldLast(ll);

        // ===================== RGB STATUS LIGHT =====================
        // Blue during 4-shot sequence
        // Orange when shooter ready stable
        // Off otherwise (including shooter disabled)
        if (!shooterEnabled) {
            statuslight.setPosition(LIGHT_OFF);
        } else if (state != SeqState.IDLE && state != SeqState.DONE && state != SeqState.ABORTED) {
            statuslight.setPosition(LIGHT_BLUE);
        } else {
            boolean readyNow = isShooterReadyStable();
            if (readyNow) statuslight.setPosition(LIGHT_ORANGE);
            else statuslight.setPosition(LIGHT_OFF);
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
                if (seqTimer.milliseconds() >= FEED_MS) startDecompress(SeqState.SHOT1_DECOMP);
                break;

            case SHOT1_DECOMP:
                if (seqTimer.milliseconds() >= DECOMPRESS_MS) startRecover(SeqState.SHOT1_RECOVER);
                break;

            case SHOT1_RECOVER:
                if (seqTimer.milliseconds() >= SHOT12_SPACING_MS && isShooterReadyStable()) startFeed(SeqState.SHOT2_FEED);
                break;

            case SHOT2_FEED:
                if (seqTimer.milliseconds() >= FEED_MS) startDecompress(SeqState.SHOT2_DECOMP);
                break;

            case SHOT2_DECOMP:
                if (seqTimer.milliseconds() >= DECOMPRESS_MS) startRecover(SeqState.SHOT2_RECOVER);
                break;

            case SHOT2_RECOVER:
                if (seqTimer.milliseconds() >= MIN_RECOVER_MS && isShooterReadyStable()) startFeed(SeqState.SHOT3_FEED);
                break;

            case SHOT3_FEED:
                if (seqTimer.milliseconds() >= FEED_MS) startDecompress(SeqState.SHOT3_DECOMP);
                break;

            case SHOT3_DECOMP:
                if (seqTimer.milliseconds() >= DECOMPRESS_MS) startRecover(SeqState.SHOT3_RECOVER);
                break;

            case SHOT3_RECOVER:
                if (seqTimer.milliseconds() >= MIN_RECOVER_MS && isShooterReadyStable()) startFeed(SeqState.SHOT4_FEED);
                break;

            case SHOT4_FEED:
                if (seqTimer.milliseconds() >= FEED_MS) startDecompress(SeqState.SHOT4_DECOMP);
                break;

            case SHOT4_DECOMP:
                if (seqTimer.milliseconds() >= DECOMPRESS_MS) startRecover(SeqState.SHOT4_RECOVER);
                break;

            case SHOT4_RECOVER:
                if (seqTimer.milliseconds() >= MIN_RECOVER_MS && isShooterReadyStable()) {
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
        } else {
            runTurretAiming(ll);
        }

        // ===================== TELEMETRY =====================
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, 0, 0, F));
        double vShooter = shooter.getVelocity();

        boolean readyNow = isShooterReadyStable();

        telemetry.addLine("=== One-Pad Master TeleOp ===");
        telemetry.addData("Alliance (init)", allianceIsRed ? "RED" : "BLUE");

        telemetry.addLine("--- Drive ---");
        telemetry.addData("Heading(deg)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        telemetry.addLine("--- Intake/Shooter ---");
        telemetry.addData("Intake1Enabled(A)", intake1Enabled);
        telemetry.addData("ReverseEject(dpad_down)", reverseEjectEnabled);
        telemetry.addData("ShooterEnabled(Y)", shooterEnabled);
        telemetry.addData("HaveLastShotSolution", haveLastShotSolution);
        telemetry.addData("SawTagThisFrame", sawTagThisFrame);
        telemetry.addData("RPM tgt (held)", "%.0f", lastRpmTgtCmd);
        telemetry.addData("ShooterVel", "%.0f", vShooter);
        telemetry.addData("READY (light)", readyNow);
        telemetry.addData("4Shot State", state);

        telemetry.addLine("--- Turret ---");
        telemetry.addData("AutoAim (RB)", autoAimEnabled);
        telemetry.addData("TurretCmd", "%.3f", lastTurretCmd);
        telemetry.addData("VisionTagSeen", turretTagSeen);
        telemetry.addData("tx(deg)", Double.isNaN(txDeg) ? "N/A" : String.format("%.2f", txDeg));

        telemetry.update();

        // ===================== Save prev =====================
        prevA=a; prevB=b; prevX=xBtn; prevY=yBtn;
        prevDpadDown = dpadDown;
        prevRB = rb;
    }

    // =========================================================
    // ShotAssist: update from Limelight BUT HOLD last solution
    // =========================================================
    private void updateFromLimelightAndComputeShot_HoldLast(LLResult result) {
        boolean gotNewSolution = false;
        sawTagThisFrame = false;

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            if (tags != null && !tags.isEmpty()) {
                LLResultTypes.FiducialResult chosen = null;
                for (LLResultTypes.FiducialResult t : tags) {
                    int id = (int) t.getFiducialId();
                    if (id == BLUE_GOAL_ID || id == RED_GOAL_ID) {
                        chosen = t;
                        break;
                    }
                }
                if (chosen == null) chosen = tags.get(0);

                Pose3D pose = null;
                try { pose = chosen.getCameraPoseTargetSpace(); } catch (Exception ignored) {}

                if (pose != null) {
                    sawTagThisFrame = true;

                    Position p = pose.getPosition();

                    double xMm = Math.abs(DistanceUnit.MM.fromUnit(p.unit, p.x));
                    double zMm = Math.abs(DistanceUnit.MM.fromUnit(p.unit, p.z));

                    double cameraToFlywheelMm = CAMERA_TO_FLYWHEEL_IN * 25.4;
                    double shooterHorizMm = Math.hypot(xMm, zMm + cameraToFlywheelMm);
                    double shooterHorizIn = shooterHorizMm * IN_PER_MM;

                    shooterDistIn = shooterHorizIn;

                    double newHood   = interp(DIST_IN, HOOD_POS, shooterDistIn);
                    double newMin    = interp(DIST_IN, RPM_MIN,  shooterDistIn);
                    double newTgt    = interp(DIST_IN, RPM_TGT,  shooterDistIn);
                    double newMax    = interp(DIST_IN, RPM_MAX,  shooterDistIn);

                    hoodCmd = newHood;
                    rpmMinCmd = newMin;
                    rpmTgtCmd = newTgt;
                    rpmMaxCmd = newMax;

                    lastHoodCmd = newHood;
                    lastRpmMinCmd = newMin;
                    lastRpmTgtCmd = newTgt;
                    lastRpmMaxCmd = newMax;

                    haveLastShotSolution = true;
                    gotNewSolution = true;

                    if (teleopStarted) hood.setPosition(lastHoodCmd);
                }
            }
        }

        if (!gotNewSolution && haveLastShotSolution) {
            hoodCmd = lastHoodCmd;
            rpmMinCmd = lastRpmMinCmd;
            rpmTgtCmd = lastRpmTgtCmd;
            rpmMaxCmd = lastRpmMaxCmd;

            if (teleopStarted) hood.setPosition(lastHoodCmd);
        }

        if (!haveLastShotSolution) {
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
        resetReadyStable();
    }

    private void startDecompress(SeqState decompState) {
        if (decompState == SeqState.SHOT1_DECOMP) intake1BurstOverride = true;

        if (decompState == SeqState.SHOT3_DECOMP || decompState == SeqState.SHOT4_DECOMP) {
            flipper.setPosition(FLIP_DOWN);
        }

        intake2.setPower(DECOMPRESS_PWR_INTAKE2);
        flicker.setPower(0);
        seqTimer.reset();
        state = decompState;
        resetReadyStable();
    }

    private void startRecover(SeqState recoverState) {
        flipper.setPosition(FLIP_DOWN);
        intake2.setPower(HOLD_PWR_INTAKE2);
        flicker.setPower(HOLD_PWR_FLICKER);
        seqTimer.reset();
        state = recoverState;
        resetReadyStable();
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

    private void resetReadyStable() { readyStableTimer.reset(); }

    private void stopShooter() { shooter.setPower(0); }

    // =========================================================
    // Turret aiming loop
    // =========================================================
    private void runTurretAiming(LLResult ll) {
        double dt = turretLoopTimer.seconds();
        turretLoopTimer.reset();
        dt = clamp(dt, 0.005, 0.050);

        odo.update();

        double rawHeadingDeg = getRawHeadingDeg();
        maybeLockHeadingOffset(rawHeadingDeg);
        double headingDegFTC = wrapDeg180(rawHeadingDeg + headingOffsetDeg);

        updateVisionTrim(ll);

        if (!autoAimEnabled) {
            setTurretCmd(lastTurretCmd);
            return;
        }

        double robotX_pin = odo.getPosX(DistanceUnit.INCH);
        double robotY_pin = odo.getPosY(DistanceUnit.INCH);

        double robotX = pinpointXToFtc(robotX_pin);
        double robotY = pinpointYToFtc(robotY_pin);

        double goalX = allianceIsRed ? RED_GOAL_X : BLUE_GOAL_X;
        double goalY = allianceIsRed ? RED_GOAL_Y : BLUE_GOAL_Y;

        double headingRad = Math.toRadians(headingDegFTC);

        double fwdX = Math.cos(headingRad);
        double fwdY = Math.sin(headingRad);

        double leftX = -Math.sin(headingRad);
        double leftY =  Math.cos(headingRad);

        double turretX = robotX + fwdX * TURRET_FWD_OFFSET_IN + leftX * TURRET_LEFT_OFFSET_IN;
        double turretY = robotY + fwdY * TURRET_FWD_OFFSET_IN + leftY * TURRET_LEFT_OFFSET_IN;

        double bearingDegField = Math.toDegrees(Math.atan2(goalY - turretY, goalX - turretX));
        bearingDegField = wrapDeg180(bearingDegField);

        double relDegCCW = wrapDeg180(bearingDegField - headingDegFTC);
        double turretDegCW_odo = -relDegCCW;

        double turretDegCW_total = turretDegCW_odo;
        if (visionTrimEnabled) turretDegCW_total = wrapDeg180(turretDegCW_odo + visionTrimDegCW);

        double currentTurretDegCW = (lastTurretCmd - TURRET_HOME) / POS_PER_DEG_CW;
        double errDegCW = wrapDeg180(turretDegCW_total - currentTurretDegCW);
        if (Math.abs(errDegCW) < BEARING_DEADBAND_DEG) errDegCW = 0.0;

        double deltaServo = errDegCW * KP_DEG_TO_SERVO;
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

        if (result == null || !result.isValid()) { handleVisionLost(); return; }

        txDeg = result.getTx();

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) { handleVisionLost(); return; }

        LLResultTypes.FiducialResult chosen = null;
        for (LLResultTypes.FiducialResult t : tags) {
            int id = (int) t.getFiducialId();
            if (id == BLUE_GOAL_ID || id == RED_GOAL_ID) { chosen = t; break; }
        }
        if (chosen == null) chosen = tags.get(0);

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

    private double getRawHeadingDeg() {
        return wrapDeg180(Math.toDegrees(odo.getHeading(UnnormalizedAngleUnit.RADIANS)));
    }

    private void maybeLockHeadingOffset(double rawDegNow) {
        if (headingOffsetLocked) return;

        if (imuSettleTimer.milliseconds() < IMU_MIN_SETTLE_MS) {
            haveLastRaw = false;
            stableCount = 0;
            return;
        }

        if (!haveLastRaw) {
            lastRawForStability = rawDegNow;
            haveLastRaw = true;
            stableCount = 0;
            return;
        }

        double delta = wrapDeg180(rawDegNow - lastRawForStability);
        lastRawForStability = rawDegNow;

        if (Math.abs(delta) <= IMU_STABLE_EPS_DEG) stableCount++;
        else stableCount = 0;

        if (stableCount >= IMU_STABLE_LOOPS) {
            headingOffsetDeg = wrapDeg180(seededHeadingDeg - rawDegNow);
            headingOffsetLocked = true;
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

        if (statuslight != null) statuslight.setPosition(LIGHT_OFF);
    }
}
