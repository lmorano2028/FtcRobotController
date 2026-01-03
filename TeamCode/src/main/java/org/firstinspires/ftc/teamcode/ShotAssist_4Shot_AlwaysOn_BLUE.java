package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

/**
 * ShotAssist_4Shot (Always-On Limelight)
 *
 * Controls:
 *  A = Toggle intakeOneMotor
 *  Y = Toggle shooter ON/OFF (velocity mode)
 *  X = Fire 4-shot sequence (ONLY if tag seen + shooter READY stable)
 *  B = ABORT (stop shooter + intake2 + flicker; intake1 remains under A toggle)
 *
 * NEW (Turret Auto-Aim):
 *  - Uses tx(deg) to command turret servos LRotation + RRotation toward tag
 *  - Clamped to SAFE_MIN..SAFE_MAX so turret cannot over-rotate
 *  - BACK button = re-home turret to 0.50 instantly (and holds it until you release)
 */
@TeleOp(name="ShotAssist_4Shot_AlwaysOn_BLUE", group="Test")
public class ShotAssist_4Shot_AlwaysOn_BLUE extends OpMode {

    // ======= CONFIG NAMES =======
    private static final String LIMELIGHT_NAME = "limelight";
    private static final String SHOOTER_NAME   = "ShooterMotor";
    private static final String FLICKER_NAME   = "Outertake";
    private static final String INTAKE1_NAME   = "intakeOneMotor";
    private static final String INTAKE2_NAME   = "intakeTwoMotor";
    private static final String HOOD_NAME      = "Shooter hood";
    private static final String FLIPPER_NAME   = "fingler";

    // ======= NEW: TURRET SERVO NAMES =======
    private static final String TURRET_L_NAME  = "LRotation";
    private static final String TURRET_R_NAME  = "RRotation";

    // ======= NEW: TURRET LIMITS / HOME =======
    private static final double TURRET_HOME = 0.50;
    private static final double TURRET_SAFE_MIN = 0.10;
    private static final double TURRET_SAFE_MAX = 0.90;

    // Convert tx(deg) -> servo position offset (tune this)
    // Example: tx=10deg => 0.06 pos shift if KP=0.006
    private static final double TURRET_KP_POS_PER_DEG = 0.006;

    // Small deadband so tiny tx noise doesn't jitter the turret
    private static final double TURRET_TX_DEADBAND_DEG = 0.40;

    // If turret moves opposite direction, flip this.
    private static final boolean TURRET_INVERT = false;

    // Optional: keep turret centered when no tag (false = hold last command)
    private static final boolean TURRET_RETURN_HOME_WHEN_NO_TAG = false;

    // ======= LIMELIGHT / PIPELINE =======
    private static final int APRILTAG_PIPELINE_INDEX = 0;
    private static final int BLUE_GOAL_ID = 20;
    private static final int RED_GOAL_ID  = 24;

    // ======= TEAM SELECTION (BLUE ONLY) =======
    private static final int ACTIVE_GOAL_ID = BLUE_GOAL_ID;

    // ======= DISTANCE OFFSETS =======
    private static final double CAMERA_TO_FLYWHEEL_IN = 8.0; // camera is 8" in FRONT of flywheel exit
    private static final double IN_PER_MM = 1.0 / 25.4;

    // ======= SHOT MAP TABLE (YOUR VALUES) =======
    // ShooterDist_in (from flywheel exit to depot face)
    private static final double[] DIST_IN = { 24, 48, 80, 120 };
    private static final double[] HOOD_POS = { 0.350, 0.525, 0.790, 0.790 };
    private static final double[] RPM_MIN  = {  970, 1070, 1150, 1400 };
    private static final double[] RPM_TGT  = { 1000, 1100, 1180, 1440 };
    private static final double[] RPM_MAX  = { 1030, 1130, 1210, 1470 };

    // ======= HARDWARE =======
    private Limelight3A limelight;
    private DcMotorEx shooter;
    private DcMotorEx flicker;
    private DcMotorEx intake1;
    private DcMotorEx intake2;
    private Servo hood;
    private Servo flipper;

    // ======= NEW: TURRET SERVOS =======
    private Servo turretL;
    private Servo turretR;
    private double turretCmd = TURRET_HOME;

    // ======= FLIPPER POSITIONS (your tuned values) =======
    private static final double FLIP_DOWN = 0.15;
    private static final double FLIP_UP   = 0.00;

    // ======= INTAKES =======
    private static final double INTAKE1_PWR = 0.8;
    private static final double INTAKE1_BURST_PWR = 1.0;

    // ======= FEED / HOLD / DECOMPRESS =======
    private static final double HOLD_PWR_INTAKE2 = 0.10;
    private static final double HOLD_PWR_FLICKER = 0.20;

    private static final double FEED_PWR_INTAKE2 = 1.0;
    private static final double FEED_PWR_FLICKER = 1.0;
    private static final int FEED_MS = 200;

    private static final double DECOMPRESS_PWR_INTAKE2 = -0.35;
    private static final int DECOMPRESS_MS = 200;

    // ======= READY STABLE =======
    private static final int READY_STABLE_MS = 110;
    private static final int MIN_RECOVER_MS  = 120;

    // ======= LATCHES =======
    private boolean intake1Enabled = false;
    private boolean shooterEnabled = false;
    private boolean intake1BurstOverride = false;

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
    private boolean prevBack=false; // NEW for re-home

    // ======= “LIVE” SHOT ASSIST OUTPUTS =======
    private boolean tagSeen = false;
    private int tagId = -1;
    private double tx = Double.NaN;
    private double ty = Double.NaN;

    private double camZmm = Double.NaN;
    private double camZin = Double.NaN;

    // NEW: components + range telemetry (does not change behavior besides distance math)
    private double camXmm = Double.NaN;
    private double camXin = Double.NaN;
    private double camHorizMm = Double.NaN;
    private double camHorizIn = Double.NaN;
    private double shooterHorizMm = Double.NaN;
    private double shooterHorizIn = Double.NaN;

    private double shooterDistIn = Double.NaN;
    // P and F values
    double F = 16.53;
    double P = 265;

    private double hoodCmd = 0.22;         // safe-ish default
    private double rpmMinCmd = 0;
    private double rpmTgtCmd = 0;
    private double rpmMaxCmd = 0;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);

        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);
        flicker = hardwareMap.get(DcMotorEx.class, FLICKER_NAME);
        intake1 = hardwareMap.get(DcMotorEx.class, INTAKE1_NAME);
        intake2 = hardwareMap.get(DcMotorEx.class, INTAKE2_NAME);
        hood    = hardwareMap.get(Servo.class, HOOD_NAME);
        flipper = hardwareMap.get(Servo.class, FLIPPER_NAME);

        // NEW: Turret servos
        turretL = hardwareMap.get(Servo.class, TURRET_L_NAME);
        turretR = hardwareMap.get(Servo.class, TURRET_R_NAME);
        turretCmd = TURRET_HOME;
        applyTurret(turretCmd);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        flicker.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake1.setDirection(DcMotor.Direction.REVERSE);

        flicker.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // safety defaults
        hood.setPosition(hoodCmd);
        flipper.setPosition(FLIP_DOWN);

        stopSequenceMotors();
        stopShooter();

        // Limelight
        limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
        limelight.start();

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("ShotAssist_4Shot_AlwaysOn READY");
        telemetry.addLine("A intake1 toggle | Y shooter toggle | X 4-shot | B abort");
        telemetry.addLine("BLUE ONLY: will ONLY accept TagID " + ACTIVE_GOAL_ID);
        telemetry.addLine("Limelight ALWAYS updates Hood/RPM. If NO TAG -> shooter forced OFF & X disabled.");
        telemetry.addLine("Pipeline: enable Full 3D for AprilTags.");
        telemetry.addLine("NEW: Turret auto-aim on tx with clamps ["+TURRET_SAFE_MIN+".."+TURRET_SAFE_MAX+"], HOME=0.50");
        telemetry.addLine("NEW: BACK = Re-home turret to 0.50");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ===== Read buttons =====
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;
        boolean back = gamepad1.back; // NEW

        boolean aPressed = a && !prevA;
        boolean bPressed = b && !prevB;
        boolean xPressed = x && !prevX;
        boolean yPressed = y && !prevY;
        boolean backPressed = back && !prevBack; // NEW

        // ===== ALWAYS update shot assist from Limelight =====
        updateFromLimelightAndComputeShot();

        // ===== NEW: Turret auto-aim (runs every loop) =====
        // BACK button instantly re-homes turret to 0.50 (you can press anytime)
        if (backPressed) {
            turretCmd = TURRET_HOME;
            applyTurret(turretCmd);
        } else {
            updateTurretAutoAim();
        }

        // ===== Intake1 toggle =====
        if (aPressed) {
            intake1Enabled = !intake1Enabled;
        }
        if (intake1BurstOverride) {
            intake1.setPower(INTAKE1_BURST_PWR);
        } else {
            intake1.setPower(intake1Enabled ? INTAKE1_PWR : 0.0);
        }

        // ===== Shooter toggle =====
        if (yPressed) {
            shooterEnabled = !shooterEnabled;
            resetReadyStable();
            if (!shooterEnabled) stopShooter();
        }

        // Shooter command rule:
        // - If shooterEnabled AND tagSeen AND we have a valid rpm target -> run velocity
        // - Else -> FORCE shooter off
        if (shooterEnabled && tagSeen && rpmTgtCmd > 0) {
            shooter.setVelocity(rpmTgtCmd);
        } else {
            stopShooter();
        }

        // ===== Abort =====
        if (bPressed) {
            abortSequence("User pressed B");
        }

        // ===== Start 4-shot sequence =====
        if (xPressed && state == SeqState.IDLE) {
            // Hard gate: must have tag, shooter toggled on, and ready stable
            if (shooterEnabled && tagSeen && isShooterReadyStable()) {
                startFeed(SeqState.SHOT1_FEED);
            }
        }

        // ===== Run 4-shot state machine =====
        switch (state) {
            case IDLE:
                intake1BurstOverride = false;
                flipper.setPosition(FLIP_DOWN);
                break;

            case SHOT1_FEED:
                if (seqTimer.milliseconds() >= FEED_MS) startDecompress(SeqState.SHOT1_DECOMP);
                break;

            case SHOT1_DECOMP:
                if (seqTimer.milliseconds() >= DECOMPRESS_MS) startRecover(SeqState.SHOT1_RECOVER);
                break;

            case SHOT1_RECOVER:
                if (seqTimer.milliseconds() >= MIN_RECOVER_MS && isShooterReadyStable()) startFeed(SeqState.SHOT2_FEED);
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

        // ===== Telemetry =====
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        double vShooter = shooter.getVelocity();

        telemetry.addLine("=== ShotAssist_4Shot (Always-On) ===");
        telemetry.addData("Pipeline", APRILTAG_PIPELINE_INDEX);

        telemetry.addData("TagSeen", tagSeen);
        telemetry.addData("TagID", tagId);
        telemetry.addData("tx(deg)", Double.isNaN(tx) ? "N/A" : String.format("%.2f", tx));
        telemetry.addData("ty(deg)", Double.isNaN(ty) ? "N/A" : String.format("%.2f", ty));

        telemetry.addData("CamX (mm)", Double.isNaN(camXmm) ? "N/A" : String.format("%.1f", camXmm));
        telemetry.addData("CamZ (mm)", Double.isNaN(camZmm) ? "N/A" : String.format("%.1f", camZmm));
        telemetry.addData("CamX (in)", Double.isNaN(camXin) ? "N/A" : String.format("%.2f", camXin));
        telemetry.addData("CamZ (in)", Double.isNaN(camZin) ? "N/A" : String.format("%.2f", camZin));

        telemetry.addData("CamHoriz (in)", Double.isNaN(camHorizIn) ? "N/A" : String.format("%.2f", camHorizIn));
        telemetry.addData("ShooterHoriz (in)", Double.isNaN(shooterHorizIn) ? "N/A" : String.format("%.2f", shooterHorizIn));

        telemetry.addData("ShooterDist (in)", Double.isNaN(shooterDistIn) ? "N/A" : String.format("%.2f", shooterDistIn));

        telemetry.addData("HoodCmd", "%.3f", hoodCmd);
        telemetry.addData("RPM(min/tgt/max)", "%.0f / %.0f / %.0f", rpmMinCmd, rpmTgtCmd, rpmMaxCmd);
        telemetry.addLine("------------------------------");

        telemetry.addData("ShooterToggle(Y)", shooterEnabled);
        telemetry.addData("ShooterVel", "%.0f", vShooter);
        telemetry.addData("ReadyStable(ms)", "%.0f / %d", readyStableTimer.milliseconds(), READY_STABLE_MS);
        telemetry.addData("READY?", isShooterReadyStable());
        telemetry.addLine("------------------------------");

        telemetry.addData("Tuning P","%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F","%.4f (D-Pad L/R)", F);
        telemetry.addLine("------------------------------");

        telemetry.addData("Intake1Toggle(A)", intake1Enabled);
        telemetry.addData("Intake1BurstOverride", intake1BurstOverride);
        telemetry.addData("State", state);
        telemetry.addData("SeqTimer(ms)", "%.0f", seqTimer.milliseconds());
        telemetry.addData("FlipperPos", "%.2f", flipper.getPosition());

        // NEW turret telemetry
        telemetry.addLine("------------------------------");
        telemetry.addData("TurretCmd", "%.3f", turretCmd);
        telemetry.addData("TurretSafe[min..max]", "%.2f .. %.2f", TURRET_SAFE_MIN, TURRET_SAFE_MAX);
        telemetry.addData("TurretHome", "%.2f", TURRET_HOME);
        telemetry.addLine("BACK = Re-home turret to 0.50");

        if (!tagSeen) telemetry.addLine("NOTE: No tag -> shooter forced OFF and X disabled.");
        telemetry.update();

        prevA=a; prevB=b; prevX=x; prevY=y;
        prevBack=back;
    }

    // ===================== LIMELIGHT + SHOT ASSIST =====================

    private void updateFromLimelightAndComputeShot() {
        tagSeen = false;
        tagId = -1;
        tx = Double.NaN;
        ty = Double.NaN;
        camZmm = Double.NaN;
        camZin = Double.NaN;

        camXmm = Double.NaN;
        camXin = Double.NaN;
        camHorizMm = Double.NaN;
        camHorizIn = Double.NaN;
        shooterHorizMm = Double.NaN;
        shooterHorizIn = Double.NaN;

        shooterDistIn = Double.NaN;

        // Defaults if no tag
        hoodCmd = HOOD_POS[0];
        rpmMinCmd = 0;
        rpmTgtCmd = 0;
        rpmMaxCmd = 0;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        tx = result.getTx();
        ty = result.getTy();

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return;

        // BLUE ONLY: choose TagID 20 ONLY. Ignore everything else.
        LLResultTypes.FiducialResult chosen = null;
        for (LLResultTypes.FiducialResult t : tags) {
            int id = (int) t.getFiducialId();
            if (id == ACTIVE_GOAL_ID) {
                chosen = t;
                break;
            }
        }
        if (chosen == null) {
            tagSeen = false;
            return;
        }

        tagSeen = true;
        tagId = (int) chosen.getFiducialId();

        Pose3D pose = null;
        try {
            pose = chosen.getCameraPoseTargetSpace();
        } catch (Exception ignored) {}

        if (pose == null) {
            tagSeen = false;
            return;
        }

        Position p = pose.getPosition();

        double xMm = Math.abs(DistanceUnit.MM.fromUnit(p.unit, p.x));
        double zMm = Math.abs(DistanceUnit.MM.fromUnit(p.unit, p.z));

        camXmm = xMm;
        camZmm = zMm;

        camXin = camXmm * IN_PER_MM;
        camZin = camZmm * IN_PER_MM;

        camHorizMm = Math.hypot(camXmm, camZmm);
        camHorizIn = camHorizMm * IN_PER_MM;

        double cameraToFlywheelMm = CAMERA_TO_FLYWHEEL_IN * 25.4;
        shooterHorizMm = Math.hypot(camXmm, camZmm + cameraToFlywheelMm);
        shooterHorizIn = shooterHorizMm * IN_PER_MM;

        shooterDistIn = shooterHorizIn;
        if (shooterDistIn < 0) shooterDistIn = 0;

        hoodCmd   = interp(DIST_IN, HOOD_POS, shooterDistIn);
        rpmMinCmd = interp(DIST_IN, RPM_MIN,  shooterDistIn);
        rpmTgtCmd = interp(DIST_IN, RPM_TGT,  shooterDistIn);
        rpmMaxCmd = interp(DIST_IN, RPM_MAX,  shooterDistIn);

        hood.setPosition(hoodCmd);
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

    // ===================== NEW: TURRET AUTO AIM =====================

    private void updateTurretAutoAim() {
        if (!tagSeen || Double.isNaN(tx)) {
            if (TURRET_RETURN_HOME_WHEN_NO_TAG) {
                turretCmd = TURRET_HOME;
                applyTurret(turretCmd);
            }
            return; // hold last turretCmd otherwise
        }

        double err = tx;

        // deadband to reduce jitter
        if (Math.abs(err) < TURRET_TX_DEADBAND_DEG) err = 0.0;

        // Convert tx (deg) -> desired absolute servo position around HOME
        // If your turret moves the wrong direction, flip TURRET_INVERT.
        double sign = TURRET_INVERT ? +1.0 : -1.0;
        double desired = TURRET_HOME + sign * (err * TURRET_KP_POS_PER_DEG);

        // Clamp to safe range: this is your "software hard stop"
        desired = clamp(desired, TURRET_SAFE_MIN, TURRET_SAFE_MAX);

        turretCmd = desired;
        applyTurret(turretCmd);
    }

    private void applyTurret(double pos) {
        // You said mirror right = false worked, so both get same position.
        turretL.setPosition(pos);
        turretR.setPosition(pos);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // ===================== SEQUENCE HELPERS (WITH FINGLER) =====================

    private void startFeed(SeqState feedState) {
        // Fingler UP only for Shot3 + Shot4 feeds (immediately)
        if (feedState == SeqState.SHOT3_FEED || feedState == SeqState.SHOT4_FEED) {
            flipper.setPosition(FLIP_UP);
        } else {
            flipper.setPosition(FLIP_DOWN);
        }

        intake2.setPower(FEED_PWR_INTAKE2);
        flicker.setPower(FEED_PWR_FLICKER);
        seqTimer.reset();
        state = feedState;
        resetReadyStable();
    }

    private void startDecompress(SeqState decompState) {
        // After shot #1 is taken, force intake1 full power until sequence ends
        if (decompState == SeqState.SHOT1_DECOMP) {
            intake1BurstOverride = true;
        }

        // Fingler DOWN at start of Shot3/Shot4 DECOMP
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

    // ===================== READY / ABORT =====================

    private boolean isShooterReadyStable() {
        if (!shooterEnabled || !tagSeen || rpmTgtCmd <= 0) {
            resetReadyStable();
            return false;
        }

        double v = shooter.getVelocity();
        boolean inWindow = (v >= rpmMinCmd && v <= rpmMaxCmd);

        if (inWindow) {
            return readyStableTimer.milliseconds() >= READY_STABLE_MS;
        } else {
            resetReadyStable();
            return false;
        }
    }

    private void resetReadyStable() {
        readyStableTimer.reset();
    }

    private void stopShooter() {
        shooter.setPower(0);
    }

    private void abortSequence(String reason) {
        stopSequenceMotors();
        stopShooter();
        shooterEnabled = false;
        intake1BurstOverride = false;
        resetReadyStable();
        state = SeqState.ABORTED;
    }

    @Override
    public void stop() {
        intake1.setPower(0);
        stopSequenceMotors();
        stopShooter();
        limelight.stop();
    }
}

