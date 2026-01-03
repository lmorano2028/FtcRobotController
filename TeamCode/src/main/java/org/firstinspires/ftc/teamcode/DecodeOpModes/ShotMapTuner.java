package org.firstinspires.ftc.teamcode.DecodeOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "ShotMapTuner", group = "Test")
public class ShotMapTuner extends OpMode {

    // ======= CONFIG NAMES =======
    private static final String SHOOTER_NAME = "ShooterMotor";
    private static final String FLICKER_NAME = "Outertake";
    private static final String INTAKE1_NAME = "intakeOneMotor";
    private static final String INTAKE2_NAME = "intakeTwoMotor";
    private static final String HOOD_SERVO_NAME = "Shooter hood";

    // ======= HARDWARE =======
    private DcMotorEx shooter;
    private DcMotorEx flicker;
    private DcMotorEx intake1;
    private DcMotorEx intake2;
    private Servo hood;

    // ======= SHOOTER TARGETS (ticks/sec) =======
    private double shooterTargetVel = 1440;     	// <- tunable live
    private double shooterReadyMin  = 1400;     	// window tracks target
    private double shooterReadyMax  = 1470;

    private static final int READY_STABLE_MS = 110;
    private static final int MIN_RECOVER_MS = 120;

    // Tuning increments for shooter target (ticks/sec)
    private static final double SHOOTER_STEP = 10.0;  // tap triggers to change
    private static final double SHOOTER_MIN  = 900.0;
    private static final double SHOOTER_MAX  = 2000.0;

    // ===== Fix #2: Shooter Velocity PIDF tuning =====
    // We will mostly change kF to reduce overshoot.

    private static final double KF_STEP = 0.5;  // how much kF changes per press

    // ======= FEED / HOLD / DECOMPRESS =======
    private static final double HOLD_PWR_INTAKE2 = 0.10;
    private static final double HOLD_PWR_FLICKER = 0.20;

    private static final double FEED_PWR_INTAKE2 = 1.0;
    private static final double FEED_PWR_FLICKER = 1.0;
    private static final int FEED_MS = 200;

    private static final double DECOMPRESS_PWR_INTAKE2 = -0.35;
    private static final int DECOMPRESS_MS = 200;

    // ======= INTAKE 1 =======
    private static final double INTAKE1_PWR = 0.8;

    // ======= Intake1 burst override =======
    private static final double INTAKE1_BURST_PWR = 1.0;
    private boolean intake1BurstOverride = false;

    // ======= LATCHES =======
    private boolean intake1Enabled = false; // A toggles this
    private boolean shooterEnabled = false; // Y toggles this

    // ======= HOOD TUNING =======
    private double hoodPos = 0.22; 	// start SAFE-ish
    private double hoodStep = 0.005;

    double F = 16.53;
    double P = 265;

    // Your measured values
    private static final double HOOD_LOW_LIMIT  = 0.20;
    private static final double HOOD_HIGH_LIMIT = 0.85;

    private static final double HOOD_PRESET_SAFE  = 0.22;
    private static final double HOOD_PRESET_CLOSE = 0.25;
    private static final double HOOD_PRESET_MID   = 0.525;
    private static final double HOOD_PRESET_FAR   = 0.790;

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
    private boolean prevDUp=false, prevDDown=false, prevDLeft=false, prevDRight=false;
    private boolean prevLB=false, prevRB=false;
    private boolean prevLS=false, prevRS=false;
    private boolean prevRTTap=false, prevLTTap=false;


    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);
        flicker = hardwareMap.get(DcMotorEx.class, FLICKER_NAME);
        intake1 = hardwareMap.get(DcMotorEx.class, INTAKE1_NAME);
        intake2 = hardwareMap.get(DcMotorEx.class, INTAKE2_NAME);
        hood	= hardwareMap.get(Servo.class, HOOD_SERVO_NAME);

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

        stopSequenceMotors();
        stopShooter();

        // Hood start
        hoodPos = HOOD_PRESET_SAFE;
        hood.setPosition(hoodPos);

        // Initialize ready window around target
        updateReadyWindow();

        telemetry.addLine("ShotMapTuner READY");
        telemetry.addLine("Shooter: Y toggle | X fire 4-shot | B abort | A toggle intake1");
        telemetry.addLine("Hood: Dpad Up/Down bump | LB/RB step | LS=SAFE, DpadL=CLOSE, DpadR=MID, RS=FAR");
        telemetry.addLine("RPM: Tap RT=+  Tap LT=-");
        telemetry.addLine("Fix#2 (ONE GAMEPAD): HOLD LB + DpadL/R changes kF, HOLD LB + A applies");
        telemetry.addLine("Init complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ===== Read gamepad =====
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;

        boolean aPressed = a && !prevA;
        boolean bPressed = b && !prevB;
        boolean xPressed = x && !prevX;
        boolean yPressed = y && !prevY;

        // ===== Intake1 toggle (independent) =====
        if (aPressed) {
            intake1Enabled = !intake1Enabled;
        }

        // intake1 power decision (burst override wins)
        if (intake1BurstOverride) {
            intake1.setPower(INTAKE1_BURST_PWR);
        } else {
            intake1.setPower(intake1Enabled ? INTAKE1_PWR : 0);
        }

        // ===== Shooter toggle (independent) =====
        if (yPressed) {
            shooterEnabled = !shooterEnabled;
            if (!shooterEnabled) {
                stopShooter();
                resetReadyStable();
            } else {
                shooter.setVelocity(shooterTargetVel);
                resetReadyStable();
            }
        }
        if (shooterEnabled) {
            shooter.setVelocity(shooterTargetVel);
        }


        // ===== Hood step size adjust OR kF tune mode (HOLD LB) =====
        boolean lb = gamepad1.left_bumper;
        boolean rb = gamepad1.right_bumper;
        boolean lbPressed = lb && !prevLB;
        boolean rbPressed = rb && !prevRB;


        prevLB = lb; prevRB = rb;

        // ===== Hood bump =====
        boolean dUp = gamepad1.dpad_up;
        boolean dDown = gamepad1.dpad_down;
        boolean dUpPressed = dUp && !prevDUp;
        boolean dDownPressed = dDown && !prevDDown;

        if (dUpPressed) hoodPos += hoodStep;
        if (dDownPressed) hoodPos -= hoodStep;
        prevDUp = dUp; prevDDown = dDown;

        // ===== Hood presets (avoid A/B/X/Y conflicts) =====

        // Normal hood presets
        if (gamepad1.dpadRightWasPressed()) {
            hood.setPosition(HOOD_PRESET_MID);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            hood.setPosition(HOOD_PRESET_CLOSE);
        }
        if (gamepad1.bWasPressed()) {
            hood.setPosition(HOOD_PRESET_FAR);
        }

        // Clamp hood to your measured safe range
        hoodPos = clamp(hoodPos, HOOD_LOW_LIMIT, HOOD_HIGH_LIMIT);
        hood.setPosition(hoodPos);

        // ===== Shooter target tuning (tap triggers) =====
        boolean rtTap = gamepad1.right_trigger > 0.6;
        boolean ltTap = gamepad1.left_trigger > 0.6;

        boolean rtPressed = rtTap && !prevRTTap;
        boolean ltPressed = ltTap && !prevLTTap;

        if (rtPressed) {
            shooterTargetVel = clamp(shooterTargetVel + SHOOTER_STEP, SHOOTER_MIN, SHOOTER_MAX);
            updateReadyWindow();
            resetReadyStable();
        }
        if (ltPressed) {
            shooterTargetVel = clamp(shooterTargetVel - SHOOTER_STEP, SHOOTER_MIN, SHOOTER_MAX);
            updateReadyWindow();
            resetReadyStable();
        }
        prevRTTap = rtTap; prevLTTap = ltTap;

        // Apply shooter velocity if enabled
        if (shooterEnabled) {
            shooter.setVelocity(shooterTargetVel);
        }

        // ===== Start 4-shot sequence =====
        if (xPressed && state == SeqState.IDLE) {
            if (shooterEnabled && isShooterReadyStable()) {
                startFeed(SeqState.SHOT1_FEED);
            }
        }

        // ===== Run sequence state machine =====
        switch (state) {
            case IDLE:
                intake1BurstOverride = false;
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
                    state = SeqState.DONE;
                    seqTimer.reset();
                }
                break;

            case DONE:
                stopSequenceMotors();
                intake1BurstOverride = false;
                state = SeqState.IDLE;
                break;

            case ABORTED:
                break;
        }

        // ===== Telemetry =====
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        double vShooter = shooter.getVelocity();

        telemetry.addLine("=== ShotMapTuner ===");
        telemetry.addData("Hood Pos", "%.3f", hoodPos);
        telemetry.addData("Hood Step", "%.4f (LB half / RB double)", hoodStep);
        telemetry.addData("Hood Presets", "SAFE(LS)=%.2f CLOSE(DL)=%.2f MID(DR)=%.3f FAR(RS)=%.2f",
                HOOD_PRESET_SAFE, HOOD_PRESET_CLOSE, HOOD_PRESET_MID, HOOD_PRESET_FAR);

        telemetry.addData("ShooterEnabled (Y)", shooterEnabled);
        telemetry.addData("Shooter TargetVel", "%.0f", shooterTargetVel);
        telemetry.addData("Shooter Vel", "%.0f", vShooter);
        telemetry.addData("ReadyWindow", "[%.0f..%.0f]", shooterReadyMin, shooterReadyMax);
        telemetry.addData("ReadyStable(ms)", "%.0f / %d", readyStableTimer.milliseconds(), READY_STABLE_MS);

        telemetry.addData("P Value","%.4f (D-Pad U/D)", P);
        telemetry.addData("F Value","%.4f (D-Pad L/R)", F);
        telemetry.addLine("Fix#2 (ONE GAMEPAD): HOLD LB + DpadL/R changes kF, HOLD LB + A applies");

        telemetry.addData("Intake1 Toggle (A)", intake1Enabled);
        telemetry.addData("Intake1 BurstOverride", intake1BurstOverride);

        telemetry.addData("State", state);
        telemetry.addData("SeqTimer(ms)", "%.0f", seqTimer.milliseconds());

        telemetry.addLine("Tune: Tap RT/LT to change target speed. DpadUp/Down to tweak hood.");
        telemetry.addLine("Shoot: Y ON -> wait READY -> X burst. Record: Distance + Hood Pos + TargetVel.");
        telemetry.update();

        prevA=a; prevB=b; prevX=x; prevY=y;
    }

    // ======== Sequence helpers (ONLY intake2 + flicker) ========
    private void startFeed(SeqState feedState) {
        intake2.setPower(FEED_PWR_INTAKE2);
        flicker.setPower(FEED_PWR_FLICKER);
        seqTimer.reset();
        state = feedState;
        resetReadyStable();
    }

    private void startDecompress(SeqState decompState) {
        if (decompState == SeqState.SHOT1_DECOMP) {
            intake1BurstOverride = true;
        }
        intake2.setPower(DECOMPRESS_PWR_INTAKE2);
        flicker.setPower(0);
        seqTimer.reset();
        state = decompState;
        resetReadyStable();
    }

    private void startRecover(SeqState recoverState) {
        intake2.setPower(HOLD_PWR_INTAKE2);
        flicker.setPower(HOLD_PWR_FLICKER);
        seqTimer.reset();
        state = recoverState;
        resetReadyStable();
    }

    private void stopSequenceMotors() {
        intake2.setPower(0);
        flicker.setPower(0);
    }

    // ======== Shooter ready helpers ========
    private boolean isShooterReadyStable() {
        if (!shooterEnabled) return false;

        double v = shooter.getVelocity();
        boolean inWindow = (v >= shooterReadyMin && v <= shooterReadyMax);

        if (inWindow) {
            return readyStableTimer.milliseconds() >= READY_STABLE_MS;
        } else {
            resetReadyStable();
            return false;
        }
    }

    private void updateReadyWindow() {
        double halfWindow = 25.0;
        shooterReadyMin = shooterTargetVel - halfWindow;
        shooterReadyMax = shooterTargetVel + halfWindow;
    }

    private void resetReadyStable() {
        readyStableTimer.reset();
    }

    private void stopShooter() {
        shooter.setPower(0);
    }

    // ======== Abort ========
    @Override
    public void stop() {
        intake1.setPower(0);
        stopSequenceMotors();
        stopShooter();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}

