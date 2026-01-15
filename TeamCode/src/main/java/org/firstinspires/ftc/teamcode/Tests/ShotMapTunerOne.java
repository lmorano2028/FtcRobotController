package org.firstinspires.ftc.teamcode.Tests;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "ShotMapTunerOringinal", group = "Test")
public class ShotMapTunerOne extends OpMode {


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
    private double shooterTargetVel = 1450;         // <- tunable live
    private double shooterReadyMin  = 1410;         // window tracks target
    private double shooterReadyMax  = 1480;


    private static final int READY_STABLE_MS = 110;
    private static final int MIN_RECOVER_MS = 120;


    // Tuning increments for shooter target (ticks/sec)
    private static final double SHOOTER_STEP = 10.0;  // tap triggers to change
    private static final double SHOOTER_MIN  = 900.0;
    private static final double SHOOTER_MAX  = 2000.0;


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
    private double hoodPos = 0.22;     // start SAFE-ish
    private double hoodStep = 0.005;


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
        hood    = hardwareMap.get(Servo.class, HOOD_SERVO_NAME);


        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


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


        // ===== Abort =====
        if (bPressed) {
            abortSequence("User pressed B");
        }


        // ===== Hood step size adjust =====
        boolean lb = gamepad1.left_bumper;
        boolean rb = gamepad1.right_bumper;
        boolean lbPressed = lb && !prevLB;
        boolean rbPressed = rb && !prevRB;


        if (lbPressed) hoodStep = Math.max(0.001, hoodStep / 2.0);
        if (rbPressed) hoodStep = Math.min(0.05,  hoodStep * 2.0);
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
        boolean dLeft = gamepad1.dpad_left;
        boolean dRight = gamepad1.dpad_right;
        boolean dLeftPressed = dLeft && !prevDLeft;
        boolean dRightPressed = dRight && !prevDRight;
        prevDLeft = dLeft; prevDRight = dRight;


        boolean ls = gamepad1.left_stick_button;   // SAFE
        boolean rs = gamepad1.right_stick_button;  // FAR
        boolean lsPressed = ls && !prevLS;
        boolean rsPressed = rs && !prevRS;
        prevLS = ls; prevRS = rs;


        if (lsPressed) hoodPos = HOOD_PRESET_SAFE;
        if (dLeftPressed) hoodPos = HOOD_PRESET_CLOSE;
        if (dRightPressed) hoodPos = HOOD_PRESET_MID;
        if (rsPressed) hoodPos = HOOD_PRESET_FAR;


        // Clamp hood to your measured safe range
        hoodPos = clamp(hoodPos, HOOD_LOW_LIMIT, HOOD_HIGH_LIMIT);
        hood.setPosition(hoodPos);


        // ===== Shooter target tuning (tap triggers) =====
        // treat "tap" as trigger past 0.6
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
                // do nothing until driver toggles shooter back on, or you restart opmode
                break;
        }


        // ===== Telemetry =====
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
            intake1BurstOverride = true; // after shot1, run intake1 full
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
        // Keep a +/- window around the current target. You can widen/tighten later.
        double halfWindow = 15.0; // ticks/sec
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
    private void abortSequence(String reason) {
        stopSequenceMotors();
        intake1.setPower(0);
        stopShooter();
        shooterEnabled = false;
        resetReadyStable();
        state = SeqState.ABORTED;
        intake1BurstOverride = false;
    }


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

