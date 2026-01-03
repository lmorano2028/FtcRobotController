package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * FourShotTest_v2 -> UPDATED with Flipper (servo "fingler")
 *
 * Buttons:
 *  - A: Toggle Intake ONE (intakeOneMotor) ONLY. On/off latch.
 *  - Y: Toggle Shooter motor velocity control on/off. On/off latch.
 *  - X: Fire 4-shot sequence (ONLY if shooter is ON and READY at target velocity window).
 *  - B: ABORT (stop intake2 + flicker + shooter) (intakeOne follows its own toggle)
 *
 * Sequence controls:
 *  - intakeTwoMotor
 *  - Outertake (flicker)
 *  - fingler (flipper servo) ONLY during Shot3 FEED and Shot4 FEED
 *
 * Flipper behavior (per your request):
 *  - Flipper UP immediately at start of SHOT3_FEED and SHOT4_FEED
 *  - Flipper DOWN at the start of SHOT3_DECOMP and SHOT4_DECOMP
 *
 * Intake1 burst behavior unchanged:
 *  - AFTER shot #1 is taken (after SHOT1_FEED -> SHOT1_DECOMP),
 *    intakeOneMotor goes FULL power and stays on until shot #4 completes.
 */
@TeleOp(name = "FourShotTest_v2", group = "Test")
public class ShootThreeTestV2 extends OpMode {

    // ======= CONFIG NAMES =======
    private static final String SHOOTER_NAME = "ShooterMotor";
    private static final String FLICKER_NAME = "Outertake";
    private static final String INTAKE1_NAME = "intakeOneMotor";
    private static final String INTAKE2_NAME = "intakeTwoMotor";
    private static final String FLIPPER_NAME = "fingler";

    // ======= HARDWARE =======
    private DcMotorEx shooter;
    private DcMotorEx flicker;
    private DcMotorEx intake1;
    private DcMotorEx intake2;
    private Servo flipper;

    // ======= FLIPPER POSITIONS (your tuned values) =======
    // DOWN = 0.15 (ball path clear)
    // UP   = 0.00 (flip ball up)
    private static final double FLIP_DOWN = 0.15;
    private static final double FLIP_UP   = 0.00;

    // ======= SHOOTER TARGETS (ticks/sec) =======
    private static final double SHOOTER_TARGET_VEL = 1250.0;
    private static final double SHOOTER_READY_MIN  = 1230.0;
    private static final double SHOOTER_READY_MAX  = 1260.0;
    private static final int READY_STABLE_MS = 110;
    private static final int MIN_RECOVER_MS = 120;

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

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);
        flicker = hardwareMap.get(DcMotorEx.class, FLICKER_NAME);
        intake1 = hardwareMap.get(DcMotorEx.class, INTAKE1_NAME);
        intake2 = hardwareMap.get(DcMotorEx.class, INTAKE2_NAME);
        flipper = hardwareMap.get(Servo.class, FLIPPER_NAME);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flicker.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake1.setDirection(DcMotor.Direction.REVERSE);

        flicker.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Start flipper DOWN for safety
        flipper.setPosition(FLIP_DOWN);

        stopSequenceMotors();
        stopShooter();

        telemetry.addLine("FourShotTest_v2 READY (with flipper)");
        telemetry.addLine("A = Toggle IntakeOne ONLY (intakeOneMotor)");
        telemetry.addLine("Y = Toggle Shooter ON/OFF (velocity control)");
        telemetry.addLine("X = Fire 4-shot sequence (only when Shooter READY)");
        telemetry.addLine("B = ABORT sequence (stops shooter + intake2 + flicker)");
        telemetry.addLine("Flipper: UP during Shot3/Shot4 FEED, DOWN at start of DECOMP");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;

        boolean aPressed = a && !prevA;
        boolean bPressed = b && !prevB;
        boolean xPressed = x && !prevX;
        boolean yPressed = y && !prevY;

        // ===== Intake 1 toggle (independent) =====
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
                shooter.setVelocity(SHOOTER_TARGET_VEL);
                resetReadyStable();
            }
        }
        if (shooterEnabled) {
            shooter.setVelocity(SHOOTER_TARGET_VEL);
        }

        // ===== Abort =====
        if (bPressed) {
            abortSequence("User pressed B");
        }

        // ===== Start 4-shot sequence =====
        if (xPressed && state == SeqState.IDLE) {
            if (!shooterEnabled) {
                // do nothing
            } else if (!isShooterReadyStable()) {
                // do nothing
            } else {
                startFeed(SeqState.SHOT1_FEED);
            }
        }

        // ===== Run sequence state machine =====
        switch (state) {
            case IDLE:
                intake1BurstOverride = false;
                // Safety: keep flipper down when idle
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
                    // End burst: flipper down
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
                // Keep flipper down for safety
                flipper.setPosition(FLIP_DOWN);
                break;
        }

        // ===== Telemetry =====
        double vShooter = shooter.getVelocity();
        telemetry.addLine("=== FourShotTest_v2 (with Flipper) ===");
        telemetry.addData("Intake1 Toggle (A)", intake1Enabled);
        telemetry.addData("Intake1 BurstOverride", intake1BurstOverride);
        telemetry.addData("Shooter Toggle (Y)", shooterEnabled);
        telemetry.addData("State", state);

        telemetry.addData("ShooterVel", "%.0f", vShooter);
        telemetry.addData("ReadyWindow", "[%.0f..%.0f]", SHOOTER_READY_MIN, SHOOTER_READY_MAX);
        telemetry.addData("ReadyStable(ms)", "%.0f / %d", readyStableTimer.milliseconds(), READY_STABLE_MS);
        telemetry.addData("SeqTimer(ms)", "%.0f", seqTimer.milliseconds());
        telemetry.addData("FlipperPos", "%.2f", flipper.getPosition());

        telemetry.addLine("X only works when Shooter is ON and READY.");
        telemetry.update();

        prevA=a; prevB=b; prevX=x; prevY=y;
    }

    // ======== Sequence helpers (intake2 + flicker + flipper during shot3/shot4 feed) ========

    private void startFeed(SeqState feedState) {
        // Flipper UP only for Shot3 and Shot4 feeds (immediately)
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
        // After shot #1 is taken, force intake1 to full power until shot #4 completes.
        if (decompState == SeqState.SHOT1_DECOMP) {
            intake1BurstOverride = true;
        }

        // Flipper DOWN at start of SHOT3_DECOMP and SHOT4_DECOMP (per request)
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
        // Flipper should be DOWN during recover
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
        // Keep flipper down when stopping sequence motors
        flipper.setPosition(FLIP_DOWN);
    }

    // ======== Shooter ready helpers ========

    private boolean isShooterReadyStable() {
        if (!shooterEnabled) return false;

        double v = shooter.getVelocity();
        boolean inWindow = (v >= SHOOTER_READY_MIN && v <= SHOOTER_READY_MAX);

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

    // ======== Abort ========

    private void abortSequence(String reason) {
        stopSequenceMotors();
        stopShooter();
        shooterEnabled = false;
        resetReadyStable();
        state = SeqState.ABORTED;

        // cancel burst override on abort
        intake1BurstOverride = false;

        // Safety: flipper down
        flipper.setPosition(FLIP_DOWN);

        telemetry.addLine("ABORTED: " + reason);
        telemetry.update();
    }

    @Override
    public void stop() {
        intake1.setPower(0);
        stopSequenceMotors();
        stopShooter();
        flipper.setPosition(FLIP_DOWN);
    }
}




