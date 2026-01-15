package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "FlickerTest", group = "Test")
public class FlickerTest extends OpMode {

    private DcMotorEx outertake;

    // ===== Tunables (start here, then tune FEED_TICKS) =====
    private int FEED_TICKS = 400;          // Tune this until 1 press = 1 ball feed
    private double FEED_POWER = 1.0;      // Flicker power for feed move
    private long LOCKOUT_MS = 150;         // Ignore new feed requests right after a feed finishes

    private double JOG_POWER = -0.40;      // Reverse jog power to unjam
    private long JOG_MS = 150;             // How long reverse jog runs

    // ===== State tracking =====
    private int lastTargetPos = 0;
    private long lastFeedDoneTimeMs = 0;

    private boolean isFeeding = false;
    private boolean isJogging = false;

    private final ElapsedTime jogTimer = new ElapsedTime();

    // ===== Button edge detection =====
    private boolean prevX = false;
    private boolean prevB = false;
    private boolean prevY = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;

    @Override
    public void init() {
        // Motor config name you provided:

        outertake = hardwareMap.get(DcMotorEx.class, "Outertake");

        // If it spins the wrong direction for feeding, flip it:
        // outertake.setDirection(DcMotorSimple.Direction.REVERSE);

        // BRAKE is important so it stops cleanly and repeatably
        outertake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoder ONCE here for clean testing
        outertake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outertake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outertake.setPower(0);

        lastTargetPos = outertake.getCurrentPosition();
        lastFeedDoneTimeMs = 0;

        telemetry.addLine("FlickerTest ready.");
        telemetry.addLine("X = Feed One | B = Reverse Jog (unjam) | Y = Reset Encoder (debug)");
        telemetry.addLine("DpadUp/Down adjusts FEED_TICKS (+/-50)");
        telemetry.update();
    }

    @Override
    public void loop() {
        long nowMs = System.currentTimeMillis();

        // Read buttons
        boolean x = gamepad1.x;
        boolean b = gamepad1.b;
        boolean y = gamepad1.y;
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;

        // Edge detection (true only on the rising edge)
        boolean xPressed = x && !prevX;
        boolean bPressed = b && !prevB;
        boolean yPressed = y && !prevY;
        boolean upPressed = dpadUp && !prevDpadUp;
        boolean downPressed = dpadDown && !prevDpadDown;

        // Allow tuning FEED_TICKS live
        if (upPressed) FEED_TICKS += 50;
        if (downPressed) FEED_TICKS = Math.max(50, FEED_TICKS - 50);

        // If currently jogging, manage jog timing
        if (isJogging) {
            if (jogTimer.milliseconds() >= JOG_MS) {
                outertake.setPower(0);
                outertake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                isJogging = false;
            }
        }

        // If currently feeding, detect completion
        if (isFeeding) {
            // Motor is done when it is not busy anymore (in RUN_TO_POSITION)
            if (!outertake.isBusy()) {
                outertake.setPower(0);
                outertake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                isFeeding = false;
                lastFeedDoneTimeMs = nowMs;
            }
        }

        // Compute lockout
        boolean inLockout = (nowMs - lastFeedDoneTimeMs) < LOCKOUT_MS;

        // ===== Debug reset encoder (optional) =====
        // Only allow when not feeding/jogging so you don't confuse the controller mid-move.
        if (yPressed && !isFeeding && !isJogging) {
            outertake.setPower(0);
            outertake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            outertake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lastTargetPos = 0;
            lastFeedDoneTimeMs = nowMs;
        }

        // ===== Reverse jog (unjam) =====
        // Only allow if not feeding and not in lockout (to keep behavior clean).
        if (bPressed && !isFeeding && !isJogging) {
            // Start a timed reverse jog
            outertake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            outertake.setPower(JOG_POWER);
            jogTimer.reset();
            isJogging = true;
        }

        // ===== Feed one (main function) =====
        // Only allow if:
        // - not already feeding
        // - not jogging
        // - not in lockout
        if (xPressed && !isFeeding && !isJogging && !inLockout) {
            int startPos = outertake.getCurrentPosition();
            lastTargetPos = startPos + FEED_TICKS;

            outertake.setTargetPosition(lastTargetPos);
            outertake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outertake.setPower(FEED_POWER);

            isFeeding = true;
        }

        // ===== Telemetry =====
        int currentPos = outertake.getCurrentPosition();
        DcMotor.RunMode mode = outertake.getMode();

        telemetry.addLine("=== FlickerTest (Outertake) ===");
        telemetry.addData("Mode", mode);
        telemetry.addData("FEED_TICKS", FEED_TICKS);
        telemetry.addData("FEED_POWER", "%.2f", FEED_POWER);
        telemetry.addData("LOCKOUT_MS", LOCKOUT_MS);

        telemetry.addData("CurrentPos", currentPos);
        telemetry.addData("LastTargetPos", lastTargetPos);
        telemetry.addData("isBusy()", outertake.isBusy());
        telemetry.addData("isFeeding", isFeeding);
        telemetry.addData("isJogging", isJogging);

        telemetry.addData("InLockout", inLockout);
        telemetry.addData("LockoutRemaining(ms)", Math.max(0, (lastFeedDoneTimeMs + LOCKOUT_MS) - nowMs));

        telemetry.addLine("Controls: X=FeedOne | B=ReverseJog | Y=ResetEnc | DpadUp/Down FEED_TICKS +/-50");
        telemetry.update();

        // Save previous button states
        prevX = x;
        prevB = b;
        prevY = y;
        prevDpadUp = dpadUp;
        prevDpadDown = dpadDown;
    }

    @Override
    public void stop() {
        // Safe stop
        if (outertake != null) {
            outertake.setPower(0);
            outertake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}