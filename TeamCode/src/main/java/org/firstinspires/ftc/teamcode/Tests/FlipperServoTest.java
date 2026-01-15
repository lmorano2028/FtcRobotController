package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "FlipperServoTest", group = "Test")
public class FlipperServoTest extends OpMode {

    private Servo flipper;

    // Start guesses — you will tune these
    private double FLIP_DOWN = 0.10;
    private double FLIP_UP   = 0.70;

    // For a “kick” pulse
    private final ElapsedTime kickTimer = new ElapsedTime();
    private boolean kicking = false;

    // Fine adjustment
    private double pos = FLIP_DOWN;
    private final double step = 0.01;

    // Button edge detection
    private boolean prevA = false, prevY = false, prevX = false;
    private boolean prevDpadUp = false, prevDpadDown = false;

    @Override
    public void init() {
        // IMPORTANT: This name must match your Robot Config servo name
        flipper = hardwareMap.get(Servo.class, "fingler");

        // Start at DOWN position

        telemetry.addLine("FlipperServoTest initialized.");
        telemetry.addLine("A=DOWN | Y=UP | X=KICK (UP then DOWN) | DpadUp/Down fine adjust");
        telemetry.addData("FLIP_DOWN", FLIP_DOWN);
        telemetry.addData("FLIP_UP", FLIP_UP);
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean a = gamepad1.a;
        boolean y = gamepad1.y;
        boolean x = gamepad1.x;
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;

        boolean aPressed = a && !prevA;
        boolean yPressed = y && !prevY;
        boolean xPressed = x && !prevX;
        boolean upPressed = dpadUp && !prevDpadUp;
        boolean downPressed = dpadDown && !prevDpadDown;

        // Manual UP/DOWN commands
        if (aPressed) {
            kicking = false;
            pos = FLIP_DOWN;
            flipper.setPosition(pos);
        }
        if (yPressed) {
            kicking = false;
            pos = FLIP_UP;
            flipper.setPosition(pos);
        }

        // Fine adjust current position
        if (upPressed) {
            kicking = false;
            pos = clamp(pos + step);
            flipper.setPosition(pos);
        }
        if (downPressed) {
            kicking = false;
            pos = clamp(pos - step);
            flipper.setPosition(pos);
        }

        // Kick pulse: go UP briefly then return DOWN
        if (xPressed) {
            kicking = true;
            kickTimer.reset();
            flipper.setPosition(FLIP_UP);
            pos = FLIP_UP;
        }

        if (kicking) {
            // Hold UP for 120ms, then go DOWN
            if (kickTimer.milliseconds() >= 120) {
                flipper.setPosition(FLIP_DOWN);
                pos = FLIP_DOWN;
                kicking = false;
            }
        }

        telemetry.addData("CurrentPos", "%.2f", pos);
        telemetry.addData("Kicking", kicking);
        telemetry.addLine("Tune: find DOWN that doesn't rub; find UP that reliably flips ball.");
        telemetry.update();

        prevA = a; prevY = y; prevX = x;
        prevDpadUp = dpadUp; prevDpadDown = dpadDown;
    }

    private double clamp(double v) {
        if (v < 0.0) return 0.0;
        if (v > 1.0) return 1.0;
        return v;
    }
}



