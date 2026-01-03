package org.firstinspires.ftc.teamcode.turretTesting;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TurretRecenterAndLimitsTest", group="Test")
public class TurretRecenterAndLimitsTest extends OpMode {

    private static final String LEFT_NAME  = "LRotation";
    private static final String RIGHT_NAME = "RRotation";

    // You found this is correct with gears installed
    private static final boolean MIRROR_RIGHT = false;

    // Movement tuning
    private static final double STICK_GAIN = 0.010;
    private static final double DPAD_STEP  = 0.003;
    private static final double DEADZONE   = 0.06;

    private Servo left, right;

    // Start at CENTER (this is what you want to become "forward")
    private double pos = 0.50;

    // Recordable limits
    private double safeMin = 0.20;
    private double safeMax = 0.80;

    // Toggle clamp enforcement
    private boolean enforceLimits = false; // start OFF for recenter

    // Edge detection
    private boolean prevA=false, prevB=false, prevX=false, prevY=false;

    @Override
    public void init() {
        left  = hardwareMap.get(Servo.class, LEFT_NAME);
        right = hardwareMap.get(Servo.class, RIGHT_NAME);

        apply(pos);

        telemetry.addLine("TurretRecenterAndLimitsTest READY");
        telemetry.addLine("STEP 1: Press X to force pos=0.50 and HOLD (recenter gears now).");
        telemetry.addLine("Stick X = move, Dpad L/R = nudge");
        telemetry.addLine("Y toggles limit clamp ON/OFF");
        telemetry.addLine("A saves SAFE MIN at current pos");
        telemetry.addLine("B saves SAFE MAX at current pos");
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

        // X = hard center at 0.50 (use while loosening/retightening gears)
        if (xPressed) pos = 0.50;

        // Toggle clamp enforcement
        if (yPressed) enforceLimits = !enforceLimits;

        // Movement
        double stick = gamepad1.left_stick_x;
        if (Math.abs(stick) < DEADZONE) stick = 0;
        pos += stick * STICK_GAIN;

        if (gamepad1.dpad_left)  pos -= DPAD_STEP;
        if (gamepad1.dpad_right) pos += DPAD_STEP;

        // Save limits
        if (aPressed) safeMin = pos;
        if (bPressed) safeMax = pos;

        // Clamp
        if (enforceLimits) {
            pos = clamp(pos, safeMin, safeMax);
        } else {
            pos = clamp(pos, 0.0, 1.0);
        }

        apply(pos);

        telemetry.addLine("=== TurretRecenterAndLimitsTest ===");
        telemetry.addData("pos", "%.3f", pos);
        telemetry.addData("ClampEnforced (Y)", enforceLimits);
        telemetry.addData("SAFE_MIN (A)", "%.3f", safeMin);
        telemetry.addData("SAFE_MAX (B)", "%.3f", safeMax);
        telemetry.addLine("Press X to lock to 0.50 while you remount gears for forward.");
        telemetry.update();

        prevA=a; prevB=b; prevX=x; prevY=y;
    }

    private void apply(double p) {
        left.setPosition(p);
        if (MIRROR_RIGHT) right.setPosition(1.0 - p);
        else right.setPosition(p);
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}


