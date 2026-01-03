package org.firstinspires.ftc.teamcode.turretTesting;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pinpoint.GoBildaPinpointDriver;

import java.util.Locale;

@TeleOp(name="TurretAutoAim_PathC_Pinpoint_BlueOnly", group="Test")
public class TurretAutoAim_PathC_Pinpoint_BlueOnly extends OpMode {

    // ===== Hardware names =====
    private static final String LEFT_SERVO_NAME  = "LRotation";
    private static final String RIGHT_SERVO_NAME = "RRotation";
    private static final String PINPOINT_NAME    = "pinpoint";

    // ===== Your turret limits =====
    private static final double SAFE_MIN = 0.10;
    private static final double SAFE_MAX = 0.90;
    private static final double HOME_POS = 0.50;

    // You said mirror right = false works with gears installed
    private static final boolean RIGHT_MIRROR_IN_CODE = false;

    // ===== Your measured conversion =====
    // turret moved from 0.5 -> 0.6 = +0.10 position = +14.5 deg
    // pos_per_deg = 0.10/14.5 = 0.00689
    private static final double POS_PER_DEG = -0.00689;

    // ===== BLUE GOAL FIELD COORDINATE (mm) =====
    // From your measurement: robot at (0,0,0) facing BLUE goal, push into BLUE goal wall:
    // X=1789.3mm, Y=2.7mm
    private static final double BLUE_GOAL_X_MM = 1789.3;
    private static final double BLUE_GOAL_Y_MM = 2.7;

    // ===== Pinpoint offsets (MUST match your working sample) =====
    // odo.setOffsets(82.55, -95.25, DistanceUnit.MM);
    private static final double PINPOINT_X_OFFSET_MM = 82.55;
    private static final double PINPOINT_Y_OFFSET_MM = -95.25;

    // ===== Pinpoint encoder directions (MUST match your working sample) =====
    // odo.setEncoderDirections(FORWARD, REVERSED);
    private static final GoBildaPinpointDriver.EncoderDirection X_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection Y_DIR =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;

    // ===== Control tuning =====
    // This is NOT a shooter PID; it is a "how aggressively do we correct turret angle"
    // Start small. Increase slowly if it feels sluggish.
    private static double KP_ERR_TO_TURRET_DEG = 0.12;    // turretRelDeg += KP * errDeg each loop
    private static double DEADBAND_DEG = 1.0;
    private static double SERVO_SLEW_PER_SEC = 0.75;

    // Flip if turret turns the wrong direction
    private static boolean INVERT_TURRET_RESPONSE = false;

    // ===== Internal =====
    private Servo left, right;
    private GoBildaPinpointDriver odo;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime statusTimer = new ElapsedTime();

    private double turretCmd = HOME_POS;
    private double turretRelDeg = 0.0; // 0deg corresponds to HOME_POS

    private boolean autoAimEnabled = true;

    // Edge detect
    private boolean prevRB=false, prevY=false, prevX=false, prevA=false, prevB=false;

    // Telemetry
    private double robotXmm=0, robotYmm=0, robotHdeg=0;
    private double bearingFieldDeg=0;      // angle robot->goal in field frame
    private double turretTargetRelDeg=0;   // desired turret angle relative to robot forward
    private double errDeg=0;

    @Override
    public void init() {
        left  = hardwareMap.get(Servo.class, LEFT_SERVO_NAME);
        right = hardwareMap.get(Servo.class, RIGHT_SERVO_NAME);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);

        // ===== Pinpoint init EXACTLY like your sample =====
        odo.setOffsets(PINPOINT_X_OFFSET_MM, PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(X_DIR, Y_DIR);
        odo.resetPosAndIMU(); // robot MUST be stationary

        // Start turret at home
        turretCmd = clamp(HOME_POS, SAFE_MIN, SAFE_MAX);
        turretRelDeg = 0.0;
        applyTurret(turretCmd);

        loopTimer.reset();
        statusTimer.reset();

        telemetry.addLine("TurretAutoAim_PathC_Pinpoint_BlueOnly READY");
        telemetry.addLine("RB = toggle AutoAim");
        telemetry.addLine("Y  = turret HOME");
        telemetry.addLine("X  = resetPosAndIMU (robot must be STILL)");
        telemetry.addLine("A  = resetPosAndIMU (same as X, like your sample)");
        telemetry.addLine("B  = recalibrateIMU (no position reset, like your sample)");
        telemetry.addLine("DPAD L/R = manual turret nudge (sanity check)");
        telemetry.addData("Goal(BLUE) X,Y (mm)", "%.1f , %.1f", BLUE_GOAL_X_MM, BLUE_GOAL_Y_MM);
        telemetry.addData("SAFE[min..max]", "%.2f..%.2f", SAFE_MIN, SAFE_MAX);
        telemetry.addData("HOME_POS", "%.2f", HOME_POS);
        telemetry.addData("POS_PER_DEG", "%.5f", POS_PER_DEG);
        telemetry.update();
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        dt = clamp(dt, 0.005, 0.050);

        boolean rb = gamepad1.right_bumper;
        boolean y  = gamepad1.y;
        boolean x  = gamepad1.x;
        boolean a  = gamepad1.a;
        boolean b  = gamepad1.b;

        boolean rbPressed = rb && !prevRB;
        boolean yPressed  = y  && !prevY;
        boolean xPressed  = x  && !prevX;
        boolean aPressed  = a  && !prevA;
        boolean bPressed  = b  && !prevB;

        if (rbPressed) autoAimEnabled = !autoAimEnabled;

        // ===== Pinpoint update (same as sample) =====
        odo.update();

        // X or A = resetPosAndIMU (same behavior as your sample uses on A)
        if (xPressed || aPressed) {
            odo.resetPosAndIMU(); // robot MUST be still
            turretCmd = clamp(HOME_POS, SAFE_MIN, SAFE_MAX);
            turretRelDeg = 0.0;
        }

        // B = recalibrate IMU only (same as your sample)
        if (bPressed) {
            odo.recalibrateIMU();
        }

        // Y = force turret home
        if (yPressed) {
            turretCmd = clamp(HOME_POS, SAFE_MIN, SAFE_MAX);
            turretRelDeg = 0.0;
        }

        // Manual nudge (sanity test)
        if (gamepad1.dpad_left)  turretCmd -= 0.003;
        if (gamepad1.dpad_right) turretCmd += 0.003;
        turretCmd = clamp(turretCmd, SAFE_MIN, SAFE_MAX);

        // ===== Read robot pose =====
        Pose2D pos = odo.getPosition();
        robotXmm = pos.getX(DistanceUnit.MM);
        robotYmm = pos.getY(DistanceUnit.MM);
        robotHdeg = pos.getHeading(AngleUnit.DEGREES);

        // ===== Compute field bearing robot->BLUE goal =====
        double dx = BLUE_GOAL_X_MM - robotXmm;
        double dy = BLUE_GOAL_Y_MM - robotYmm;

        // Field bearing: atan2(dy, dx) (deg)
        bearingFieldDeg = Math.toDegrees(Math.atan2(dy, dx));

        // Turret target relative to robot forward:
        // If robot heading=0 means "facing BLUE goal" (your convention), then:
        // turretTargetRelDeg = wrap(bearingFieldDeg - robotHeadingDeg)
        turretTargetRelDeg = wrapDeg(bearingFieldDeg - robotHdeg);

        // Error between current turretRelDeg and desired
        errDeg = wrapDeg(turretTargetRelDeg - turretRelDeg);

        // Deadband
        if (Math.abs(errDeg) < DEADBAND_DEG) errDeg = 0.0;

        // ===== AutoAim update =====
        if (autoAimEnabled) {
            double stepDeg = KP_ERR_TO_TURRET_DEG * errDeg;
            if (INVERT_TURRET_RESPONSE) stepDeg = -stepDeg;

            // update turret relative angle estimate
            turretRelDeg = wrapDeg(turretRelDeg + stepDeg);

            // Convert turretRelDeg -> servo command about HOME_POS
            double desiredCmd = HOME_POS + (turretRelDeg * POS_PER_DEG);

            // Clamp to safe limits
            desiredCmd = clamp(desiredCmd, SAFE_MIN, SAFE_MAX);

            // Slew limit in servo-position units
            turretCmd = slewTo(turretCmd, desiredCmd, SERVO_SLEW_PER_SEC, dt);

            // Hard clamp
            turretCmd = clamp(turretCmd, SAFE_MIN, SAFE_MAX);

            // If we hit a limit, keep turretRelDeg consistent with clamp (prevents integrator-like drift)
            turretRelDeg = (turretCmd - HOME_POS) / POS_PER_DEG;
            turretRelDeg = wrapDeg(turretRelDeg);
        }

        applyTurret(turretCmd);

        // ===== Telemetry =====
        String posStr = String.format(Locale.US, "{X: %.1f, Y: %.1f, H: %.2f}",
                robotXmm, robotYmm, robotHdeg);

        telemetry.addLine("=== Path C: Pinpoint Turret Aim (BLUE ONLY) ===");
        telemetry.addData("AutoAim (RB)", autoAimEnabled);
        telemetry.addData("Pinpoint Status", odo.getDeviceStatus());
        telemetry.addData("Robot Pose (mm,deg)", posStr);

        telemetry.addData("Goal BLUE (mm)", "X=%.1f Y=%.1f", BLUE_GOAL_X_MM, BLUE_GOAL_Y_MM);
        telemetry.addData("bearingFieldDeg", "%.2f", bearingFieldDeg);
        telemetry.addData("turretTargetRelDeg", "%.2f", turretTargetRelDeg);
        telemetry.addData("turretRelDeg(est)", "%.2f", turretRelDeg);
        telemetry.addData("errDeg", "%.2f", errDeg);

        telemetry.addData("turretCmd", "%.3f", turretCmd);
        telemetry.addData("SAFE[min..max]", "%.2f..%.2f", SAFE_MIN, SAFE_MAX);

        telemetry.addData("KP_ERR_TO_TURRET_DEG", "%.3f", KP_ERR_TO_TURRET_DEG);
        telemetry.addData("DEADBAND_DEG", "%.2f", DEADBAND_DEG);
        telemetry.addData("SlewPerSec", "%.2f", SERVO_SLEW_PER_SEC);

        telemetry.addLine("Controls: RB auto | Y home | X/A resetPos+IMU | B recalIMU | Dpad L/R nudge");
        telemetry.update();

        prevRB=rb; prevY=y; prevX=x; prevA=a; prevB=b;
    }

    private void applyTurret(double pos) {
        pos = clamp(pos, 0.0, 1.0);
        left.setPosition(pos);

        if (RIGHT_MIRROR_IN_CODE) right.setPosition(1.0 - pos);
        else right.setPosition(pos);
    }

    private double slewTo(double current, double target, double ratePerSec, double dt) {
        double maxStep = ratePerSec * dt;
        double delta = target - current;
        if (delta >  maxStep) delta =  maxStep;
        if (delta < -maxStep) delta = -maxStep;
        return current + delta;
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // Wrap to [-180, +180)
    private double wrapDeg(double deg) {
        while (deg >= 180.0) deg -= 360.0;
        while (deg < -180.0) deg += 360.0;
        return deg;
    }
}
