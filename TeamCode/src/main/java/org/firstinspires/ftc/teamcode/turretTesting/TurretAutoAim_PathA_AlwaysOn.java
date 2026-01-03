package org.firstinspires.ftc.teamcode.turretTesting;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

@TeleOp(name="TurretAutoAim_PathA_AlwaysOn", group="Test")
public class TurretAutoAim_PathA_AlwaysOn extends OpMode {

    // ===== Hardware names =====
    private static final String LIMELIGHT_NAME = "limelight";
    private static final String LEFT_SERVO_NAME  = "LRotation";
    private static final String RIGHT_SERVO_NAME = "RRotation";

    // ===== Pipeline / Tag =====
    private static final int APRILTAG_PIPELINE_INDEX = 0;
    private static int ACTIVE_GOAL_ID = 20; // press A=20, B=24

    // ===== Your turret limits =====
    private static final double SAFE_MIN = 0.10;
    private static final double SAFE_MAX = 0.90;
    private static final double HOME_POS = 0.50;

    // If your right servo needs mirroring in code, flip this to true
    private static final boolean RIGHT_MIRROR_IN_CODE = false;

    // ===== Control tuning =====
    // Kp: servo-units per degree. Start small.
    private static final double KP_DEG_TO_POS = 0.0020;

    // Deadband in degrees (prevents jitter)
    private static final double BEARING_DEADBAND_DEG = 1.2;

    // Max servo movement rate (servo-units per second). Keeps things smooth.
    private static final double SERVO_SLEW_PER_SEC = 0.55;

    // Hard clamp per loop (extra safety). Optional but helpful.
    private static final double MAX_STEP_PER_LOOP = 0.020;

    // Smoothing for bearing (0..1). Higher = more responsive, lower = smoother.
    private static final double BEARING_ALPHA = 0.35;

    // Flip this if the turret moves the wrong direction.
    private static final boolean INVERT_TURRET_RESPONSE = false;

    // ===== Internal =====
    private Limelight3A limelight;
    private Servo left, right;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double turretCmd = HOME_POS;

    // Vision
    private boolean tagSeen = false;

    // Raw bearing in degrees from pose: atan2(x,z)
    private double bearingRawDeg = Double.NaN;

    // Filtered bearing
    private double bearingFiltDeg = 0.0;

    // “Zero” capture (center offset)
    private double bearingZeroDeg = 0.0;
    private boolean zeroCaptured = false;

    // Edge detect
    private boolean prevX=false, prevY=false, prevA=false, prevB=false;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        left  = hardwareMap.get(Servo.class, LEFT_SERVO_NAME);
        right = hardwareMap.get(Servo.class, RIGHT_SERVO_NAME);

        turretCmd = clamp(HOME_POS, SAFE_MIN, SAFE_MAX);
        applyTurret(turretCmd);

        limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
        limelight.start();

        loopTimer.reset();

        telemetry.addLine("TurretAutoAim_PathA_AlwaysOn READY");
        telemetry.addData("ACTIVE_GOAL_ID", ACTIVE_GOAL_ID);
        telemetry.addLine("A = Tag20 | B = Tag24");
        telemetry.addLine("X = CAPTURE ZERO (center tag once)");
        telemetry.addLine("Y = HOME (0.50)");
        telemetry.addLine("Dpad L/R = manual nudge (sanity)");
        telemetry.addLine("AutoAim is always ON after zero is captured and tag is seen.");
        telemetry.update();
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        dt = clamp(dt, 0.005, 0.050);

        boolean x  = gamepad1.x;
        boolean y  = gamepad1.y;
        boolean a  = gamepad1.a;
        boolean b  = gamepad1.b;

        boolean xPressed  = x && !prevX;
        boolean yPressed  = y && !prevY;
        boolean aPressed  = a && !prevA;
        boolean bPressed  = b && !prevB;

        // Select which goal ID
        if (aPressed) { ACTIVE_GOAL_ID = 20; zeroCaptured = false; }
        if (bPressed) { ACTIVE_GOAL_ID = 24; zeroCaptured = false; }

        // Read vision -> bearingRawDeg (atan2(x,z))
        updateBearingFromChosenTagPose();

        // Filter bearing (if tag seen)
        if (tagSeen && !Double.isNaN(bearingRawDeg)) {
            // init filter cleanly on first seen after loss
            if (!zeroCaptured) {
                // still allow filter to track so telemetry looks sane
                bearingFiltDeg = bearingFiltDeg * (1.0 - BEARING_ALPHA) + bearingRawDeg * BEARING_ALPHA;
            } else {
                bearingFiltDeg = bearingFiltDeg * (1.0 - BEARING_ALPHA) + bearingRawDeg * BEARING_ALPHA;
            }
        }

        // Manual nudge (sanity test, always allowed)
        if (gamepad1.dpad_left)  turretCmd -= 0.003;
        if (gamepad1.dpad_right) turretCmd += 0.003;
        turretCmd = clamp(turretCmd, SAFE_MIN, SAFE_MAX);

        // X = capture zero offset when you’re aimed correctly
        if (xPressed && tagSeen && !Double.isNaN(bearingRawDeg)) {
            bearingZeroDeg = bearingRawDeg;
            bearingFiltDeg = bearingRawDeg; // start filter at same value
            zeroCaptured = true;
        }

        // Y = force home
        if (yPressed) {
            turretCmd = clamp(HOME_POS, SAFE_MIN, SAFE_MAX);
        }

        // Compute adjusted bearing error (deg). Want 0.
        double errDeg = Double.NaN;
        if (tagSeen && zeroCaptured && !Double.isNaN(bearingFiltDeg)) {
            errDeg = bearingFiltDeg - bearingZeroDeg;
        }

        // ===== AutoAim (Path A): Always on IF zero captured + tag seen =====
        if (tagSeen && zeroCaptured && !Double.isNaN(errDeg)) {

            // deadband
            if (Math.abs(errDeg) < BEARING_DEADBAND_DEG) errDeg = 0.0;

            // Convert error deg -> servo delta
            // We want to reduce error toward 0.
            double delta = errDeg * KP_DEG_TO_POS;

            if (INVERT_TURRET_RESPONSE) delta = -delta;

            // Move opposite the error to drive it toward zero
            double desired = turretCmd - delta;

            // clamp desired
            desired = clamp(desired, SAFE_MIN, SAFE_MAX);

            // slew limit
            turretCmd = slewTo(turretCmd, desired, SERVO_SLEW_PER_SEC, dt);

            // extra per-loop clamp (prevents any “jump” if dt spikes)
            turretCmd = clamp(turretCmd,
                    Math.max(SAFE_MIN, turretCmd - MAX_STEP_PER_LOOP),
                    Math.min(SAFE_MAX, turretCmd + MAX_STEP_PER_LOOP));

            // If at limits, hold there
            if (turretCmd <= SAFE_MIN + 1e-3) turretCmd = SAFE_MIN;
            if (turretCmd >= SAFE_MAX - 1e-3) turretCmd = SAFE_MAX;
        }

        applyTurret(turretCmd);

        telemetry.addLine("=== TurretAutoAim_PathA_AlwaysOn ===");
        telemetry.addData("ACTIVE_GOAL_ID", ACTIVE_GOAL_ID);
        telemetry.addData("tagSeen", tagSeen);
        telemetry.addData("bearingRawDeg", Double.isNaN(bearingRawDeg) ? "N/A" : String.format("%.2f", bearingRawDeg));
        telemetry.addData("bearingFiltDeg", String.format("%.2f", bearingFiltDeg));
        telemetry.addData("zeroCaptured(X)", zeroCaptured);
        telemetry.addData("bearingZeroDeg", String.format("%.2f", bearingZeroDeg));
        telemetry.addData("errDeg", Double.isNaN(errDeg) ? "N/A" : String.format("%.2f", errDeg));
        telemetry.addData("turretCmd", "%.3f", turretCmd);
        telemetry.addData("SAFE[min..max]", "%.2f..%.2f", SAFE_MIN, SAFE_MAX);
        telemetry.addData("KP", "%.4f", KP_DEG_TO_POS);
        telemetry.addData("Deadband(deg)", "%.2f", BEARING_DEADBAND_DEG);
        telemetry.addData("SlewPerSec", "%.2f", SERVO_SLEW_PER_SEC);
        telemetry.addLine("Procedure: center tag -> press X -> drive/rotate robot and turret should hold errDeg ~ 0.");
        telemetry.update();

        prevX=x; prevY=y; prevA=a; prevB=b;
    }

    /**
     * Path A: compute bearing from 3D pose.
     * bearingDeg = atan2(x, z) (KEEP SIGN ON x, abs z)
     */
    private void updateBearingFromChosenTagPose() {
        tagSeen = false;
        bearingRawDeg = Double.NaN;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return;

        LLResultTypes.FiducialResult chosen = null;
        for (LLResultTypes.FiducialResult t : tags) {
            if ((int) t.getFiducialId() == ACTIVE_GOAL_ID) {
                chosen = t;
                break;
            }
        }
        if (chosen == null) return;

        Pose3D pose;
        try {
            pose = chosen.getCameraPoseTargetSpace();
        } catch (Exception e) {
            return;
        }
        if (pose == null) return;

        Position p = pose.getPosition();

        // KEEP SIGN ON X. Only abs Z.
        double xMm = DistanceUnit.MM.fromUnit(p.unit, p.x);
        double zMm = Math.abs(DistanceUnit.MM.fromUnit(p.unit, p.z));

        if (zMm < 1.0) return;

        double bearingRad = Math.atan2(xMm, zMm);
        bearingRawDeg = Math.toDegrees(bearingRad);
        tagSeen = true;
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

    @Override
    public void stop() {
        limelight.stop();
    }
}
