package org.firstinspires.ftc.teamcode.turretTesting;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.pinpoint.GoBildaPinpointDriver;

import java.util.List;
import java.util.Locale;

@TeleOp(name="TurretAutoAim_PathC_LLTrim_BlueOnly", group="Test")
public class TurretAutoAim_PathC_LLTrim_BlueOnly extends OpMode {

    // ===== Hardware names =====
    private static final String LEFT_SERVO_NAME  = "LRotation";
    private static final String RIGHT_SERVO_NAME = "RRotation";
    private static final String PINPOINT_NAME    = "pinpoint";
    private static final String LIMELIGHT_NAME   = "limelight";

    // ===== Limelight / Pipeline =====
    private static final int APRILTAG_PIPELINE_INDEX = 0;

    // ===== Alliance selection (init) =====
    private enum Alliance { BLUE, RED }
    private Alliance selectedAlliance = Alliance.BLUE;   // default
    private int ACTIVE_GOAL_ID = 20;                     // updated in applyAlliance()
    private double GOAL_X_MM = 1789.3;                   // updated in applyAlliance()
    private double GOAL_Y_MM = 2.7;                      // updated in applyAlliance()

    // Your measured wall-touch coordinates:
    private static final int BLUE_GOAL_ID = 20;
    private static final double BLUE_GOAL_X_MM = 1789.3;
    private static final double BLUE_GOAL_Y_MM = 2.7;

    private static final int RED_GOAL_ID = 24;
    private static final double RED_GOAL_X_MM = 1783.9;
    private static final double RED_GOAL_Y_MM = -31.1;

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

    // ===== Pinpoint offsets (MUST match your working sample) =====
    private static final double PINPOINT_X_OFFSET_MM = 82.55;
    private static final double PINPOINT_Y_OFFSET_MM = -95.25;

    // ===== Pinpoint encoder directions (MUST match your working sample) =====
    private static final GoBildaPinpointDriver.EncoderDirection X_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection Y_DIR =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;

    // ===== Path C control tuning =====
    private static double KP_ERR_TO_TURRET_DEG = 0.12;
    private static double DEADBAND_DEG = 1.0;
    private static double SERVO_SLEW_PER_SEC = 0.85;

    // Flip if turret turns wrong direction (keep what WORKS for you)
    private static boolean INVERT_TURRET_RESPONSE = false;

    // ===== Limelight Trim tuning (anti-jitter) =====
    private static boolean LL_TRIM_ENABLED_DEFAULT = true;

    private static final double LL_DEADBAND_DEG = 1.0;
    private static final double LL_ALPHA = 0.20;
    private static final double LL_TRIM_KI = 0.035;
    private static final double LL_TRIM_MAX_DEG = 12.0;
    private static final double LL_TRIM_SLEW_DEG_PER_SEC = 40.0;

    private static final boolean INVERT_LL_BEARING = false;

    // ===== Internal =====
    private Servo left, right;
    private GoBildaPinpointDriver odo;
    private Limelight3A limelight;

    private final ElapsedTime loopTimer = new ElapsedTime();

    private double turretCmd = HOME_POS;
    private double turretRelDeg = 0.0;

    private boolean autoAimEnabled = true;
    private boolean llTrimEnabled = LL_TRIM_ENABLED_DEFAULT;

    // Limelight state
    private boolean tagSeen = false;
    private double bearingRawDeg = Double.NaN;
    private double bearingFiltDeg = 0.0;
    private double llTrimDeg = 0.0;

    // Edge detect
    private boolean prevRB=false, prevLB=false, prevY=false, prevX=false, prevA=false, prevB=false;
    private boolean prevDpadL=false, prevDpadR=false;

    // Telemetry
    private double robotXmm=0, robotYmm=0, robotHdeg=0;
    private double bearingFieldDeg=0;
    private double turretTargetRelDeg=0;
    private double errDeg=0;

    @Override
    public void init() {
        left  = hardwareMap.get(Servo.class, LEFT_SERVO_NAME);
        right = hardwareMap.get(Servo.class, RIGHT_SERVO_NAME);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);

        // ===== Pinpoint init EXACTLY like your sample =====
        odo.setOffsets(PINPOINT_X_OFFSET_MM, PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(X_DIR, Y_DIR);
        odo.resetPosAndIMU(); // robot MUST be stationary

        // Limelight
        limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
        limelight.start();

        // Alliance defaults
        applyAlliance(selectedAlliance);

        // Start turret at home
        turretCmd = clamp(HOME_POS, SAFE_MIN, SAFE_MAX);
        turretRelDeg = 0.0;
        applyTurret(turretCmd);

        loopTimer.reset();

        telemetry.addLine("TurretAutoAim_PathC_LLTrim (ALLIANCE SELECT in INIT)");
        telemetry.addLine("DPAD LEFT = BLUE | DPAD RIGHT = RED (choose BEFORE START)");
        telemetry.addLine("RB = toggle AutoAim | LB = toggle LL Trim");
        telemetry.addLine("Y  = turret HOME + clear LL trim");
        telemetry.addLine("X/A= resetPosAndIMU (robot still) + clear LL trim");
        telemetry.addLine("B  = recalibrateIMU (no pos reset)");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Select alliance BEFORE START (top-team pattern)
        boolean dpadL = gamepad1.dpad_left;
        boolean dpadR = gamepad1.dpad_right;

        boolean dpadLPressed = dpadL && !prevDpadL;
        boolean dpadRPressed = dpadR && !prevDpadR;

        if (dpadLPressed) {
            selectedAlliance = Alliance.BLUE;
            applyAlliance(selectedAlliance);
            clearLimelightTrimAndFilter();
        }
        if (dpadRPressed) {
            selectedAlliance = Alliance.RED;
            applyAlliance(selectedAlliance);
            clearLimelightTrimAndFilter();
        }

        telemetry.addLine("=== INIT (choose alliance) ===");
        telemetry.addData("SelectedAlliance", selectedAlliance);
        telemetry.addData("ACTIVE_GOAL_ID", ACTIVE_GOAL_ID);
        telemetry.addData("GOAL_X_MM", "%.1f", GOAL_X_MM);
        telemetry.addData("GOAL_Y_MM", "%.1f", GOAL_Y_MM);
        telemetry.addLine("Press START when correct alliance is selected.");
        telemetry.update();

        prevDpadL = dpadL;
        prevDpadR = dpadR;
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        dt = clamp(dt, 0.005, 0.050);

        boolean rb = gamepad1.right_bumper;
        boolean lb = gamepad1.left_bumper;
        boolean y  = gamepad1.y;
        boolean x  = gamepad1.x;
        boolean a  = gamepad1.a;
        boolean b  = gamepad1.b;

        boolean rbPressed = rb && !prevRB;
        boolean lbPressed = lb && !prevLB;
        boolean yPressed  = y  && !prevY;
        boolean xPressed  = x  && !prevX;
        boolean aPressed  = a  && !prevA;
        boolean bPressed  = b  && !prevB;

        if (rbPressed) autoAimEnabled = !autoAimEnabled;
        if (lbPressed) llTrimEnabled = !llTrimEnabled;

        // ===== Pinpoint update =====
        odo.update();

        // X or A = resetPosAndIMU
        if (xPressed || aPressed) {
            odo.resetPosAndIMU(); // robot MUST be still
            turretCmd = clamp(HOME_POS, SAFE_MIN, SAFE_MAX);
            turretRelDeg = 0.0;

            clearLimelightTrimAndFilter();
        }

        // B = recalibrate IMU only
        if (bPressed) {
            odo.recalibrateIMU();
        }

        // Y = force turret home and clear trim
        if (yPressed) {
            turretCmd = clamp(HOME_POS, SAFE_MIN, SAFE_MAX);
            turretRelDeg = 0.0;

            clearLimelightTrimAndFilter();
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

        // ===== Compute field bearing robot->GOAL (alliance locked) =====
        double dx = GOAL_X_MM - robotXmm;
        double dy = GOAL_Y_MM - robotYmm;
        bearingFieldDeg = Math.toDegrees(Math.atan2(dy, dx));

        // ===== Update Limelight bearing-to-tag (ALLIANCE locked tag) =====
        updateBearingFromActiveTagPose();

        // Filter bearing ONLY when tag is seen
        if (tagSeen && !Double.isNaN(bearingRawDeg)) {
            bearingFiltDeg = bearingFiltDeg * (1.0 - LL_ALPHA) + bearingRawDeg * LL_ALPHA;
        }

        // ===== Limelight TRIM (slow bias) =====
        if (llTrimEnabled && tagSeen && !Double.isNaN(bearingRawDeg)) {
            double llErrDeg = bearingFiltDeg; // want 0
            if (INVERT_LL_BEARING) llErrDeg = -llErrDeg;

            if (Math.abs(llErrDeg) < LL_DEADBAND_DEG) llErrDeg = 0.0;

            double desiredTrim = llTrimDeg + (-LL_TRIM_KI * llErrDeg);
            desiredTrim = clamp(desiredTrim, -LL_TRIM_MAX_DEG, +LL_TRIM_MAX_DEG);

            llTrimDeg = slewTo(llTrimDeg, desiredTrim, LL_TRIM_SLEW_DEG_PER_SEC, dt);
        }

        // ===== Turret target relative to robot forward (Pinpoint + trim) =====
        turretTargetRelDeg = wrapDeg(bearingFieldDeg - robotHdeg + llTrimDeg);

        // Error between current turretRelDeg and desired
        errDeg = wrapDeg(turretTargetRelDeg - turretRelDeg);

        if (Math.abs(errDeg) < DEADBAND_DEG) errDeg = 0.0;

        // ===== AutoAim update =====
        if (autoAimEnabled) {
            double stepDeg = KP_ERR_TO_TURRET_DEG * errDeg;
            if (INVERT_TURRET_RESPONSE) stepDeg = -stepDeg;

            turretRelDeg = wrapDeg(turretRelDeg + stepDeg);

            double desiredCmd = HOME_POS + (turretRelDeg * POS_PER_DEG);
            desiredCmd = clamp(desiredCmd, SAFE_MIN, SAFE_MAX);

            turretCmd = slewTo(turretCmd, desiredCmd, SERVO_SLEW_PER_SEC, dt);
            turretCmd = clamp(turretCmd, SAFE_MIN, SAFE_MAX);

            turretRelDeg = (turretCmd - HOME_POS) / POS_PER_DEG;
            turretRelDeg = wrapDeg(turretRelDeg);
        }

        applyTurret(turretCmd);

        // ===== Telemetry =====
        String posStr = String.format(Locale.US, "{X: %.1f, Y: %.1f, H: %.2f}",
                robotXmm, robotYmm, robotHdeg);

        telemetry.addLine("=== Path C + Limelight Trim (ALLIANCE LOCKED) ===");
        telemetry.addData("Alliance", selectedAlliance);
        telemetry.addData("ACTIVE_GOAL_ID", ACTIVE_GOAL_ID);
        telemetry.addData("AutoAim (RB)", autoAimEnabled);
        telemetry.addData("LL Trim (LB)", llTrimEnabled);

        telemetry.addData("Pinpoint Status", odo.getDeviceStatus());
        telemetry.addData("Robot Pose (mm,deg)", posStr);

        telemetry.addData("Goal (mm)", "X=%.1f Y=%.1f", GOAL_X_MM, GOAL_Y_MM);
        telemetry.addData("bearingFieldDeg", "%.2f", bearingFieldDeg);

        telemetry.addData("LL tagSeen", tagSeen);
        telemetry.addData("LL bearingRawDeg", Double.isNaN(bearingRawDeg) ? "N/A" : String.format("%.2f", bearingRawDeg));
        telemetry.addData("LL bearingFiltDeg", "%.2f", bearingFiltDeg);
        telemetry.addData("llTrimDeg", "%.2f", llTrimDeg);

        telemetry.addData("turretTargetRelDeg", "%.2f", turretTargetRelDeg);
        telemetry.addData("turretRelDeg(est)", "%.2f", turretRelDeg);
        telemetry.addData("errDeg", "%.2f", errDeg);

        telemetry.addData("turretCmd", "%.3f", turretCmd);
        telemetry.addData("SAFE[min..max]", "%.2f..%.2f", SAFE_MIN, SAFE_MAX);

        telemetry.addLine("INIT: DPAD L=BLUE, R=RED (before START)");
        telemetry.addLine("Controls: RB auto | LB trim | Y home+clear | X/A reset+clear | B recalIMU");
        telemetry.update();

        prevRB=rb; prevLB=lb; prevY=y; prevX=x; prevA=a; prevB=b;
    }

    private void applyAlliance(Alliance a) {
        if (a == Alliance.BLUE) {
            ACTIVE_GOAL_ID = BLUE_GOAL_ID;
            GOAL_X_MM = BLUE_GOAL_X_MM;
            GOAL_Y_MM = BLUE_GOAL_Y_MM;
        } else {
            ACTIVE_GOAL_ID = RED_GOAL_ID;
            GOAL_X_MM = RED_GOAL_X_MM;
            GOAL_Y_MM = RED_GOAL_Y_MM;
        }
    }

    private void clearLimelightTrimAndFilter() {
        llTrimDeg = 0.0;
        bearingFiltDeg = 0.0;
        bearingRawDeg = Double.NaN;
        tagSeen = false;
    }

    /**
     * ALLIANCE-LOCKED: compute bearing to ACTIVE_GOAL_ID from 3D pose.
     * bearingDeg = atan2(x, z) (KEEP SIGN ON x, abs z)
     * NO FALLBACKS. If the active tag isn't present, tagSeen stays false.
     */
    private void updateBearingFromActiveTagPose() {
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
        if (chosen == null) return; // HARD REJECT wrong tags

        Pose3D pose;
        try {
            pose = chosen.getCameraPoseTargetSpace();
        } catch (Exception e) {
            return;
        }
        if (pose == null) return;

        Position p = pose.getPosition();

        double xMm = DistanceUnit.MM.fromUnit(p.unit, p.x);           // KEEP SIGN
        double zMm = Math.abs(DistanceUnit.MM.fromUnit(p.unit, p.z)); // abs Z
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

    private double wrapDeg(double deg) {
        while (deg >= 180.0) deg -= 360.0;
        while (deg < -180.0) deg += 360.0;
        return deg;
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
