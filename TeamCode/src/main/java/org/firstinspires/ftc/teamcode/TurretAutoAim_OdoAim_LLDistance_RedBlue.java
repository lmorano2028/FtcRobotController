package org.firstinspires.ftc.teamcode;

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

@TeleOp(name="TurretAutoAim_OdoAim_LLDistance_RedBlue", group="Test")
public class TurretAutoAim_OdoAim_LLDistance_RedBlue extends OpMode {

    // ===== Hardware names =====
    private static final String LEFT_SERVO_NAME  = "LRotation";
    private static final String RIGHT_SERVO_NAME = "RRotation";
    private static final String PINPOINT_NAME    = "pinpoint";
    private static final String LIMELIGHT_NAME   = "limelight";

    // ===== Limelight =====
    private static final int APRILTAG_PIPELINE_INDEX = 0;

    // DECODE goal tags (your IDs)
    private static final int BLUE_GOAL_ID = 20;
    private static final int RED_GOAL_ID  = 24;

    // ===== Alliance selection (INIT) =====
    private enum Alliance { BLUE, RED }
    private Alliance selectedAlliance = Alliance.BLUE;

    // ===== Field goal positions (mm) =====
    // NOTE: These are YOUR measured “push to wall” values.
    private static final double BLUE_GOAL_X_MM = 1789.3;
    private static final double BLUE_GOAL_Y_MM = 2.7;

    private static final double RED_GOAL_X_MM  = 1783.9;
    private static final double RED_GOAL_Y_MM  = -31.1;

    // We will switch these based on alliance:
    private int ACTIVE_GOAL_ID = BLUE_GOAL_ID;
    private double GOAL_X_MM = BLUE_GOAL_X_MM;
    private double GOAL_Y_MM = BLUE_GOAL_Y_MM;

    // ===== Your turret limits =====
    private static final double SAFE_MIN = 0.10;
    private static final double SAFE_MAX = 0.90;
    private static final double HOME_POS = 0.50;

    // Right servo mirror
    private static final boolean RIGHT_MIRROR_IN_CODE = false;

    // ===== Your measured turret conversion =====
    // +0.10 servo = +14.5 deg to the RIGHT (you said 0.6 from 0.5 turns right)
    // pos_per_deg magnitude = 0.10 / 14.5 = 0.00689
    // SIGN: if +deg should increase servoCmd, then POS_PER_DEG should be +0.00689
    // You previously had negative; but your latest says 0.6 makes turret go RIGHT (increase cmd).
    // So: turretDegRightPositive => servo increases => POS_PER_DEG = +0.00689
    private static final double POS_PER_DEG = +0.00689;

    // ===== Pinpoint settings (match your sample) =====
    private static final double PINPOINT_X_OFFSET_MM = 82.55;
    private static final double PINPOINT_Y_OFFSET_MM = -95.25;

    private static final GoBildaPinpointDriver.EncoderDirection X_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection Y_DIR =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;

    // ===== Heading convention =====
    // You want: BLUE facing = 0 deg
    // RED facing = +270 deg (same as -90, but we’ll display wrapped)
    private static final double BLUE_FACE_DEG = 0.0;
    private static final double RED_FACE_DEG  = 270.0; // will wrap internally to -90

    // ===== Odometry turret control tuning =====
    private static double KP_ERR_TO_TURRET_DEG = 0.14; // slightly stronger than 0.12 to reduce lag on fast turns
    private static double DEADBAND_DEG = 0.8;
    private static double SERVO_SLEW_PER_SEC = 0.90;

    // If turret moves wrong direction, flip THIS (should not be needed with POS_PER_DEG sign fixed)
    private static boolean INVERT_TURRET_RESPONSE = false;

    // ===== Limelight distance =====
    // Camera offset from turret center = 8" forward (you said 8")
    private static final double CAM_FORWARD_OFFSET_MM = 8.0 * 25.4; // 203.2mm

    // If you want to use camera pose (recommended) for distance:
    // We compute pivot-centered Z by: zPivot = |camZ| + CAM_FORWARD_OFFSET_MM
    // Then planar distance pivot->tag = hypot(x, zPivot) (or full 3D includes y)
    private static final boolean USE_CAMERA_POSE_FOR_DISTANCE = true;

    // ===== Internal =====
    private Servo left, right;
    private GoBildaPinpointDriver odo;
    private Limelight3A limelight;

    private final ElapsedTime loopTimer = new ElapsedTime();

    private double turretCmd = HOME_POS;
    private double turretRelDeg = 0.0; // 0 corresponds to HOME

    private boolean autoAimEnabled = true;

    // Limelight state (distance only)
    private boolean tagSeen = false;
    private double camXmm = Double.NaN, camYmm = Double.NaN, camZmm = Double.NaN;
    private double distPivotPlanarMm = Double.NaN; // corrected for 8" offset
    private double distPivot3DMm = Double.NaN;

    // Pose
    private double robotXmm=0, robotYmm=0, robotHdeg=0;
    private double bearingFieldDeg=0;
    private double turretTargetRelDeg=0;
    private double errDeg=0;

    // Init selection edge detect
    private boolean prevDpadL=false, prevDpadR=false;
    private boolean prevRB=false, prevY=false, prevX=false, prevA=false, prevB=false;

    @Override
    public void init() {
        left  = hardwareMap.get(Servo.class, LEFT_SERVO_NAME);
        right = hardwareMap.get(Servo.class, RIGHT_SERVO_NAME);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);

        // Pinpoint init (DO NOT reset pose here; TeleOp must be able to start after Auto!)
        odo.setOffsets(PINPOINT_X_OFFSET_MM, PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(X_DIR, Y_DIR);

        // If Auto wrote a pose, load it. Otherwise, we do NOT force reset.
        if (PoseStorage.valid) {
            odo.setPosition(PoseStorage.readAsPose2D());
        }

        // Limelight init
        limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
        limelight.start();

        applyAlliance(selectedAlliance);

        // Turret start centered
        turretCmd = clamp(HOME_POS, SAFE_MIN, SAFE_MAX);
        turretRelDeg = 0.0;
        applyTurret(turretCmd);

        loopTimer.reset();

        telemetry.addLine("TurretAutoAim_OdoAim_LLDistance_RedBlue READY");
        telemetry.addLine("INIT: DPAD LEFT=BLUE, DPAD RIGHT=RED (choose BEFORE START)");
        telemetry.addLine("RB toggle auto-aim | Y turret HOME | X/A set Pinpoint pose to 'Alliance Facing'");
        telemetry.addLine("B recal IMU (still) | NOTE: TeleOp loads pose from Auto if PoseStorage.valid");
        telemetry.addData("PoseStorage.valid", PoseStorage.valid);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Alliance selection before start
        boolean dpadL = gamepad1.dpad_left;
        boolean dpadR = gamepad1.dpad_right;

        boolean dpadLPressed = dpadL && !prevDpadL;
        boolean dpadRPressed = dpadR && !prevDpadR;

        if (dpadLPressed) { selectedAlliance = Alliance.BLUE; applyAlliance(selectedAlliance); }
        if (dpadRPressed) { selectedAlliance = Alliance.RED;  applyAlliance(selectedAlliance); }

        telemetry.addLine("=== INIT ===");
        telemetry.addData("SelectedAlliance", selectedAlliance);
        telemetry.addData("ACTIVE_GOAL_ID", ACTIVE_GOAL_ID);
        telemetry.addData("Goal (mm)", "X=%.1f Y=%.1f", GOAL_X_MM, GOAL_Y_MM);
        telemetry.addData("PoseStorage.valid", PoseStorage.valid);
        telemetry.addLine("Press START when alliance is correct.");
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

        // Update Pinpoint
        odo.update();

        // X/A helper: set heading to your “alliance facing” convention WITHOUT moving X/Y.
        // This is a “dummies” tool: if you KNOW your robot is facing the goal direction, press it.
        if (xPressed || aPressed) {
            Pose2D p = odo.getPosition();
            double newH = (selectedAlliance == Alliance.BLUE) ? BLUE_FACE_DEG : RED_FACE_DEG;
            Pose2D updated = new Pose2D(
                    DistanceUnit.MM, p.getX(DistanceUnit.MM), p.getY(DistanceUnit.MM),
                    AngleUnit.DEGREES, newH
            );
            odo.setPosition(updated);
            PoseStorage.writeFromPose(updated); // keep memory consistent
        }

        // B: recal IMU (robot must be still)
        if (bPressed) odo.recalibrateIMU();

        // Y: turret home
        if (yPressed) {
            turretCmd = clamp(HOME_POS, SAFE_MIN, SAFE_MAX);
            turretRelDeg = 0.0;
        }

        // Read robot pose
        Pose2D pos = odo.getPosition();
        robotXmm = pos.getX(DistanceUnit.MM);
        robotYmm = pos.getY(DistanceUnit.MM);
        robotHdeg = pos.getHeading(AngleUnit.DEGREES);

        // Compute field bearing robot->goal
        double dx = GOAL_X_MM - robotXmm;
        double dy = GOAL_Y_MM - robotYmm;
        bearingFieldDeg = Math.toDegrees(Math.atan2(dy, dx));

        // Turret target relative to robot forward
        turretTargetRelDeg = wrapDeg(bearingFieldDeg - robotHdeg);

        // Error between current turretRelDeg estimate and desired
        errDeg = wrapDeg(turretTargetRelDeg - turretRelDeg);
        if (Math.abs(errDeg) < DEADBAND_DEG) errDeg = 0.0;

        // Auto-aim turret (ODOMETRY ONLY)
        if (autoAimEnabled) {
            double stepDeg = KP_ERR_TO_TURRET_DEG * errDeg;
            if (INVERT_TURRET_RESPONSE) stepDeg = -stepDeg;

            turretRelDeg = wrapDeg(turretRelDeg + stepDeg);

            double desiredCmd = HOME_POS + (turretRelDeg * POS_PER_DEG);
            desiredCmd = clamp(desiredCmd, SAFE_MIN, SAFE_MAX);

            turretCmd = slewTo(turretCmd, desiredCmd, SERVO_SLEW_PER_SEC, dt);
            turretCmd = clamp(turretCmd, SAFE_MIN, SAFE_MAX);

            // keep estimate consistent with clamps
            turretRelDeg = (turretCmd - HOME_POS) / POS_PER_DEG;
            turretRelDeg = wrapDeg(turretRelDeg);
        }

        applyTurret(turretCmd);

        // Limelight distance (does NOT control turret)
        updateDistanceFromActiveTag();

        // Telemetry
        String poseStr = String.format(Locale.US, "{X: %.1f, Y: %.1f, H: %.2f}", robotXmm, robotYmm, robotHdeg);

        telemetry.addLine("=== TurretAutoAim: ODO Aim + LL Distance ===");
        telemetry.addData("Alliance", selectedAlliance);
        telemetry.addData("ACTIVE_GOAL_ID", ACTIVE_GOAL_ID);
        telemetry.addData("AutoAim (RB)", autoAimEnabled);
        telemetry.addData("Robot Pose", poseStr);

        telemetry.addData("Goal (mm)", "X=%.1f Y=%.1f", GOAL_X_MM, GOAL_Y_MM);
        telemetry.addData("bearingFieldDeg", "%.2f", bearingFieldDeg);
        telemetry.addData("turretTargetRelDeg", "%.2f", turretTargetRelDeg);
        telemetry.addData("turretRelDeg(est)", "%.2f", turretRelDeg);
        telemetry.addData("errDeg", "%.2f", errDeg);
        telemetry.addData("turretCmd", "%.3f", turretCmd);

        telemetry.addLine("--- Limelight Distance (for shooter RPM) ---");
        telemetry.addData("tagSeen", tagSeen);
        telemetry.addData("camXmm", Double.isNaN(camXmm) ? "N/A" : String.format("%.1f", camXmm));
        telemetry.addData("camZmm", Double.isNaN(camZmm) ? "N/A" : String.format("%.1f", camZmm));
        telemetry.addData("CamOffsetForward(mm)", "%.1f", CAM_FORWARD_OFFSET_MM);
        telemetry.addData("PivotPlanarDist(mm)", Double.isNaN(distPivotPlanarMm) ? "N/A" : String.format("%.1f", distPivotPlanarMm));
        telemetry.addData("PivotPlanarDist(in)", Double.isNaN(distPivotPlanarMm) ? "N/A" : String.format("%.2f", distPivotPlanarMm/25.4));
        telemetry.addData("Pivot3DDist(mm)", Double.isNaN(distPivot3DMm) ? "N/A" : String.format("%.1f", distPivot3DMm));

        telemetry.addLine("INIT: DPAD L=BLUE / R=RED (before START)");
        telemetry.addLine("Controls: RB auto | Y home | X/A set heading to alliance-facing | B recalIMU(still)");
        telemetry.update();

        prevRB=rb; prevY=y; prevX=x; prevA=a; prevB=b;

        // Persist pose periodically so TeleOp->Auto->TeleOp cycles stay sane
        PoseStorage.writeFromPose(pos);
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

    /**
     * Uses Limelight pose to compute distance.
     * We correct for camera being 8" forward of turret pivot:
     *   zPivot = |camZ| + offset
     *   planarPivotDist = hypot(camX, zPivot)
     *   3D includes camY: sqrt(x^2 + y^2 + zPivot^2)
     */
    private void updateDistanceFromActiveTag() {
        tagSeen = false;
        camXmm = camYmm = camZmm = Double.NaN;
        distPivotPlanarMm = distPivot3DMm = Double.NaN;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return;

        LLResultTypes.FiducialResult chosen = null;
        for (LLResultTypes.FiducialResult t : tags) {
            if ((int)t.getFiducialId() == ACTIVE_GOAL_ID) { chosen = t; break; }
        }
        if (chosen == null) return;

        Pose3D camPose;
        try { camPose = chosen.getCameraPoseTargetSpace(); }
        catch (Exception e) { return; }
        if (camPose == null) return;

        Position p = camPose.getPosition();
        camXmm = DistanceUnit.MM.fromUnit(p.unit, p.x);
        camYmm = DistanceUnit.MM.fromUnit(p.unit, p.y);
        camZmm = DistanceUnit.MM.fromUnit(p.unit, p.z);

        double zCamAbs = Math.abs(camZmm);
        if (zCamAbs < 1.0) return;

        // pivot is behind camera by 8", so pivot->tag forward distance is larger:
        double zPivot = zCamAbs + CAM_FORWARD_OFFSET_MM;

        distPivotPlanarMm = Math.hypot(camXmm, zPivot);
        distPivot3DMm = Math.sqrt(camXmm*camXmm + camYmm*camYmm + zPivot*zPivot);

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

    // Wrap to [-180, +180)
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
