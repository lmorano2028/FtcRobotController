package org.firstinspires.ftc.teamcode.turretTesting;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.List;

import org.firstinspires.ftc.teamcode.pinpoint.GoBildaPinpointDriver;

// >>> ONLY NEW IMPORT (PoseStorage)
import org.firstinspires.ftc.teamcode.PoseStorage;

@TeleOp(name="TurretAimPinpoint_LL", group="Test")
public class TurretAimPinpoint_LL extends LinearOpMode {

    // =========================
    // Hardware names
    // =========================
    private static final String PINPOINT_NAME = "pinpoint";
    private static final String LIMELIGHT_NAME = "limelight";
    private static final String SERVO_L_NAME = "LRotation";
    private static final String SERVO_R_NAME = "RRotation";

    // =========================
    // Field units
    // =========================
    private static final double MM_PER_IN = 25.4;

    // =========================
    // Goal locations (INCHES) - DECODE field coords
    // =========================
    private static final double RED_GOAL_X  = -58.3727;
    private static final double RED_GOAL_Y  = +55.6425;

    private static final double BLUE_GOAL_X = -58.3727;
    private static final double BLUE_GOAL_Y = -55.6425;

    private static final int RED_TAG_ID  = 24;
    private static final int BLUE_TAG_ID = 20;

    // =========================
    // Turret servo mapping
    // =========================
    private static final double TURRET_HOME    = 0.50;      // home (forward)
    private static final double POS_PER_DEG    = -0.00689;  // +deg -> LEFT, -deg -> RIGHT (FIXED SIGN)
    private static final double SERVO_MIN_SAFE = 0.10;
    private static final double SERVO_MAX_SAFE = 0.90;

    // =========================
    // Camera offset on robot (INCHES) - Level 1 for now
    // Forward of robot center: +4.5 in
    // Right of robot center:   0 in
    // =========================
    private static final double CAM_FWD_IN  = +4.5;
    private static final double CAM_RIGHT_IN = 0.0;

    // =========================
    // Pinpoint pod offsets (MM) for your goBILDA 4-bar odometry pack
    // X pod offset: +82.55mm (left of tracking point)
    // Y pod offset: -95.25mm (behind tracking point)
    // =========================
    private static final double POD_X_OFFSET_MM = 82.55;
    private static final double POD_Y_OFFSET_MM = -95.25;

    // =========================
    // Starting pose requirement (your request)
    // FTC origin (0,0,0) and bot pointing -X (toward obelisk)
    // Interpreting field heading:
    // 0 rad = +X, so pointing -X = 180 degrees (PI radians)
    // =========================
    private static final double START_X_IN = 0.0;
    private static final double START_Y_IN = 0.0;
    private static final double START_HEADING_DEG = 180.0;

    // =========================
    // HEADING OFFSET (NEW)
    // You observed: when bot is physically pointing -X, Pinpoint heading reads 0.
    // Therefore, Pinpoint heading must be shifted by +PI to align to field-heading convention.
    // =========================
    private static final double HEADING_OFFSET_RAD = Math.PI;

    // =========================
    // Tag trim logic (Option B: stable + hold last-good)
    // =========================
    private static final int TAG_STABLE_MS     = 120;   // must see required tag for this long to update trim
    private static final int TAG_LOST_HOLD_MS  = 800;   // keep last trim for this long after losing tag
    private static final double TRIM_MAX_DEG   = 12.0;  // clamp trim
    private static final double TRIM_ALPHA     = 0.25;  // smoothing for trim updates (0..1)

    // If trim pulls turret the wrong way, flip this between +1 and -1.
    private static final double LL_X_SIGN      = +1.0;

    // =========================
    // Turret slew-rate limiting (prevents snapping to 0.1/0.9)
    // (kept)
    // =========================
    private static final double SERVO_SLEW_PER_SEC = 1.2; // servo-position units per second
    private double lastTurretCmd = TURRET_HOME;

    // =========================
    // >>> PathA-style smooth controller (NEW)
    // - deadband
    // - proportional step
    // - slew + max step per loop
    // =========================
    private static final double KP_DEG_TO_POS = 0.0020;     // servo-units per degree (start small)
    private static final double BEARING_DEADBAND_DEG = 1.2; // degrees
    private static final double MAX_STEP_PER_LOOP = 0.020;  // safety clamp per loop
    private static final boolean INVERT_TURRET_RESPONSE = false; // flip if needed

    // =========================
    // Devices
    // =========================
    private GoBildaPinpointDriver odo;
    private Limelight3A limelight;
    private Servo turretL, turretR;

    // =========================
    // Alliance + required tag
    // =========================
    private boolean allianceIsRed = true;
    private int requiredTagId = RED_TAG_ID;

    // =========================
    // Pinpoint pose cache (INCHES + RADIANS)
    // =========================
    private double robotX_in = 0.0;        // Pinpoint X (forward)
    private double robotY_in = 0.0;        // Pinpoint Y (FTC field +Y)
    private double robotHeading_rad = 0.0; // CCW+ (you confirmed)

    // =========================
    // Trim state
    // =========================
    private boolean tagCurrentlySeen = false;
    private boolean tagStable = false;

    private double filteredTrimDeg = 0.0;   // smoothed trim
    private double lastGoodTrimDeg = 0.0;   // held trim

    private final ElapsedTime tagStableTimer = new ElapsedTime();
    private final ElapsedTime tagSeenTimer   = new ElapsedTime();
    private final ElapsedTime loopTimer      = new ElapsedTime();

    // =========================
    // >>> PoseStorage / Localization gate (NEW)
    // If PoseStorage.valid -> seed pinpoint from it and allow auto-aim normally.
    // If not valid -> turret stays HOME and auto-aim stays disabled until user confirms
    // after seeing an AprilTag (your Limelight fiducial) stably.
    // =========================
    private boolean poseSeededFromStorage = false;
    private boolean localizationConfirmed = false;  // user-confirmed after tag stable
    private boolean prevY = false;

    @Override
    public void runOpMode() {

        odo = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);

        turretL = hardwareMap.get(Servo.class, SERVO_L_NAME);
        turretR = hardwareMap.get(Servo.class, SERVO_R_NAME);

        // =========================
        // Pinpoint configuration
        // 1) Offsets
        // 2) Pod type
        // 3) Encoder directions
        // =========================
        odo.setOffsets(POD_X_OFFSET_MM, POD_Y_OFFSET_MM, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Keep exactly as provided:
        // X = FORWARD, Y = REVERSED
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        // Start devices
        odo.resetPosAndIMU();
        limelight.start();

        // =========================
        // >>> Seed pose from PoseStorage if valid (NEW)
        // Otherwise seed to your known safe default (0,0,180) but keep turret HOME
        // and require confirmation after tag localization.
        // =========================
        Pose2D seedPose;
        if (PoseStorage.valid) {
            seedPose = PoseStorage.readAsPose2D(); // mm + deg
            poseSeededFromStorage = true;
            localizationConfirmed = true; // pose is trusted (handoff from auton)
            // Optional safety: clear so stale pose cannot be reused later
            PoseStorage.clear();
        } else {
            // default: (0,0,180deg) in INCHES for your current convention
            seedPose = new Pose2D(
                    DistanceUnit.INCH, START_X_IN, START_Y_IN,
                    AngleUnit.DEGREES, START_HEADING_DEG
            );
            poseSeededFromStorage = false;
            localizationConfirmed = false; // must confirm with tag
        }

        // IMPORTANT: setPosition accepts Pose2D units; keep seedPose units as-is
        odo.setPosition(seedPose);

        // Put turret at home during init
        lastTurretCmd = TURRET_HOME;
        setTurretCmd(TURRET_HOME);

        // ===== INIT LOOP: alliance selection =====
        while (!isStarted() && !isStopRequested()) {

            if (gamepad1.dpad_left)  allianceIsRed = true;
            if (gamepad1.dpad_right) allianceIsRed = false;

            requiredTagId = allianceIsRed ? RED_TAG_ID : BLUE_TAG_ID;

            telemetry.addLine("INIT: Select Alliance");
            telemetry.addLine("  dpad_left  = RED  (Tag 24)");
            telemetry.addLine("  dpad_right = BLUE (Tag 20)");
            telemetry.addData("Alliance", allianceIsRed ? "RED" : "BLUE");
            telemetry.addData("Required Tag", requiredTagId);

            telemetry.addLine("");
            telemetry.addLine("Pose seed:");
            telemetry.addData("PoseStorage.valid", PoseStorage.valid);
            telemetry.addData("SeededFromStorage", poseSeededFromStorage);
            telemetry.addData("SeedPose",
                    "X=%.1fmm Y=%.1fmm H=%.1fdeg",
                    seedPose.getX(DistanceUnit.MM),
                    seedPose.getY(DistanceUnit.MM),
                    seedPose.getHeading(AngleUnit.DEGREES));

            telemetry.addLine("");
            telemetry.addLine("Heading alignment:");
            telemetry.addData("HEADING_OFFSET(deg)", "%.1f", Math.toDegrees(HEADING_OFFSET_RAD));
            telemetry.addLine("");

            telemetry.addLine("Controls after START:");
            telemetry.addLine("  A = reset pinpoint pos+IMU AND re-apply start pose");
            telemetry.addLine("  X = toggle turret auto-aim ON/OFF (optional)");
            telemetry.addLine("  Y = CONFIRM localization (requires tag stable) when PoseStorage not used");
            telemetry.update();

            sleep(20);
        }

        waitForStart();

        boolean autoAimEnabled = true;
        boolean prevX = false;

        // >>> If PoseStorage wasn't used, default behavior: stay HOME until confirmed (NEW)
        if (!poseSeededFromStorage) {
            autoAimEnabled = false;     // turret will not chase targets
            lastTurretCmd = TURRET_HOME;
            setTurretCmd(TURRET_HOME);
        }

        tagStableTimer.reset();
        tagSeenTimer.reset();
        loopTimer.reset();

        while (opModeIsActive()) {

            double dt = loopTimer.seconds();
            loopTimer.reset();
            dt = clamp(dt, 0.005, 0.050);

            // Toggle auto-aim
            boolean x = gamepad1.x;
            if (x && !prevX) autoAimEnabled = !autoAimEnabled;
            prevX = x;

            // Reset pinpoint
            if (gamepad1.a) {
                odo.resetPosAndIMU(); // robot must be stationary
                odo.setPosition(new Pose2D(
                        DistanceUnit.INCH, START_X_IN, START_Y_IN,
                        AngleUnit.DEGREES, START_HEADING_DEG
                ));
                // If you reset during match, treat localization as not confirmed until tag confirm (only if no storage seed)
                if (!poseSeededFromStorage) {
                    localizationConfirmed = false;
                    autoAimEnabled = false;
                    lastTurretCmd = TURRET_HOME;
                    setTurretCmd(TURRET_HOME);
                }
            }

            // 1) Update pinpoint
            odo.update();
            readPinpointPoseInches_Unnormalized();

            // 2) Update limelight trim (stable + hold)
            updateTrimFromLimelight();

            // >>> If PoseStorage not used: allow user to confirm only when tag is stable (NEW)
            boolean y = gamepad1.y;
            if (y && !prevY) {
                if (!poseSeededFromStorage) {
                    if (tagStable) {
                        localizationConfirmed = true;
                        autoAimEnabled = true; // enable aiming once confirmed
                    }
                }
            }
            prevY = y;

            // 3) Desired turret angle (deg) from odometry + trim
            double relDeg = computeRelativeGoalBearingDeg(); // now uses heading offset
            double desiredTurretDeg = relDeg + lastGoodTrimDeg;

            // 4) PathA-style incremental control
            double appliedCmd = lastTurretCmd;

            // >>> If PoseStorage not used and not confirmed: HOLD HOME (NEW)
            if (!poseSeededFromStorage && !localizationConfirmed) {
                appliedCmd = TURRET_HOME;
            } else if (autoAimEnabled) {

                // Current turret angle estimate from servo command
                double currentTurretDeg = (lastTurretCmd - TURRET_HOME) / POS_PER_DEG;

                // Error (wrap to [-180, +180])
                double errDeg = wrapDeg(desiredTurretDeg - currentTurretDeg);

                // deadband
                if (Math.abs(errDeg) < BEARING_DEADBAND_DEG) errDeg = 0.0;

                // Convert error deg -> servo delta (PathA)
                double deltaPos = errDeg * KP_DEG_TO_POS;
                if (INVERT_TURRET_RESPONSE) deltaPos = -deltaPos;

                // Move opposite the error to drive it toward zero (PathA)  <-- FIXED
                double desiredCmd = lastTurretCmd - deltaPos;

                // clamp desired
                desiredCmd = clamp(desiredCmd, SERVO_MIN_SAFE, SERVO_MAX_SAFE);

                // slew limit
                appliedCmd = slewTo(lastTurretCmd, desiredCmd, SERVO_SLEW_PER_SEC, dt);

                // extra per-loop clamp
                appliedCmd = clamp(appliedCmd,
                        Math.max(SERVO_MIN_SAFE, lastTurretCmd - MAX_STEP_PER_LOOP),
                        Math.min(SERVO_MAX_SAFE, lastTurretCmd + MAX_STEP_PER_LOOP));

                // If at limits, hold there
                if (appliedCmd <= SERVO_MIN_SAFE + 1e-3) appliedCmd = SERVO_MIN_SAFE;
                if (appliedCmd >= SERVO_MAX_SAFE - 1e-3) appliedCmd = SERVO_MAX_SAFE;
            }

            // Apply
            lastTurretCmd = appliedCmd;
            setTurretCmd(lastTurretCmd);

            // ===================== TELEMETRY =====================
            telemetry.addLine("=== TurretAimPinpoint_LL (Heading Offset + PathA Smooth) ===");
            telemetry.addData("Alliance", allianceIsRed ? "RED" : "BLUE");
            telemetry.addData("RequiredTag", requiredTagId);

            telemetry.addData("PoseSeededFromStorage", poseSeededFromStorage);
            telemetry.addData("LocalizationConfirmed", localizationConfirmed);
            telemetry.addData("AutoAim", autoAimEnabled);

            telemetry.addData("Pinpoint X(in)", "%.2f", robotX_in);
            telemetry.addData("Pinpoint Y(in)", "%.2f", robotY_in);
            telemetry.addData("Pinpoint Heading(deg, unnorm)", "%.1f", Math.toDegrees(robotHeading_rad));

            double headingFieldDeg = Math.toDegrees(wrapRad(robotHeading_rad + HEADING_OFFSET_RAD));
            telemetry.addData("HeadingField(deg)", "%.1f", headingFieldDeg);

            double gx = allianceIsRed ? RED_GOAL_X : BLUE_GOAL_X;
            double gy = allianceIsRed ? RED_GOAL_Y : BLUE_GOAL_Y;
            telemetry.addData("Goal (x,y)", "(%.2f, %.2f)", gx, gy);

            telemetry.addData("TagSeenNow?", tagCurrentlySeen);
            telemetry.addData("TagStable?", tagStable);
            telemetry.addData("TrimDeg(hold)", "%.2f", lastGoodTrimDeg);

            telemetry.addData("relDeg(odo)", "%.2f", relDeg);
            telemetry.addData("desiredDeg(odo+trim)", "%.2f", desiredTurretDeg);

            double currentTurretDegTele = (lastTurretCmd - TURRET_HOME) / POS_PER_DEG;
            telemetry.addData("turretDeg(est)", "%.2f", currentTurretDegTele);
            telemetry.addData("turretCmd(applied)", "%.3f", lastTurretCmd);

            if (!poseSeededFromStorage && !localizationConfirmed) {
                telemetry.addLine("HOLDING HOME: Press Y to confirm (requires TagStable=true)");
            }

            telemetry.update();

            sleep(20);
        }

        limelight.stop();
    }

    // =========================
    // Pinpoint read helpers
    // =========================
    private void readPinpointPoseInches_Unnormalized() {
        robotX_in = odo.getPosX(DistanceUnit.INCH);

        // FIX: FTC field convention (with your -X starting direction):
        // pushing LEFT should make Y NEGATIVE, pushing RIGHT should make Y POSITIVE.
        // Therefore: do NOT negate here; keep Pinpoint's Y as the field Y.
        robotY_in = odo.getPosY(DistanceUnit.INCH);

        robotHeading_rad = odo.getHeading(UnnormalizedAngleUnit.RADIANS);
    }

    // =========================
    // Compute relative bearing to goal from odometry
    // FIX: apply heading offset so Pinpoint heading is in FIELD frame
    // =========================
    private double computeRelativeGoalBearingDeg() {
        double goalX = allianceIsRed ? RED_GOAL_X : BLUE_GOAL_X;
        double goalY = allianceIsRed ? RED_GOAL_Y : BLUE_GOAL_Y;

        double dx = goalX - robotX_in;
        double dy = goalY - robotY_in;

        double goalBearing_rad = Math.atan2(dy, dx);

        // Apply heading offset (NEW)
        double headingField_rad = wrapRad(robotHeading_rad + HEADING_OFFSET_RAD);

        double rel_rad = wrapRad(goalBearing_rad - headingField_rad);
        return Math.toDegrees(rel_rad);
    }

    // =========================
    // Limelight trim (Option B)
    // (unchanged)
    // =========================
    private void updateTrimFromLimelight() {
        boolean seenThisLoop = false;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            tagCurrentlySeen = false;
            handleTagNotSeen();
            return;
        }

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) {
            tagCurrentlySeen = false;
            handleTagNotSeen();
            return;
        }

        LLResultTypes.FiducialResult chosen = null;
        for (LLResultTypes.FiducialResult t : tags) {
            if ((int)t.getFiducialId() == requiredTagId) {
                chosen = t;
                break;
            }
        }
        if (chosen == null) {
            tagCurrentlySeen = false;
            handleTagNotSeen();
            return;
        }

        Pose3D pose;
        try {
            pose = chosen.getCameraPoseTargetSpace();
        } catch (Exception e) {
            tagCurrentlySeen = false;
            handleTagNotSeen();
            return;
        }
        if (pose == null) {
            tagCurrentlySeen = false;
            handleTagNotSeen();
            return;
        }

        Position p = pose.getPosition();

        double xMm = DistanceUnit.MM.fromUnit(p.unit, p.x);
        double zMm = DistanceUnit.MM.fromUnit(p.unit, p.z);

        if (Math.abs(zMm) < 1e-6) {
            tagCurrentlySeen = false;
            handleTagNotSeen();
            return;
        }

        seenThisLoop = true;
        tagCurrentlySeen = true;

        double trimRad = Math.atan2(LL_X_SIGN * xMm, Math.abs(zMm));
        double trimDeg = Math.toDegrees(trimRad);
        trimDeg = clamp(trimDeg, -TRIM_MAX_DEG, +TRIM_MAX_DEG);

        if (seenThisLoop) {
            if (!tagStableTimerStarted()) {
                tagStableTimer.reset();
            }
            if (tagStableTimer.milliseconds() >= TAG_STABLE_MS) {
                tagStable = true;
            }
            tagSeenTimer.reset();
        }

        if (tagStable) {
            filteredTrimDeg = filteredTrimDeg + TRIM_ALPHA * (trimDeg - filteredTrimDeg);
            lastGoodTrimDeg = filteredTrimDeg;
        }
    }

    private boolean tagStableTimerStarted() {
        return tagStableTimer.seconds() > 0.0001;
    }

    private void handleTagNotSeen() {
        tagStableTimer.reset();

        if (tagSeenTimer.milliseconds() > TAG_LOST_HOLD_MS) {
            tagStable = false;
            filteredTrimDeg *= 0.98;
            lastGoodTrimDeg = filteredTrimDeg;
        }
    }

    // =========================
    // Servo write helper
    // =========================
    private void setTurretCmd(double cmd) {
        cmd = clamp(cmd, SERVO_MIN_SAFE, SERVO_MAX_SAFE);
        turretL.setPosition(cmd);
        turretR.setPosition(cmd);
    }

    // =========================
    // Slew limiter (PathA style)
    // =========================
    private double slewTo(double current, double target, double ratePerSec, double dt) {
        double maxStep = ratePerSec * dt;
        double delta = target - current;
        if (delta >  maxStep) delta =  maxStep;
        if (delta < -maxStep) delta = -maxStep;
        return current + delta;
    }

    // =========================
    // Math helpers
    // =========================
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double wrapRad(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    private static double wrapDeg(double d) {
        while (d > 180.0) d -= 360.0;
        while (d < -180.0) d += 360.0;
        return d;
    }
}
