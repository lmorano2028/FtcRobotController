package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

public class ShooterSubsystem {

    // Limelight
    private final Limelight3A limelight;
    private final int aprilTagPipelineIndex;
    private final int blueGoalId;
    private final int redGoalId;

    // Shooter hardware
    private final DcMotorEx shooter;
    private final Servo hood;

    // PIDF (your values)
    private final double P;
    private final double F;

    // Distance offsets
    private final double CAMERA_TO_FLYWHEEL_IN;
    private static final double IN_PER_MM = 1.0 / 25.4;

    // Shotmap tables (your values)
    private final double[] DIST_IN;
    private final double[] HOOD_POS;
    private final double[] RPM_MIN;
    private final double[] RPM_TGT;
    private final double[] RPM_MAX;

    // Ready stable timing (your values)
    private final int READY_STABLE_MS;

    // Live outputs
    private boolean tagSeen = false;
    private int tagId = -1;
    private double shooterDistIn = Double.NaN;

    private double hoodCmd = 0.22;
    private double rpmMinCmd = 0, rpmTgtCmd = 0, rpmMaxCmd = 0;

    private boolean enabled = false;

    private final ElapsedTime readyStableTimer = new ElapsedTime();

    // =========================
    // HOLD-LAST-shot-solution (TeleOp behavior)
    // =========================
    private boolean haveLastShotSolution = false;
    private double lastHoodCmd = 0.22;
    private double lastRpmMinCmd = 0;
    private double lastRpmTgtCmd = 0;
    private double lastRpmMaxCmd = 0;

    // =========================
    // NEW: No movement in INIT + spinup behavior at START
    // =========================
    private static final double STARTUP_RPM = 1000.0;

    // We defer any hood movement until enabled (START)
    private boolean hoodInitialized = false;

    // Prefer setting velocity rather than setPower for stopping too (consistent mode)
    private void stopMotorVelocity() {
        shooter.setVelocity(0);
    }

    public ShooterSubsystem(
            Limelight3A limelight,
            DcMotorEx shooter,
            Servo hood,
            int aprilTagPipelineIndex,
            int blueGoalId,
            int redGoalId,
            double cameraToFlywheelIn,
            double P, double F,
            double[] distIn,
            double[] hoodPos,
            double[] rpmMin,
            double[] rpmTgt,
            double[] rpmMax,
            int readyStableMs
    ) {
        this.limelight = limelight;
        this.shooter = shooter;
        this.hood = hood;

        this.aprilTagPipelineIndex = aprilTagPipelineIndex;
        this.blueGoalId = blueGoalId;
        this.redGoalId = redGoalId;

        this.CAMERA_TO_FLYWHEEL_IN = cameraToFlywheelIn;

        this.P = P;
        this.F = F;

        this.DIST_IN = distIn;
        this.HOOD_POS = hoodPos;
        this.RPM_MIN = rpmMin;
        this.RPM_TGT = rpmTgt;
        this.RPM_MAX = rpmMax;

        this.READY_STABLE_MS = readyStableMs;

        PIDFCoefficients pidf = new PIDFCoefficients(this.P, 0, 0, this.F);
        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);

        // ===== IMPORTANT CHANGE =====
        // NO hood.setPosition(...) here (no servo movement in INIT).
        // NO stop() here (stop() could move motor in INIT).

        // Initialize state only (no hardware commands)
        enabled = false;
        hoodInitialized = false;

        tagSeen = false;
        tagId = -1;
        shooterDistIn = Double.NaN;

        haveLastShotSolution = false;
        lastHoodCmd = 0.22;
        lastRpmMinCmd = 0;
        lastRpmTgtCmd = 0;
        lastRpmMaxCmd = 0;

        hoodCmd = 0.22;
        rpmMinCmd = 0;
        rpmTgtCmd = 0;
        rpmMaxCmd = 0;
    }

    public void startVision() {
        limelight.pipelineSwitch(aprilTagPipelineIndex);
        limelight.start();
    }

    public void stopVision() {
        limelight.stop();
    }

    /**
     * Call this at START (your opmode start()).
     * - When enabled becomes true: immediately spin at 1000 RPM until first tag solution arrives.
     * - Also initializes hood to a safe default ONCE (at enable), so no INIT movement.
     */
    public void setEnabled(boolean on) {
        enabled = on;
        resetReadyStable();

        if (enabled) {
            // First time we enable: set hood to a safe default (deferred from INIT)
            if (!hoodInitialized) {
                double safeHood = (HOOD_POS != null && HOOD_POS.length > 0) ? HOOD_POS[0] : 0.22;
                hood.setPosition(safeHood);
                hoodInitialized = true;

                // Also initialize "last" solution to safe hood; RPM still 0 until tag solution
                lastHoodCmd = safeHood;
            }
        } else {
            // Disable: stop motor output
            stopMotorVelocity();
        }
    }

    public boolean isEnabled() { return enabled; }
    public boolean isTagSeen() { return tagSeen; }
    public int getTagId() { return tagId; }
    public double getShooterDistanceIn() { return shooterDistIn; }
    public double getHoodCmd() { return hoodCmd; }
    public double getRpmTargetCmd() { return rpmTgtCmd; }

    /** Call every loop in auton/teleop to refresh tag → distance → hood/RPM and command motor. */
    public void update() {
        // Always compute/refresh shot solution (but only moves hood if enabled)
        updateFromLimelightAndComputeShot_HoldLast();

        if (!enabled) {
            // If not enabled, do not command anything (including hood)
            return;
        }

        // If enabled but we don't have a tag solution yet: spin up immediately to startup RPM.
        if (!haveLastShotSolution || lastRpmTgtCmd <= 0) {
            shooter.setVelocity(STARTUP_RPM);
            return;
        }

        // Normal behavior: HOLD last solution if tag drops
        shooter.setVelocity(lastRpmTgtCmd);
    }

    public void stop() {
        // Keep for compatibility: stop motor output
        stopMotorVelocity();
    }

    public void resetReadyStable() {
        readyStableTimer.reset();
    }

    public boolean isShooterReadyStable() {
        // Use LAST known values even if tag is not seen now (TeleOp behavior)
        if (!enabled || !haveLastShotSolution || lastRpmTgtCmd <= 0) {
            resetReadyStable();
            return false;
        }

        double v = shooter.getVelocity();
        boolean inWindow = (v >= lastRpmMinCmd && v <= lastRpmMaxCmd);

        if (inWindow) {
            return readyStableTimer.milliseconds() >= READY_STABLE_MS;
        } else {
            resetReadyStable();
            return false;
        }
    }

    // ---------------- Internal Limelight math (TeleOp HOLD-LAST behavior) ----------------

    private void updateFromLimelightAndComputeShot_HoldLast() {
        // Always refresh tagSeen/tagId for telemetry/logic, but do NOT zero commands on a miss
        tagSeen = false;
        tagId = -1;
        shooterDistIn = Double.NaN;

        boolean gotNewSolution = false;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {

            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            if (tags != null && !tags.isEmpty()) {

                // Choose tag 20/24 if present, else first
                LLResultTypes.FiducialResult chosen = null;
                for (LLResultTypes.FiducialResult t : tags) {
                    int id = (int) t.getFiducialId();
                    if (id == blueGoalId || id == redGoalId) {
                        chosen = t;
                        break;
                    }
                }
                if (chosen == null) chosen = tags.get(0);

                // Get camera->tag pose
                Pose3D pose = null;
                try {
                    pose = chosen.getCameraPoseTargetSpace();
                } catch (Exception ignored) {}

                if (pose != null) {
                    tagSeen = true;
                    tagId = (int) chosen.getFiducialId();

                    Position p = pose.getPosition();

                    double xMm = Math.abs(DistanceUnit.MM.fromUnit(p.unit, p.x));
                    double zMm = Math.abs(DistanceUnit.MM.fromUnit(p.unit, p.z));

                    double cameraToFlywheelMm = CAMERA_TO_FLYWHEEL_IN * 25.4;
                    double shooterHorizMm = Math.hypot(xMm, zMm + cameraToFlywheelMm);

                    shooterDistIn = shooterHorizMm * IN_PER_MM;
                    if (shooterDistIn < 0) shooterDistIn = 0;

                    double newHood   = interp(DIST_IN, HOOD_POS, shooterDistIn);
                    double newMin    = interp(DIST_IN, RPM_MIN,  shooterDistIn);
                    double newTgt    = interp(DIST_IN, RPM_TGT,  shooterDistIn);
                    double newMax    = interp(DIST_IN, RPM_MAX,  shooterDistIn);

                    // Update live cmds
                    hoodCmd = newHood;
                    rpmMinCmd = newMin;
                    rpmTgtCmd = newTgt;
                    rpmMaxCmd = newMax;

                    // Update held-last cmds
                    lastHoodCmd = newHood;
                    lastRpmMinCmd = newMin;
                    lastRpmTgtCmd = newTgt;
                    lastRpmMaxCmd = newMax;
                    haveLastShotSolution = true;

                    gotNewSolution = true;
                }
            }
        }

        // If we didn't get a new solution but we have a last one, keep using it
        if (!gotNewSolution && haveLastShotSolution) {
            hoodCmd = lastHoodCmd;
            rpmMinCmd = lastRpmMinCmd;
            rpmTgtCmd = lastRpmTgtCmd;
            rpmMaxCmd = lastRpmMaxCmd;
        }

        // If we have nothing yet, keep "no-solution" outputs; motor logic handles startup RPM.
        if (!haveLastShotSolution) {
            // Keep hoodCmd as-is (no forced movement); keep RPM cmds at 0
            rpmMinCmd = 0;
            rpmTgtCmd = 0;
            rpmMaxCmd = 0;
        }

        // ===== IMPORTANT CHANGE =====
        // Only move hood when enabled (so no INIT servo motion).
        if (enabled) {
            if (haveLastShotSolution) {
                hood.setPosition(lastHoodCmd);
            } else if (hoodInitialized) {
                // keep whatever safe/default was set on enable
                // (do nothing)
            } else {
                // enabled but hood not initialized (shouldn't happen because setEnabled(true) initializes it)
                // do nothing
            }
        }
    }

    private static double interp(double[] x, double[] y, double xi) {
        if (x == null || y == null) return 0;
        if (x.length != y.length || x.length == 0) return 0;

        if (xi <= x[0]) return y[0];
        if (xi >= x[x.length - 1]) return y[y.length - 1];

        int hi = 1;
        while (hi < x.length && xi > x[hi]) hi++;
        int lo = hi - 1;

        double x0 = x[lo], x1 = x[hi];
        double y0 = y[lo], y1 = y[hi];

        double t = (xi - x0) / (x1 - x0);
        return y0 + t * (y1 - y0);
    }
}
