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
    // Option B: Odometry-based distance fallback (no vision required)
    // =========================
    private boolean haveExternalDistance = false;
    private double externalDistIn = Double.NaN;

    // =========================
    // NEW: No movement in INIT + spinup behavior at START
    // =========================
    private static final double STARTUP_RPM = 1000.0;

    // We defer any hood movement until enabled (START)
    private boolean hoodInitialized = false;

    // =========================
    // #4 NEW: Distance filtering + RPM slew + Hood deadband/slew
    // =========================
    // Median-of-3 to kill spikes
    private final Median3Filter distMedian = new Median3Filter();
    // Optional: light smoothing after median (set alpha higher for less lag)
    private final LowPassFilter distLpf = new LowPassFilter(0.30);

    // Hold last good filtered distance so dropouts don't whipsaw
    private boolean haveLastFilteredDist = false;
    private double lastFilteredDistIn = Double.NaN;

    // RPM and hood stabilization
    private final SlewLimiter rpmSlew = new SlewLimiter();
    private final HoodController hoodStabilizer = new HoodController();

    // Tune knobs (safe starters)
    private static final double RPM_SLEW_PER_LOOP = 70.0;   // max delta rpm per update() call
    private static final double HOOD_DEADBAND = 0.003;      // servo units
    private static final double HOOD_MAX_STEP = 0.006;      // servo units per update() call

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

        // Option B state init
        haveExternalDistance = false;
        externalDistIn = Double.NaN;

        hoodCmd = 0.22;
        rpmMinCmd = 0;
        rpmTgtCmd = 0;
        rpmMaxCmd = 0;

        // #4 init
        haveLastFilteredDist = false;
        lastFilteredDistIn = Double.NaN;
        distMedian.reset();
        distLpf.reset();
        rpmSlew.reset();
        hoodStabilizer.reset();
    }

    public void startVision() {
        limelight.pipelineSwitch(aprilTagPipelineIndex);
        limelight.start();
    }

    public void stopVision() {
        limelight.stop();
    }

    /**
     * Option B: Provide an external distance (e.g., odometry → goal distance) when LL is missing/unreliable.
     * - Pass NaN or <=0 to clear.
     */
    public void setExternalDistanceIn(double distIn) {
        if (Double.isNaN(distIn) || distIn <= 0) {
            haveExternalDistance = false;
            externalDistIn = Double.NaN;
            return;
        }
        haveExternalDistance = true;
        externalDistIn = distIn;
    }

    /**
     * Call this at START (your opmode start()).
     * - When enabled becomes true: immediately spin at 1000 RPM until first shot solution arrives.
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

                // Also initialize "last" solution to safe hood; RPM still 0 until we compute a solution
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

    // #4 useful getters for telemetry
    public boolean hasLastFilteredDistance() { return haveLastFilteredDist; }
    public double getLastFilteredDistanceIn() { return lastFilteredDistIn; }

    /** Call every loop in auton/teleop to refresh tag → distance → hood/RPM and command motor. */
    public void update() {
        // Always compute/refresh shot solution (but only moves hood if enabled)
        updateFromLimelightAndComputeShot_HoldLast_Filtered();

        if (!enabled) {
            // If not enabled, do not command anything (including hood)
            return;
        }

        // If enabled but we don't have a shot solution yet: spin up immediately to startup RPM.
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

    private void updateFromLimelightAndComputeShot_HoldLast_Filtered() {
        // Always refresh tagSeen/tagId for telemetry/logic, but do NOT zero commands on a miss
        tagSeen = false;
        tagId = -1;
        shooterDistIn = Double.NaN;

        boolean gotNewSolution = false;

        // (A) Try vision first
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

                    double distRawIn = shooterHorizMm * IN_PER_MM;
                    if (distRawIn < 0) distRawIn = 0;

                    // #4: FILTER distance BEFORE interpolation
                    double distMed = distMedian.update(distRawIn);
                    double distFilt = distLpf.update(distMed);

                    shooterDistIn = distRawIn; // keep RAW for telemetry; filtered is stored below

                    // Update "last good filtered dist"
                    lastFilteredDistIn = distFilt;
                    haveLastFilteredDist = true;

                    // Build shot solution from filtered distance
                    applyShotSolutionFromDistance(distFilt);

                    gotNewSolution = true;
                }
            }
        }

        // (B) If no new vision solution, but we have an external distance, build a filtered solution from it.
        if (!gotNewSolution && haveExternalDistance) {
            double distRawIn = externalDistIn;

            // Filter external too (keeps behavior identical between vision and fallback)
            double distMed = distMedian.update(distRawIn);
            double distFilt = distLpf.update(distMed);

            shooterDistIn = distRawIn; // raw external for telemetry
            lastFilteredDistIn = distFilt;
            haveLastFilteredDist = true;

            applyShotSolutionFromDistance(distFilt);
            gotNewSolution = true;
        }

        // (C) If no new solution now, HOLD last solution (including last filtered distance)
        if (!gotNewSolution && haveLastShotSolution) {
            hoodCmd = lastHoodCmd;
            rpmMinCmd = lastRpmMinCmd;
            rpmTgtCmd = lastRpmTgtCmd;
            rpmMaxCmd = lastRpmMaxCmd;
        }

        // (D) If we have nothing yet, keep "no-solution" outputs; motor logic handles startup RPM.
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
            }
        }
    }

    /**
     * Builds hood/rpm commands from a distance that has ALREADY been filtered.
     * Applies:
     *  - distance -> interpolate -> raw targets
     *  - RPM slew limiting
     *  - Hood deadband + slew
     *  - updates hold-last (TeleOp behavior)
     */
    private void applyShotSolutionFromDistance(double distInFiltered) {
        double newHoodRaw = interp(DIST_IN, HOOD_POS, distInFiltered);
        double newMinRaw  = interp(DIST_IN, RPM_MIN,  distInFiltered);
        double newTgtRaw  = interp(DIST_IN, RPM_TGT,  distInFiltered);
        double newMaxRaw  = interp(DIST_IN, RPM_MAX,  distInFiltered);

        // #4 optional: RPM slew limit (slew ONLY the target; keep min/max window centered around slewed target)
        double newTgt = rpmSlew.update(newTgtRaw, RPM_SLEW_PER_LOOP);
        double halfWin = 0.5 * Math.max(0.0, (newMaxRaw - newMinRaw));
        double newMin = Math.max(0.0, newTgt - halfWin);
        double newMax = newTgt + halfWin;

        // #4 optional: hood deadband/slew
        double newHood = hoodStabilizer.update(newHoodRaw, HOOD_DEADBAND, HOOD_MAX_STEP);

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

    // =========================
    // Small helper classes (kept inside ShooterSubsystem for copy/paste simplicity)
    // =========================

    private static class Median3Filter {
        private double a = Double.NaN, b = Double.NaN, c = Double.NaN;

        public void reset() { a = b = c = Double.NaN; }

        public double update(double x) {
            a = b; b = c; c = x;
            if (Double.isNaN(a) || Double.isNaN(b)) return x; // not enough history yet
            return median(a, b, c);
        }

        private double median(double x, double y, double z) {
            return Math.max(Math.min(x, y), Math.min(Math.max(x, y), z));
        }
    }

    private static class LowPassFilter {
        private boolean init = false;
        private double y = 0.0;
        private final double alpha; // 0..1 (higher = less smoothing)

        public LowPassFilter(double alpha) { this.alpha = alpha; }

        public void reset() { init = false; y = 0.0; }

        public double update(double x) {
            if (!init) { y = x; init = true; return y; }
            y = y + alpha * (x - y);
            return y;
        }
    }

    private static class SlewLimiter {
        private double last = Double.NaN;

        public void reset() { last = Double.NaN; }

        public double update(double target, double maxDeltaPerCall) {
            if (Double.isNaN(last)) { last = target; return last; }
            double delta = target - last;
            if (delta >  maxDeltaPerCall) delta =  maxDeltaPerCall;
            if (delta < -maxDeltaPerCall) delta = -maxDeltaPerCall;
            last += delta;
            return last;
        }
    }

    private static class HoodController {
        private double last = Double.NaN;

        public void reset() { last = Double.NaN; }

        public double update(double targetPos, double deadband, double maxStep) {
            if (Double.isNaN(last)) { last = targetPos; return last; }

            if (Math.abs(targetPos - last) <= deadband) {
                return last; // ignore micro adjustments
            }

            double delta = targetPos - last;
            if (delta >  maxStep) delta =  maxStep;
            if (delta < -maxStep) delta = -maxStep;
            last += delta;
            return last;
        }
    }
}