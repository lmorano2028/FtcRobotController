package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * IntakeSubsystem (rewritten)
 *
 * Fixes the real issue:
 *  - Your old version ONLY applies motor power inside update().
 *    In auton, if you never call intake.update() every loop, burstOverride does nothing.
 *
 * This rewrite makes burst + enabled changes apply IMMEDIATELY, and update() is optional.
 * Public API preserved (setEnabled, isEnabled, setBurstOverride, update, stop).
 */
public class IntakeSubsystem {
    private final DcMotorEx intake1;
    private final double INTAKE1_PWR;
    private final double INTAKE1_BURST_PWR;

    private boolean enabled = false;
    private boolean burstOverride = false;

    public IntakeSubsystem(DcMotorEx intake1, double intake1Pwr, double intake1BurstPwr) {
        this.intake1 = intake1;
        this.INTAKE1_PWR = intake1Pwr;
        this.INTAKE1_BURST_PWR = intake1BurstPwr;

        // Safe default
        intake1.setPower(0.0);
    }

    public void setEnabled(boolean on) {
        enabled = on;
        apply(); // IMMEDIATE apply so it works even if update() isn't being called
    }

    public boolean isEnabled() { return enabled; }

    public void setBurstOverride(boolean on) {
        burstOverride = on;
        apply(); // IMMEDIATE apply so loader burst works reliably
    }

    /** Optional periodic call; safe to call every loop. */
    public void update() {
        apply();
    }

    public void stop() {
        enabled = false;
        burstOverride = false;
        intake1.setPower(0.0);
    }

    // ---------------- Internal ----------------
    private void apply() {
        if (burstOverride) {
            intake1.setPower(INTAKE1_BURST_PWR);
        } else {
            intake1.setPower(enabled ? INTAKE1_PWR : 0.0);
        }
    }
}
