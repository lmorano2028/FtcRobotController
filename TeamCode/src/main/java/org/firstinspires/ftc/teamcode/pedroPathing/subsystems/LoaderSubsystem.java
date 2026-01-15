package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LoaderSubsystem {

    private final DcMotorEx intake2;
    private final DcMotorEx flicker;
    private final Servo flipper;

    // Flipper positions
    private final double FLIP_DOWN;
    private final double FLIP_UP;

    // Hold / Feed / Decompress
    private final double HOLD_PWR_INTAKE2;
    private final double HOLD_PWR_FLICKER;

    private final double FEED_PWR_INTAKE2;
    private final double FEED_PWR_FLICKER;
    private final int FEED_MS;

    private final double DECOMPRESS_PWR_INTAKE2;
    private final int DECOMPRESS_MS;

    private final int MIN_RECOVER_MS;

    // TeleOp special spacing between shot1->shot2
    private final int SHOT12_SPACING_MS;

    private final IntakeSubsystem intakeSubsystemOrNull;

    private boolean volleyFeedAssistEnabled = false;
    private boolean volleyFeedAssistActive = false;

    public enum SeqState {
        IDLE,

        SHOT1_FEED, SHOT1_DECOMP, SHOT1_RECOVER,
        SHOT2_FEED, SHOT2_DECOMP, SHOT2_RECOVER,
        SHOT3_FEED, SHOT3_DECOMP, SHOT3_RECOVER,
        SHOT4_FEED, SHOT4_DECOMP, SHOT4_RECOVER,

        DONE,
        ABORTED
    }

    private SeqState state = SeqState.IDLE;
    private final ElapsedTime stateTimer = new ElapsedTime();

    public LoaderSubsystem(
            DcMotorEx intake2,
            DcMotorEx flicker,
            Servo flipper,
            IntakeSubsystem intakeSubsystemOrNull,
            double flipDown, double flipUp,
            double holdIntake2, double holdFlicker,
            double feedIntake2, double feedFlicker, int feedMs,
            double decompressIntake2, int decompressMs,
            int minRecoverMs
    ) {
        this.intake2 = intake2;
        this.flicker = flicker;
        this.flipper = flipper;
        this.intakeSubsystemOrNull = intakeSubsystemOrNull;

        this.FLIP_DOWN = flipDown;
        this.FLIP_UP = flipUp;

        this.HOLD_PWR_INTAKE2 = holdIntake2;
        this.HOLD_PWR_FLICKER = holdFlicker;

        this.FEED_PWR_INTAKE2 = feedIntake2;
        this.FEED_PWR_FLICKER = feedFlicker;
        this.FEED_MS = feedMs;

        this.DECOMPRESS_PWR_INTAKE2 = decompressIntake2;
        this.DECOMPRESS_MS = decompressMs;

        this.MIN_RECOVER_MS = minRecoverMs;

        this.SHOT12_SPACING_MS = 220;

        flipper.setPosition(FLIP_DOWN);
        stopAll();
    }

    public void setVolleyFeedAssistEnabled(boolean enabled) {
        this.volleyFeedAssistEnabled = enabled;
        if (!enabled) {
            volleyFeedAssistActive = false;
        }
    }

    public boolean isVolleyFeedAssistEnabled() {
        return volleyFeedAssistEnabled;
    }

    private void updateVolleyFeedAssistLatch() {
        if (!volleyFeedAssistEnabled) {
            volleyFeedAssistActive = false;
            return;
        }

        if (state == SeqState.SHOT2_FEED) {
            volleyFeedAssistActive = true;
        }

        if (state == SeqState.IDLE || state == SeqState.DONE || state == SeqState.ABORTED) {
            volleyFeedAssistActive = false;
        }
    }

    public void feedOn() {
        flipper.setPosition(FLIP_DOWN);
        intake2.setPower(FEED_PWR_INTAKE2);
        flicker.setPower(FEED_PWR_FLICKER);
    }

    public void feedOff() {
        intake2.setPower(0);
        flicker.setPower(0);
        flipper.setPosition(FLIP_DOWN);
        if (intakeSubsystemOrNull != null) intakeSubsystemOrNull.setBurstOverride(false);
    }

    public void hold() {
        flipper.setPosition(FLIP_DOWN);
        intake2.setPower(HOLD_PWR_INTAKE2);
        flicker.setPower(HOLD_PWR_FLICKER);
        if (intakeSubsystemOrNull != null) intakeSubsystemOrNull.setBurstOverride(false);
    }

    public void decompress() {
        flipper.setPosition(FLIP_DOWN);
        intake2.setPower(DECOMPRESS_PWR_INTAKE2);
        flicker.setPower(0);
    }

    public void stopAll() {
        intake2.setPower(0);
        flicker.setPower(0);
        flipper.setPosition(FLIP_DOWN);
        if (intakeSubsystemOrNull != null) intakeSubsystemOrNull.setBurstOverride(false);
        state = SeqState.IDLE;
        stateTimer.reset();
        volleyFeedAssistActive = false;
    }

    public void startFourShot() {
        if (state != SeqState.IDLE) return;

        flipper.setPosition(FLIP_DOWN);
        if (intakeSubsystemOrNull != null) intakeSubsystemOrNull.setBurstOverride(false);

        volleyFeedAssistActive = false;

        transitionTo(SeqState.SHOT1_FEED);
        applyFeedForCurrentShot();
    }

    public void updateFourShot(boolean shooterReadyStable) {

        updateVolleyFeedAssistLatch();

        if (volleyFeedAssistActive && intakeSubsystemOrNull != null) {
            intakeSubsystemOrNull.setEnabled(true);
        }

        switch (state) {
            case IDLE:
            case ABORTED:
                return;

            case SHOT1_FEED:
                if (elapsedMs() >= FEED_MS) {
                    transitionTo(SeqState.SHOT1_DECOMP);
                    applyDecompress(true); // ONLY shot that decompresses
                }
                break;

            case SHOT1_DECOMP:
                if (elapsedMs() >= DECOMPRESS_MS) {
                    transitionTo(SeqState.SHOT1_RECOVER);
                    applyRecoverHold();
                }
                break;

            case SHOT1_RECOVER:
                if (elapsedMs() >= SHOT12_SPACING_MS && shooterReadyStable) {
                    transitionTo(SeqState.SHOT2_FEED);
                    applyFeedForCurrentShot();
                }
                break;

            case SHOT2_FEED:
                if (elapsedMs() >= FEED_MS) {
                    // CHANGED: skip decompress for shot2
                    transitionTo(SeqState.SHOT2_RECOVER);
                    applyRecoverHold();
                }
                break;

            case SHOT2_RECOVER:
                if (elapsedMs() >= MIN_RECOVER_MS && shooterReadyStable) {
                    transitionTo(SeqState.SHOT3_FEED);
                    applyFeedForCurrentShot();
                }
                break;

            case SHOT3_FEED:
                if (elapsedMs() >= FEED_MS) {
                    // CHANGED: skip decompress for shot3
                    transitionTo(SeqState.SHOT3_RECOVER);
                    applyRecoverHold();
                }
                break;

            case SHOT3_RECOVER:
                if (elapsedMs() >= MIN_RECOVER_MS && shooterReadyStable) {
                    transitionTo(SeqState.SHOT4_FEED);
                    applyFeedForCurrentShot();
                }
                break;

            case SHOT4_FEED:
                if (elapsedMs() >= FEED_MS) {
                    // CHANGED: skip decompress for shot4
                    transitionTo(SeqState.SHOT4_RECOVER);
                    applyRecoverHold();
                }
                break;

            case SHOT4_RECOVER:
                if (elapsedMs() >= MIN_RECOVER_MS && shooterReadyStable) {
                    feedOff();
                    state = SeqState.DONE;
                    stateTimer.reset();
                    volleyFeedAssistActive = false;
                }
                break;

            case SHOT2_DECOMP:
            case SHOT3_DECOMP:
            case SHOT4_DECOMP:
                // Should never be entered now, but keep safe behavior if it happens
                if (elapsedMs() >= DECOMPRESS_MS) {
                    // fall through to corresponding recover
                    if (state == SeqState.SHOT2_DECOMP) transitionTo(SeqState.SHOT2_RECOVER);
                    if (state == SeqState.SHOT3_DECOMP) transitionTo(SeqState.SHOT3_RECOVER);
                    if (state == SeqState.SHOT4_DECOMP) transitionTo(SeqState.SHOT4_RECOVER);
                    applyRecoverHold();
                }
                break;

            case DONE:
                state = SeqState.IDLE;
                stateTimer.reset();
                volleyFeedAssistActive = false;
                break;
        }
    }

    public SeqState getState() { return state; }
    public boolean isIdle() { return state == SeqState.IDLE; }
    public boolean isDone() { return state == SeqState.DONE; }

    public void abort() {
        feedOff();
        state = SeqState.ABORTED;
        stateTimer.reset();
        volleyFeedAssistActive = false;
    }

    private void transitionTo(SeqState newState) {
        state = newState;
        stateTimer.reset();
        updateVolleyFeedAssistLatch();
    }

    private long elapsedMs() {
        return (long) stateTimer.milliseconds();
    }

    private void applyFeedForCurrentShot() {
        if (state == SeqState.SHOT3_FEED || state == SeqState.SHOT4_FEED) {
            flipper.setPosition(FLIP_UP);
        } else {
            flipper.setPosition(FLIP_DOWN);
        }

        intake2.setPower(FEED_PWR_INTAKE2);
        flicker.setPower(FEED_PWR_FLICKER);

        if (intakeSubsystemOrNull != null) intakeSubsystemOrNull.setBurstOverride(false);
    }

    private void applyDecompress(boolean enableBurst) {
        flipper.setPosition(FLIP_DOWN);

        intake2.setPower(DECOMPRESS_PWR_INTAKE2);
        flicker.setPower(0);

        if (intakeSubsystemOrNull != null) {
            intakeSubsystemOrNull.setBurstOverride(enableBurst);
        }
    }

    private void applyRecoverHold() {
        flipper.setPosition(FLIP_DOWN);
        intake2.setPower(HOLD_PWR_INTAKE2);
        flicker.setPower(HOLD_PWR_FLICKER);

        if (intakeSubsystemOrNull != null) intakeSubsystemOrNull.setBurstOverride(false);
    }
}
