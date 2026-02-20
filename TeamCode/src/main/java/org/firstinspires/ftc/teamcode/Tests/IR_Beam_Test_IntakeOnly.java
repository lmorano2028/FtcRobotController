package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * IR Beam Break Sensor Test TeleOp (beamIntake ONLY)
 *
 * Hardware Config names:
 *  - beamIntake  (Digital Device)
 *  - statuslight (Servo)
 *
 * Light behavior:
 *  - SOLID GREEN whenever beamIntake is broken (held low)
 *  - FLASH GREEN for 1.5s when beamIntake was broken and then becomes NOT broken (broken -> clear edge)
 *  - OFF otherwise
 *
 * NOTE: With pull-up wiring:
 *  - beam CLEAR  => DigitalChannel.getState() == true
 *  - beam BROKEN => DigitalChannel.getState() == false
 */
@Disabled
@TeleOp(name="IR_Beam_Test_IntakeOnly", group="Test")
public class IR_Beam_Test_IntakeOnly extends OpMode {

    // ======= CONFIG NAMES =======
    private static final String BEAM_INTAKE_NAME = "beamIntake";
    private static final String STATUS_LIGHT_NAME = "statuslight";

    // ======= RGB LIGHT POSITIONS (goBILDA chart FTC positions) =======
    // From your chart:
    //  Green 1500us => FTC: 0.500
    private static final double LIGHT_OFF   = 0.000; // Off (500us)
    private static final double LIGHT_GREEN = 0.500; // Green (1500us)

    // Flash timing
    private static final double FLASH_DURATION_SEC = 1.5;
    private static final double FLASH_PERIOD_SEC   = 0.15; // 150ms toggle rate

    private DigitalChannel beamIntake;
    private Servo statuslight;

    // Previous broken state for edge detect
    private boolean prevIntakeBroken = false;

    // Flash state
    private boolean flashing = false;
    private final ElapsedTime flashTimer = new ElapsedTime();
    private final ElapsedTime flashToggleTimer = new ElapsedTime();
    private boolean flashOn = false;

    @Override
    public void init() {
        beamIntake = hardwareMap.get(DigitalChannel.class, BEAM_INTAKE_NAME);
        beamIntake.setMode(DigitalChannel.Mode.INPUT);

        statuslight = hardwareMap.get(Servo.class, STATUS_LIGHT_NAME);
        statuslight.setPosition(LIGHT_OFF);

        prevIntakeBroken = false;

        flashing = false;
        flashTimer.reset();
        flashToggleTimer.reset();
        flashOn = false;

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("IR_Beam_Test_IntakeOnly READY");
        telemetry.addLine("Break beamIntake -> SOLID GREEN");
        telemetry.addLine("Pass ball through (break then clear) -> FLASH 1.5s");
        telemetry.addData("GREEN PWM (FTC pos)", "%.3f", LIGHT_GREEN);
        telemetry.update();
    }

    @Override
    public void loop() {
        // With pull-up wiring: getState() == true means CLEAR (not broken)
        // Beam broken pulls line LOW => getState() == false
        boolean intakeBroken = !beamIntake.getState();

        // Edge detect: broken -> clear (ball passed the beam)
        boolean intakeClearedEvent = (prevIntakeBroken && !intakeBroken);

        if (intakeClearedEvent) {
            startFlash();
        }

        // Priority: SOLID GREEN while broken
        if (intakeBroken) {
            flashing = false; // cancel flash while currently broken
            statuslight.setPosition(LIGHT_GREEN);
        } else if (flashing) {
            // Run flash for 1.5s
            if (flashTimer.seconds() >= FLASH_DURATION_SEC) {
                flashing = false;
                statuslight.setPosition(LIGHT_OFF);
            } else {
                // Toggle flash on/off
                if (flashToggleTimer.seconds() >= FLASH_PERIOD_SEC) {
                    flashToggleTimer.reset();
                    flashOn = !flashOn;
                }
                statuslight.setPosition(flashOn ? LIGHT_GREEN : LIGHT_OFF);
            }
        } else {
            statuslight.setPosition(LIGHT_OFF);
        }

        // Telemetry
        telemetry.addLine("=== IR Beam Test (Intake Only) ===");
        telemetry.addData("beamIntake port", BEAM_INTAKE_NAME);
        telemetry.addData("beamIntake (raw getState)", beamIntake.getState());
        telemetry.addData("IntakeBroken", intakeBroken);
        telemetry.addData("IntakeClearedEvent", intakeClearedEvent);
        telemetry.addData("Flashing", flashing);
        telemetry.addData("FlashTimer(s)", "%.2f", flashTimer.seconds());
        telemetry.update();

        // Save prev
        prevIntakeBroken = intakeBroken;
    }

    private void startFlash() {
        flashing = true;
        flashTimer.reset();
        flashToggleTimer.reset();
        flashOn = true;
    }

    @Override
    public void stop() {
        if (statuslight != null) statuslight.setPosition(LIGHT_OFF);
    }
}
