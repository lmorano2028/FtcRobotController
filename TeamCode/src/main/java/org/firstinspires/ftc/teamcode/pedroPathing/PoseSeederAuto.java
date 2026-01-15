package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.PoseStorage;

@Autonomous(name="PoseSeederAuto_FTC", group="Test")
public class PoseSeederAuto extends LinearOpMode {

    // Default seed pose (FTC field coordinates: inches + degrees)
    private double seedXIn = 0.0;
    private double seedYIn = 0.0;

    // Heading choices (FTC): 0, +90, 180, -90
    private int headingIndex = 2; // default 180
    private final double[] headingChoices = new double[] {0.0, 90.0, 180.0, -90.0};

    // Edge-detect buttons
    private boolean prevLeft  = false;
    private boolean prevRight = false;

    @Override
    public void runOpMode() {

        telemetry.addLine("=== PoseSeederAuto_FTC ===");
        telemetry.addLine("This Auto ONLY seeds PoseStorage for TeleOp.");
        telemetry.addLine("");
        telemetry.addLine("Controls (INIT):");
        telemetry.addLine("  dpad_left/right  = select heading (0,90,180,-90)");
        telemetry.addLine("");
        telemetry.addLine("After START:");
        telemetry.addLine("  It will write PoseStorage then end immediately.");
        telemetry.update();

        // INIT loop: choose heading
        while (!isStarted() && !isStopRequested()) {

            boolean left  = gamepad1.dpad_left;
            boolean right = gamepad1.dpad_right;

            if (left && !prevLeft) {
                headingIndex--;
                if (headingIndex < 0) headingIndex = headingChoices.length - 1;
            }
            if (right && !prevRight) {
                headingIndex++;
                if (headingIndex >= headingChoices.length) headingIndex = 0;
            }

            prevLeft  = left;
            prevRight = right;

            telemetry.addLine("=== PoseSeederAuto_FTC (INIT) ===");
            telemetry.addData("Seed X (in)", "%.1f", seedXIn);
            telemetry.addData("Seed Y (in)", "%.1f", seedYIn);
            telemetry.addData("Seed Heading (deg)", "%.1f", headingChoices[headingIndex]);
            telemetry.addLine("");
            telemetry.addLine("Next step: PRESS START, then run TeleOp:");
            telemetry.addLine("  TurretAim_OdoOnly_FTC_Seeded");
            telemetry.update();

            sleep(20);
        }

        waitForStart();
        if (isStopRequested()) return;

        // Write PoseStorage right after START (and before Auto ends)
        PoseStorage.xIn = seedXIn;
        PoseStorage.yIn = seedYIn;
        PoseStorage.headingDeg = headingChoices[headingIndex];
        PoseStorage.valid = true;

        telemetry.addLine("=== Pose seeded! ===");
        telemetry.addData("PoseStorage.valid", PoseStorage.valid);
        telemetry.addData("xIn", "%.1f", PoseStorage.xIn);
        telemetry.addData("yIn", "%.1f", PoseStorage.yIn);
        telemetry.addData("headingDeg", "%.1f", PoseStorage.headingDeg);
        telemetry.addLine("");
        telemetry.addLine("Now STOP this Auto and INIT TeleOp:");
        telemetry.addLine("  TurretAim_OdoOnly_FTC_Seeded");
        telemetry.update();

        // Give DS time to display telemetry
        sleep(500);

        // End auto (PoseStorage persists for TeleOp)
        requestOpModeStop();
    }
}
