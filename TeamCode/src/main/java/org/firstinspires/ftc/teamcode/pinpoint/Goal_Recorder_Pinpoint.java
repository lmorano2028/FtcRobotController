package org.firstinspires.ftc.teamcode.pinpoint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@TeleOp(name="Goal_Recorder_Pinpoint", group="Test")
//@Disabled
public class Goal_Recorder_Pinpoint extends LinearOpMode {

    private GoBildaPinpointDriver odo;

    // Stored goal points (Pinpoint frame, mm)
    private boolean blueSaved = false;
    private boolean redSaved  = false;

    private double blueXmm = 0, blueYmm = 0, blueHmm = 0;
    private double redXmm  = 0, redYmm  = 0, redHmm  = 0;

    // Button edge detection
    private boolean lastA = false, lastX = false, lastY = false, lastB = false;

    // Your current tuned offsets (from your example)
    private static final double X_POD_OFFSET_MM = 82.55;
    private static final double Y_POD_OFFSET_MM = -95.25;

    @Override
    public void runOpMode() {

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Same setup as your example (do not change your conventions)
        odo.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,   // X increases forward
                GoBildaPinpointDriver.EncoderDirection.REVERSED   // Y increases left (per your note)
        );

        // Start with a clean calibration and zero pose
        odo.resetPosAndIMU();

        telemetry.addLine("Step 3: Goal Coordinate Recorder (Pinpoint frame, mm)");
        telemetry.addLine("Place robot at FIELD CENTER, facing BLUE wall.");
        telemetry.addLine("A = Zero pose (0,0,0) at center");
        telemetry.addLine("X = Save BLUE goal point (after you push into BLUE wall/goal)");
        telemetry.addLine("Y = Save RED goal point (after you push into RED wall/goal)");
        telemetry.addLine("B = Clear saved points");
        telemetry.addLine("Tip: Keep turret centered; robot heading should stay ~0 deg when driving straight.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            odo.update();

            Pose2D pos = odo.getPosition();
            double x = pos.getX(DistanceUnit.MM);
            double y = pos.getY(DistanceUnit.MM);
            double h = pos.getHeading(AngleUnit.DEGREES);

            // Edge-detect buttons
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean xBtn = gamepad1.x;
            boolean yBtn = gamepad1.y;

            boolean aPressed = a && !lastA;
            boolean bPressed = b && !lastB;
            boolean xPressed = xBtn && !lastX;
            boolean yPressed = yBtn && !lastY;

            lastA = a;
            lastB = b;
            lastX = xBtn;
            lastY = yBtn;

            // A: Zero pose at field center
            if (aPressed) {
                // We want (0,0,0) at the center without forcing a full IMU recal every time.
                // resetPosAndIMU() is okay here since you're stationary at center; it's the most robust.
                odo.resetPosAndIMU();
                blueSaved = false;
                redSaved = false;
            }

            // B: Clear saves (does not change odometry)
            if (bPressed) {
                blueSaved = false;
                redSaved = false;
            }

            // X: Save BLUE goal
            if (xPressed) {
                blueXmm = x;
                blueYmm = y;
                blueHmm = h;
                blueSaved = true;
            }

            // Y: Save RED goal
            if (yPressed) {
                redXmm = x;
                redYmm = y;
                redHmm = h;
                redSaved = true;
            }

            // ===== Telemetry =====
            telemetry.addLine("=== LIVE PINPOINT POSE (mm, deg) ===");
            telemetry.addData("Pose", String.format(Locale.US, "X: %.1f  Y: %.1f  H: %.1f", x, y, h));
            telemetry.addData("Device Status", odo.getDeviceStatus());
            telemetry.addData("Pinpoint Hz", odo.getFrequency());

            telemetry.addLine("");
            telemetry.addLine("=== STEP 3 INSTRUCTIONS ===");
            telemetry.addLine("1) Put robot at center, facing BLUE wall.");
            telemetry.addLine("2) Press A to zero pose -> should read near X=0, Y=0, H=0.");
            telemetry.addLine("3) Push/drive straight into BLUE goal/wall. Press X to save BLUE.");
            telemetry.addLine("4) Return to center, press A again to re-zero.");
            telemetry.addLine("5) Push/drive straight into RED goal/wall. Press Y to save RED.");

            telemetry.addLine("");
            telemetry.addLine("=== SAVED GOAL POINTS (Pinpoint frame) ===");
            if (blueSaved) {
                telemetry.addData("BLUE GOAL", String.format(Locale.US,
                        "X: %.1f mm  Y: %.1f mm  H: %.1f deg", blueXmm, blueYmm, blueHmm));
            } else {
                telemetry.addData("BLUE GOAL", "Not saved (push to BLUE wall then press X)");
            }

            if (redSaved) {
                telemetry.addData("RED GOAL", String.format(Locale.US,
                        "X: %.1f mm  Y: %.1f mm  H: %.1f deg", redXmm, redYmm, redHmm));
            } else {
                telemetry.addData("RED GOAL", "Not saved (push to RED wall then press Y)");
            }

            telemetry.addLine("");
            telemetry.addLine("=== WHAT TO SEND ME ===");
            telemetry.addLine("Send the two lines exactly:");
            telemetry.addLine("BLUE: (Xmm, Ymm) = (_____, _____)");
            telemetry.addLine("RED:  (Xmm, Ymm) = (_____, _____)");

            telemetry.update();
        }
    }
}
