package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pinpoint.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.pinpoint.PoseStorage;

@Autonomous(name="BasicPinpointAutoTests", group="Test")
public class BasicPinpointAutoTests extends LinearOpMode {

    // ===== CHANGE THESE TO YOUR DRIVE MOTOR NAMES =====
    private static final String LF_NAME = "FL";
    private static final String RF_NAME = "FR";
    private static final String LB_NAME = "BL";
    private static final String RB_NAME = "BR";

    private static final String PINPOINT_NAME = "pinpoint";

    // Pinpoint config (must match TeleOp)
    private static final double PINPOINT_X_OFFSET_MM = 82.55;
    private static final double PINPOINT_Y_OFFSET_MM = -95.25;

    private static final GoBildaPinpointDriver.EncoderDirection X_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection Y_DIR =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;

    // Simple “good enough” powers for testing
    private static final double DRIVE_POWER = 0.35;
    private static final double STRAFE_POWER = 0.35;
    private static final double TURN_POWER = 0.30;

    // Distance targets
    private static final double INCH_TO_MM = 25.4;
    private static final double FWD_36_MM  = 36.0 * INCH_TO_MM;
    private static final double FWD_12_MM  = 12.0 * INCH_TO_MM;

    // Heading target
    private static final double TURN_LEFT_90_DEG = 90.0;

    private enum TestMode { FORWARD_36, FORWARD_36_TURN_90_FWD_12 }
    private TestMode mode = TestMode.FORWARD_36;

    // ===== NEW: Start heading chooser (deg) =====
    // Choose the Pinpoint *raw* heading you want to seed at start.
    // This should match your TeleOp convention (where TeleOp applies its own offset).
    private double startHeadingDeg = 0.0;

    private DcMotorEx lf, rf, lb, rb;
    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() {
        PoseStorage.clear();

        lf = hardwareMap.get(DcMotorEx.class, LF_NAME);
        rf = hardwareMap.get(DcMotorEx.class, RF_NAME);
        lb = hardwareMap.get(DcMotorEx.class, LB_NAME);
        rb = hardwareMap.get(DcMotorEx.class, RB_NAME);

        // Typical mecanum directions vary—don’t change here unless your bot drives wrong
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);
        odo.setOffsets(PINPOINT_X_OFFSET_MM, PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(X_DIR, Y_DIR);

        // Reset + seed a known start pose (0,0,startHeadingDeg)
        odo.resetPosAndIMU();
        odo.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, startHeadingDeg));

        // Choose test mode + start heading in INIT
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_up) mode = TestMode.FORWARD_36;
            if (gamepad1.dpad_right) mode = TestMode.FORWARD_36_TURN_90_FWD_12;

            // ===== NEW: Start heading chooser =====
            // dpad_left  = 0 deg
            // dpad_down  = -90 deg
            // dpad_right = +90 deg
            // dpad_up    = 180 deg
            // (Note: dpad_up is also used for mode; mode selection still works because it’s just setting a value.)
            if (gamepad1.dpad_left)  startHeadingDeg = 0.0;
            if (gamepad1.dpad_right) startHeadingDeg = 90.0;
            if (gamepad1.dpad_down)  startHeadingDeg = -90.0;
            if (gamepad1.dpad_up)    startHeadingDeg = 180.0;

            // Re-seed pose continuously so what you see is what you’ll start with
            odo.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, startHeadingDeg));

            odo.update();
            Pose2D p = odo.getPosition();

            telemetry.addLine("Choose Auto Test Mode:");
            telemetry.addLine("DPAD UP    = Forward 36\"");
            telemetry.addLine("DPAD RIGHT = Forward 36\" + Left 90 + Forward 12\"");
            telemetry.addData("Selected", mode);

            telemetry.addLine("");
            telemetry.addLine("Choose START HEADING (Pinpoint raw heading seed):");
            telemetry.addLine("DPAD LEFT  =   0 deg");
            telemetry.addLine("DPAD DOWN  = -90 deg");
            telemetry.addLine("DPAD RIGHT = +90 deg");
            telemetry.addLine("DPAD UP    = 180 deg");
            telemetry.addData("StartHeadingDeg", "%.1f", startHeadingDeg);

            telemetry.addData("Pose (mm,deg)",
                    "X=%.1f Y=%.1f H=%.1f",
                    p.getX(DistanceUnit.MM), p.getY(DistanceUnit.MM), p.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // Always update a few cycles first
        for (int i=0; i<10; i++) {
            odo.update();
            sleep(10);
        }

        if (mode == TestMode.FORWARD_36) {
            driveForwardMm(FWD_36_MM);
        } else {
            driveForwardMm(FWD_36_MM);
            turnToHeadingDeg(TURN_LEFT_90_DEG);
            driveForwardMm(FWD_36_MM);
            //driveForwardMm(FWD_12_MM);
        }

        stopDrive();

        // Save pose for TeleOp
        odo.update();
        Pose2D endPose = odo.getPosition();
        PoseStorage.writeFromPose(endPose);

        telemetry.addLine("AUTO DONE - Pose saved for TeleOp");
        telemetry.addData("End Pose (mm,deg)",
                "X=%.1f Y=%.1f H=%.1f",
                endPose.getX(DistanceUnit.MM), endPose.getY(DistanceUnit.MM), endPose.getHeading(AngleUnit.DEGREES));
        telemetry.update();

        sleep(1500);
    }

    private void driveForwardMm(double targetMm) {
        odo.update();
        Pose2D start = odo.getPosition();
        double startX = start.getX(DistanceUnit.MM);
        double startY = start.getY(DistanceUnit.MM);

        while (opModeIsActive()) {
            odo.update();
            Pose2D p = odo.getPosition();
            double dx = p.getX(DistanceUnit.MM) - startX;
            double dy = p.getY(DistanceUnit.MM) - startY;
            double dist = Math.hypot(dx, dy);

            if (dist >= targetMm) break;

            setMecanum(DRIVE_POWER, 0, 0); // forward
            telemetry.addData("Driving", "%.0f / %.0f mm", dist, targetMm);
            telemetry.update();
        }
        stopDrive();
        sleep(200);
    }

    private void turnToHeadingDeg(double targetDeg) {
        // Turn left to +90 relative to start heading in your seeded coordinate.
        odo.update();
        Pose2D start = odo.getPosition();
        double startH = start.getHeading(AngleUnit.DEGREES);
        double goalH = wrapDeg(startH + targetDeg);

        while (opModeIsActive()) {
            odo.update();
            double h = odo.getPosition().getHeading(AngleUnit.DEGREES);
            double err = wrapDeg(goalH - h);

            if (Math.abs(err) < 2.0) break;

            double dir = (err > 0) ? +1.0 : -1.0; // positive error -> turn left
            setMecanum(0, 0, dir * TURN_POWER);

            telemetry.addData("Turning", "H=%.1f Goal=%.1f Err=%.1f", h, goalH, err);
            telemetry.update();
        }
        stopDrive();
        sleep(200);
    }

    // Simple mecanum mixer: forward, strafe, turn
    private void setMecanum(double fwd, double strafe, double turn) {
        double lfP = fwd + strafe + turn;
        double rfP = fwd - strafe - turn;
        double lbP = fwd - strafe + turn;
        double rbP = fwd + strafe - turn;

        // normalize
        double max = Math.max(1.0, Math.max(Math.abs(lfP), Math.max(Math.abs(rfP), Math.max(Math.abs(lbP), Math.abs(rbP)))));
        lfP /= max; rfP /= max; lbP /= max; rbP /= max;

        lf.setPower(lfP);
        rf.setPower(rfP);
        lb.setPower(lbP);
        rb.setPower(rbP);
    }

    private void stopDrive() {
        lf.setPower(0); rf.setPower(0); lb.setPower(0); rb.setPower(0);
    }

    private double wrapDeg(double deg) {
        while (deg >= 180.0) deg -= 360.0;
        while (deg < -180.0) deg += 360.0;
        return deg;
    }
}
