package org.firstinspires.ftc.teamcode.DecodeOpModes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "ShooterMcGavin") //capitalize
public class ShooterMcGavin extends LinearOpMode {

    private DcMotor FranklinDelanoRoosevelt; // FR
    private DcMotor FL;
    private DcMotor BoeingDriveR;            // BR
    private DcMotor BL;

    private DcMotor intakeOne;
    private DcMotor intakeTwo;


    private DcMotor Shooter;

    //turret
    private Servo leftServo;
    private Servo rightServo;
    private DcMotor FastFingler;
    private Servo Servoo;

    IMU imu;

    //button variables
    boolean intakeOneOn = false;
    boolean intakeTwoOn = false;
    boolean FastFinglerOn = false;
    boolean shooterOn = false;
    boolean prevCircle = false;
    boolean prevX = false;

    boolean servoUp = false;
    boolean prevY = false;
    boolean prevDP = false;
    boolean prevA = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // Drive Motors
        FranklinDelanoRoosevelt = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BoeingDriveR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");


        // Intakes
        intakeOne = hardwareMap.get(DcMotor.class, "intakeOneMotor");
        intakeTwo = hardwareMap.get(DcMotor.class, "intakeTwoMotor");

        //shooter
        Shooter = hardwareMap.get(DcMotor.class,"ShooterMotor");

        //turret
        leftServo = hardwareMap.get(Servo.class,"leftServo");
        rightServo = hardwareMap.get(Servo.class,"rightServo");

        //shooter loader
        FastFingler = hardwareMap.get(DcMotor.class,"Outertake");
        Servoo = hardwareMap.get(Servo.class, "fingler");


        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        FranklinDelanoRoosevelt.setDirection(DcMotorSimple.Direction.FORWARD);
        BoeingDriveR.setDirection(DcMotorSimple.Direction.FORWARD);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FranklinDelanoRoosevelt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BoeingDriveR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- FIXED IMU INIT ---
        imu = hardwareMap.get(IMU.class, "imu");


        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )

        );
        imu.initialize(parameters);
        // ----------------------


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.dpad_left) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            FL.setPower(frontLeftPower);
            BL.setPower(backLeftPower);
            FranklinDelanoRoosevelt.setPower(frontRightPower);
            BoeingDriveR.setPower(backRightPower);
            // -------------------
            // INTAKE CONTROLS
            // -------------------
            if (gamepad1.b && !prevCircle) {
                intakeOneOn = !intakeOneOn;
                intakeOne.setPower(intakeOneOn ? -1 : 0);
            }
            prevCircle = gamepad1.b;

            if (gamepad1.x && !prevX) {
                intakeTwoOn = !intakeTwoOn;
                intakeTwo.setPower(intakeTwoOn ? 1 : 0);
                FastFinglerOn = !FastFinglerOn;
                FastFingler.setPower(FastFinglerOn ? 1 : 0);
            }
            prevX = gamepad1.x;

            //Emergency off
            if (gamepad1.y && !prevY) {
                FastFingler.setPower(0);
                FastFinglerOn = false;
                Shooter.setPower(0);
                shooterOn = false;
                intakeOne.setPower(0);
                intakeOneOn = false;
                intakeTwo.setPower(0);
                intakeTwoOn = false;
            }
            prevY = gamepad1.y;

            //Fast Flipper Code


            //kick balls out of robot
            if (gamepad1.dpad_up && !prevDP) {
                intakeTwoOn = !intakeTwoOn;
                intakeTwo.setPower(intakeTwoOn ? -0.7 : 0);
                intakeOneOn = !intakeOneOn;
                intakeOne.setPower(intakeOneOn ? 1 : 0);
            }
            prevDP = gamepad1.dpad_up;


            //turn shooter on
            if (gamepad1.a && !prevA) {
                shooterOn = !shooterOn;
                Shooter.setPower(shooterOn ? 1 : 0);
            }
            prevA = gamepad1.a;

        }
    }
}