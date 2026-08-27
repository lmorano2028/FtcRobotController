package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Shooter_demo", group = "Test")
public class Shooter_demo extends OpMode {

    // =========================================================
    // HARDWARE NAMES
    // =========================================================

    private static final String SHOOTER_NAME = "ShooterMotor";
    private static final String HOOD_NAME = "Shooter hood";

    private static final String TURRET_L_NAME = "LRotation";
    private static final String TURRET_R_NAME = "RRotation";


    // =========================================================
    // HARDWARE
    // =========================================================

    private DcMotorEx shooter;
    private Servo hood;

    private Servo turretL;
    private Servo turretR;

    // =========================================================
    // SHOOTER SETTINGS
    // =========================================================

    // Middle-distance speed from your main shot map
    private double shooterTargetRPM = 1155.0;

    // Each bumper press changes speed by 10
    private static final double SHOOTER_STEP = 10.0;

    // Safety limits
    private static final double SHOOTER_MIN = 900.0;
    private static final double SHOOTER_MAX = 2000.0;

    private boolean shooterEnabled = false;

    // =========================================================
    // HOOD SETTINGS
    // =========================================================

    private double hoodPos = 0.525;

    // Amount hood moves each D-pad press
    private static final double HOOD_STEP = 0.005;

    // Limits from your testing file
    private static final double HOOD_LOW_LIMIT = 0.20;
    private static final double HOOD_HIGH_LIMIT = 0.825;


    // =========================================================
    // TURRET SETTINGS
    // =========================================================

    // Starting/home position from main TeleOp
    private static final double TURRET_HOME = 0.50;

    private double turretPos = TURRET_HOME;

    // Amount turret moves each D-pad press
    private static final double TURRET_STEP = 0.01;

    // Safe limits from main TeleOp
    private static final double TURRET_MIN_LIMIT = 0.10;
    private static final double TURRET_MAX_LIMIT = 0.90;


    // =========================================================
    // BUTTON EDGE DETECTION
    // =========================================================

    private boolean prevY = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;

    private boolean prevLeftBumper = false;
    private boolean prevRightBumper = false;


    @Override
    public void init() {

        // =========================
        // HARDWARE MAP
        // =========================

        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);

        hood = hardwareMap.get(Servo.class, HOOD_NAME);

        turretL = hardwareMap.get(Servo.class, TURRET_L_NAME);
        turretR = hardwareMap.get(Servo.class, TURRET_R_NAME);


        // =========================
        // SHOOTER SETUP
        // =========================

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        // =========================
        // START POSITIONS
        // =========================

        hoodPos = clamp(
                hoodPos,
                HOOD_LOW_LIMIT,
                HOOD_HIGH_LIMIT
        );

        hood.setPosition(hoodPos);


        turretPos = clamp(
                turretPos,
                TURRET_MIN_LIMIT,
                TURRET_MAX_LIMIT
        );

        setTurretPosition(turretPos);


        // Make sure shooter starts off
        shooter.setPower(0);


        telemetry.addLine("=== SHOOTER DEMO READY ===");
        telemetry.addLine("");
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("Y = Toggle shooter ON/OFF");
        telemetry.addLine("LB = Shooter RPM -10");
        telemetry.addLine("RB = Shooter RPM +10");
        telemetry.addLine("Dpad LEFT/RIGHT = Move turret");
        telemetry.addLine("Dpad UP/DOWN = Move hood");
        telemetry.update();
    }


    @Override
    public void loop() {

        // =====================================================
        // READ BUTTONS
        // =====================================================

        boolean y = gamepad1.y;

        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

        boolean leftBumper = gamepad1.left_bumper;
        boolean rightBumper = gamepad1.right_bumper;


        // =====================================================
        // EDGE DETECTION
        // =====================================================

        boolean yPressed =
                y && !prevY;

        boolean dpadUpPressed =
                dpadUp && !prevDpadUp;

        boolean dpadDownPressed =
                dpadDown && !prevDpadDown;

        boolean dpadLeftPressed =
                dpadLeft && !prevDpadLeft;

        boolean dpadRightPressed =
                dpadRight && !prevDpadRight;

        boolean leftBumperPressed =
                leftBumper && !prevLeftBumper;

        boolean rightBumperPressed =
                rightBumper && !prevRightBumper;


        // =====================================================
        // SHOOTER RPM ADJUSTMENT
        // =====================================================

        // Left bumper = decrease RPM by 10
        if (leftBumperPressed) {

            shooterTargetRPM -= SHOOTER_STEP;

            shooterTargetRPM = clamp(
                    shooterTargetRPM,
                    SHOOTER_MIN,
                    SHOOTER_MAX
            );
        }


        // Right bumper = increase RPM by 10
        if (rightBumperPressed) {

            shooterTargetRPM += SHOOTER_STEP;

            shooterTargetRPM = clamp(
                    shooterTargetRPM,
                    SHOOTER_MIN,
                    SHOOTER_MAX
            );
        }


        // =====================================================
        // SHOOTER Y TOGGLE
        // =====================================================

        if (yPressed) {

            shooterEnabled = !shooterEnabled;

            if (!shooterEnabled) {
                shooter.setPower(0);
            }
        }


        // Keep shooter at selected RPM while enabled
        if (shooterEnabled) {

            shooter.setVelocity(shooterTargetRPM);

        } else {

            shooter.setPower(0);
        }


        // =====================================================
        // HOOD CONTROL
        // =====================================================

        // D-pad UP = hood up
        if (dpadUpPressed) {

            hoodPos += HOOD_STEP;
        }


        // D-pad DOWN = hood down
        if (dpadDownPressed) {

            hoodPos -= HOOD_STEP;
        }


        // Keep hood inside safe limits
        hoodPos = clamp(
                hoodPos,
                HOOD_LOW_LIMIT,
                HOOD_HIGH_LIMIT
        );


        hood.setPosition(hoodPos);


        // =====================================================
        // TURRET CONTROL
        // =====================================================

        // D-pad LEFT = turret left
        if (dpadLeftPressed) {

            turretPos -= TURRET_STEP;
        }


        // D-pad RIGHT = turret right
        if (dpadRightPressed) {

            turretPos += TURRET_STEP;
        }


        // Keep turret inside safe limits
        turretPos = clamp(
                turretPos,
                TURRET_MIN_LIMIT,
                TURRET_MAX_LIMIT
        );


        setTurretPosition(turretPos);


        // =====================================================
        // TELEMETRY
        // =====================================================

        telemetry.addLine("=== SHOOTER DEMO ===");

        telemetry.addLine("--- Shooter ---");
        telemetry.addData(
                "Shooter Enabled",
                shooterEnabled
        );

        telemetry.addData(
                "Target RPM",
                "%.0f",
                shooterTargetRPM
        );

        telemetry.addData(
                "Actual Velocity",
                "%.0f",
                shooter.getVelocity()
        );


        telemetry.addLine("--- Hood ---");

        telemetry.addData(
                "Hood Position",
                "%.3f",
                hoodPos
        );


        telemetry.addData(
                "Hood Limits",
                "%.3f to %.3f",
                HOOD_LOW_LIMIT,
                HOOD_HIGH_LIMIT
        );


        telemetry.addLine("--- Turret ---");

        telemetry.addData(
                "Turret Position",
                "%.3f",
                turretPos
        );


        telemetry.addData(
                "Turret Limits",
                "%.2f to %.2f",
                TURRET_MIN_LIMIT,
                TURRET_MAX_LIMIT
        );


        telemetry.addLine("");
        telemetry.addLine("Y = Shooter Toggle");
        telemetry.addLine("LB/RB = RPM -/+ 10");
        telemetry.addLine("Dpad Left/Right = Turret");
        telemetry.addLine("Dpad Up/Down = Hood");

        telemetry.update();


        // =====================================================
        // SAVE PREVIOUS BUTTON STATES
        // =====================================================

        prevY = y;

        prevDpadUp = dpadUp;
        prevDpadDown = dpadDown;
        prevDpadLeft = dpadLeft;
        prevDpadRight = dpadRight;

        prevLeftBumper = leftBumper;
        prevRightBumper = rightBumper;
    }


    // =========================================================
    // TURRET HELPER
    // =========================================================

    private void setTurretPosition(double position) {

        position = clamp(
                position,
                TURRET_MIN_LIMIT,
                TURRET_MAX_LIMIT
        );

        turretL.setPosition(position);
        turretR.setPosition(position);
    }


    // =========================================================
    // CLAMP HELPER
    // =========================================================

    private static double clamp(
            double value,
            double min,
            double max
    ) {

        return Math.max(
                min,
                Math.min(
                        max,
                        value
                )
        );
    }


    // =========================================================
    // STOP
    // =========================================================

    @Override
    public void stop() {

        shooter.setPower(0);
    }
}