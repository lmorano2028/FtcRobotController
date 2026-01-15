package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

public class PoseStorage {
    public static boolean valid = false;

    // FTC field coordinates (INCHES, DEGREES)
    public static double xIn = 0.0;
    public static double yIn = 0.0;
    public static double headingDeg = 0.0;

    public static void clear() {
        valid = false;
    }
}

