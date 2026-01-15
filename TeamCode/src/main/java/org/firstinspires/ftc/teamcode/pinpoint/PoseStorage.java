package org.firstinspires.ftc.teamcode.pinpoint;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PoseStorage {
    public static volatile boolean valid = false;

    // Stored in mm and degrees for convenience
    public static volatile double xMm = 0.0;
    public static volatile double yMm = 0.0;
    public static volatile double headingDeg = 0.0;

    public static void writeFromPose(Pose2D p) {
        xMm = p.getX(DistanceUnit.MM);
        yMm = p.getY(DistanceUnit.MM);
        headingDeg = p.getHeading(AngleUnit.DEGREES);
        valid = true;
    }

    public static Pose2D readAsPose2D() {
        return new Pose2D(
                DistanceUnit.MM, xMm, yMm,
                AngleUnit.DEGREES, headingDeg
        );
    }

    public static void clear() {
        valid = false;
        xMm = yMm = headingDeg = 0.0;
    }
}
