package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * FieldTransform
 *
 * Single source of truth for coordinate + heading transforms between:
 *  - Pedro Pathing field space (Auto follower pose)
 *  - FTC field space (TeleOp math + goal coordinates)
 *  - Pinpoint odometry space (Pose2D: mm + radians)
 *
 * Assumptions you confirmed:
 *  - FTC +X points toward the AUDIENCE
 *  - FTC +Y points toward BLUE alliance side
 *  - FTC heading 0° points toward +X (audience), CCW positive
 *  - Heading values wrap in [-180, +180]
 *
 * Your proven Auto mapping (Pedro -> FTC):
 *  - FTC_X(in) = 72 - Pedro_Y
 *  - FTC_Y(in) = Pedro_X - 72
 *  - FTC_HeadingDeg = wrapDeg180(PedroHeadingDeg + 90)
 *
 * Pinpoint:
 *  - We treat Pinpoint X/Y axes as matching FTC axes (no mirroring)
 *  - Pinpoint reports Pose2D in MM + RADIANS
 *
 * If anything ever changes, update ONLY this file.
 */
public final class FieldTransform {

    private FieldTransform() { /* no instances */ }

    // ----------------------------
    // Constants
    // ----------------------------
    public static final double FIELD_HALF_IN = 72.0;

    /** Pedro heading -> FTC heading offset (deg). Validated: FTC = Pedro + 90 */
    public static final double PEDRO_TO_FTC_HEADING_OFFSET_DEG = 90.0;

    /** Unit conversions */
    public static final double IN_TO_MM = 25.4;
    public static final double MM_TO_IN = 1.0 / 25.4;

    // ----------------------------
    // Units
    // ----------------------------
    public static double inToMm(double in) { return in * IN_TO_MM; }
    public static double mmToIn(double mm) { return mm * MM_TO_IN; }

    // ----------------------------
    // Pedro -> FTC (Auto seeding)
    // ----------------------------

    /** Convert Pedro Y (in) to FTC X (in). */
    public static double pedroToFtcX(double pedroY) {
        return FIELD_HALF_IN - pedroY;
    }

    /** Convert Pedro X (in) to FTC Y (in). */
    public static double pedroToFtcY(double pedroX) {
        return pedroX - FIELD_HALF_IN;
    }

    /** Convert Pedro heading (deg) to FTC heading (deg), wrapped to [-180, 180]. */
    public static double pedroToFtcHeadingDeg(double pedroHeadingDeg) {
        return wrapDeg180(pedroHeadingDeg + PEDRO_TO_FTC_HEADING_OFFSET_DEG);
    }

    // ----------------------------
    // FTC <-> Pinpoint (axes/sign)
    // ----------------------------
    // In your current system Pinpoint axes match FTC axes (no mirroring).
    // Keep these as explicit constants so the assumption lives here.

    public static final double PINPOINT_TO_FTC_X_SIGN = +1.0;
    public static final double PINPOINT_TO_FTC_Y_SIGN = +1.0;

    /** Pinpoint X (in) -> FTC X (in). */
    public static double pinpointToFtcX(double xPinIn) { return PINPOINT_TO_FTC_X_SIGN * xPinIn; }

    /** Pinpoint Y (in) -> FTC Y (in). */
    public static double pinpointToFtcY(double yPinIn) { return PINPOINT_TO_FTC_Y_SIGN * yPinIn; }

    /** FTC X (in) -> Pinpoint X (in). */
    public static double ftcToPinpointX(double xFtcIn)  { return PINPOINT_TO_FTC_X_SIGN * xFtcIn; }

    /** FTC Y (in) -> Pinpoint Y (in). */
    public static double ftcToPinpointY(double yFtcIn)  { return PINPOINT_TO_FTC_Y_SIGN * yFtcIn; }

    // ----------------------------
    // PoseStorage <-> Pinpoint Pose2D
    // ----------------------------

    /**
     * PoseStorage (FTC inches/deg) -> Pose2D for Pinpoint (mm/rad).
     * Use this in TeleOp start() to seed Pinpoint.
     */
    public static Pose2D poseStorageToPinpointPose2D() {
        double xPinIn = ftcToPinpointX(PoseStorage.xIn);
        double yPinIn = ftcToPinpointY(PoseStorage.yIn);

        double xMm = inToMm(xPinIn);
        double yMm = inToMm(yPinIn);

        double hRad = degToRadWrapped(PoseStorage.headingDeg);

        return new Pose2D(DistanceUnit.MM, xMm, yMm, AngleUnit.RADIANS, hRad);
    }

    /** Pinpoint Pose2D (mm/rad) -> FTC X (in). */
    public static double pinpointPoseToFtcXIn(Pose2D p) {
        double xMm = p.getX(DistanceUnit.MM);
        double xInPin = mmToIn(xMm);
        return pinpointToFtcX(xInPin);
    }

    /** Pinpoint Pose2D (mm/rad) -> FTC Y (in). */
    public static double pinpointPoseToFtcYIn(Pose2D p) {
        double yMm = p.getY(DistanceUnit.MM);
        double yInPin = mmToIn(yMm);
        return pinpointToFtcY(yInPin);
    }

    /** Pinpoint Pose2D (mm/rad) -> FTC heading degrees wrapped [-180, 180]. */
    public static double pinpointPoseToFtcHeadingDeg(Pose2D p) {
        double hRad = p.getHeading(AngleUnit.RADIANS);
        return radToDegWrapped(hRad);
    }

    // ----------------------------
    // Angle helpers
    // ----------------------------

    /** Wrap degrees into [-180, 180]. */
    public static double wrapDeg180(double deg) {
        while (deg > 180.0) deg -= 360.0;
        while (deg < -180.0) deg += 360.0;
        return deg;
    }

    /** Wrap radians into [-pi, pi]. */
    public static double wrapRadPi(double rad) {
        while (rad > Math.PI) rad -= (2.0 * Math.PI);
        while (rad < -Math.PI) rad += (2.0 * Math.PI);
        return rad;
    }

    public static double radToDegWrapped(double rad) { return wrapDeg180(Math.toDegrees(rad)); }
    public static double degToRadWrapped(double deg) { return wrapRadPi(Math.toRadians(deg)); }

    // ----------------------------
    // Convenience helpers
    // ----------------------------

    /** Write PoseStorage from a Pedro pose (x,y inches, heading degrees). */
    public static void writePoseStorageFromPedro(double pedroX, double pedroY, double pedroHeadingDeg) {
        PoseStorage.valid = true;
        PoseStorage.xIn = pedroToFtcX(pedroY);
        PoseStorage.yIn = pedroToFtcY(pedroX);
        PoseStorage.headingDeg = pedroToFtcHeadingDeg(pedroHeadingDeg);
    }

    public static void clearPoseStorage() {
        PoseStorage.clear();
    }
}
