
package org.firstinspires.ftc.teamcode.limelgiht;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.lang.reflect.Method;
import java.util.List;

/**
 * DROP-IN TEST TELEOP (FTC SDK 11 / Android 16 friendly)
 * - Fixes Pose3D import (uses FTC Pose3D)
 * - Avoids getTranslation() (uses pose.getPosition())
 * - Works even if your FiducialResult pose getter name differs (uses reflection to find one)
 *
 * REQUIREMENTS:
 * 1) In Robot Configuration, add Limelight as "limelight" (exact name)
 * 2) Pipeline index below must match your AprilTag pipeline (0 by default)
 */
@TeleOp(name="Test Limelight AprilTags (Drop-In)", group="Test")
public class TestLimelightAprilTag extends LinearOpMode {

    // Set this to your AprilTag pipeline index in Limelight UI
    private static final int APRILTAG_PIPELINE_INDEX = 0;

    // Your DECODE goal tag IDs
    private static final int BLUE_GOAL_ID = 20;
    private static final int RED_GOAL_ID  = 24;

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        telemetry.setMsTransmissionInterval(50);

        // Switch to your AprilTag pipeline and start
        limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
        limelight.start();

        telemetry.addLine("Limelight started.");
        telemetry.addLine("Point at tag 20 (blue) or 24 (red). Press PLAY.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            boolean tagSeen = false;
            int bestId = -1;
            double bestDistanceMm = Double.NaN;

            double tx = Double.NaN;
            double ty = Double.NaN;

            if (result != null && result.isValid()) {
                tx = result.getTx();
                ty = result.getTy();

                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (tags != null && !tags.isEmpty()) {

                    // Pick the first tag that is one of our goal tags (20 or 24).
                    // If neither is present, just use the first detected tag.
                    LLResultTypes.FiducialResult chosen = null;
                    for (LLResultTypes.FiducialResult t : tags) {
                        int id = (int) t.getFiducialId();
                        if (id == BLUE_GOAL_ID || id == RED_GOAL_ID) {
                            chosen = t;
                            break;
                        }
                    }
                    if (chosen == null) chosen = tags.get(0);

                    tagSeen = true;
                    bestId = (int) chosen.getFiducialId();

                    // Get a Pose3D from the tag using whichever getter exists in YOUR library build
                    Pose3D poseCam = tryGetAnyPose3D(chosen);

                    if (poseCam != null) {
                        // FTC Pose3D -> Position. Z is typically forward distance in CAMERA space.
                        Position p = poseCam.getPosition();
                        // Convert p.z from whatever units the Pose3D is using to mm
                        bestDistanceMm = DistanceUnit.MM.fromUnit(p.unit, p.z);
                    }
                }
            }

            telemetry.addData("Pipeline", APRILTAG_PIPELINE_INDEX);
            telemetry.addData("Tag Seen", tagSeen);
            telemetry.addData("Tag ID", bestId);
            telemetry.addData("tx (deg)", fmt(tx));
            telemetry.addData("ty (deg)", fmt(ty));

            if (!Double.isNaN(bestDistanceMm)) {
                telemetry.addData("Distance (mm)", String.format("%.1f", bestDistanceMm));
                telemetry.addData("Distance (in)", String.format("%.2f", bestDistanceMm / 25.4));
            } else {
                telemetry.addData("Distance", "N/A (pose getter not found or not returned)");
                telemetry.addLine("If Distance is N/A: your FiducialResult pose method name differs.");
                telemetry.addLine("This code already tries common names; share your 'tag.' autocomplete list if needed.");
            }

            telemetry.update();
        }

        limelight.stop();
    }

    /**
     * Some Limelight FTC library builds expose the tag pose with different method names.
     * This tries several common ones and returns the first Pose3D it can obtain.
     *
     * NOTE: This compiles even if some methods don't exist because we use reflection.
     */
    private Pose3D tryGetAnyPose3D(LLResultTypes.FiducialResult tag) {
        // Common method names seen across different limelight ftc library versions
        String[] candidates = new String[] {
                "getTargetPoseCameraSpace",
                "getTargetPose_CameraSpace",
                "getCameraPoseTargetSpace",
                "getCameraPose_TargetSpace",
                "getRobotPoseTargetSpace",
                "getRobotPose_TargetSpace",
                "getTargetPoseRobotSpace",
                "getTargetPose_RobotSpace"
        };

        for (String name : candidates) {
            Pose3D pose = invokePose3DGetter(tag, name);
            if (pose != null) return pose;
        }
        return null;
    }

    private Pose3D invokePose3DGetter(Object obj, String methodName) {
        try {
            Method m = obj.getClass().getMethod(methodName);
            Object out = m.invoke(obj);
            if (out instanceof Pose3D) {
                return (Pose3D) out;
            }
        } catch (Exception ignored) {
            // Method not found or invocation failed; try next
        }
        return null;
    }

    private String fmt(double v) {
        if (Double.isNaN(v)) return "N/A";
        return String.format("%.2f", v);
    }
}
