package org.firstinspires.ftc.teamcode.limelgiht;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

@TeleOp(name="Test Limelight AprilTags Distance (Verbose)", group="Test")
public class TestLimelightAprilTagsDistance extends LinearOpMode {

    private static final int APRILTAG_PIPELINE_INDEX = 0;

    // DECODE goal tags
    private static final int BLUE_GOAL_ID = 20;
    private static final int RED_GOAL_ID  = 24;

    // If you want inches everywhere, set true. (Internally we keep mm.)
    private static final boolean SHOW_INCHES_TOO = true;

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(50);

        limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
        limelight.start();

        telemetry.addLine("INIT done. Press PLAY.");
        telemetry.addLine("Pipeline must be AprilTags + Full 3D ON + Marker Size = 165.10mm (for DECODE PDF black square).");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            boolean tagSeen = false;
            int tagId = -1;

            double tx = Double.NaN;
            double ty = Double.NaN;

            // Poses
            Pose3D camPoseTargetSpace = null;
            Pose3D robotPoseTargetSpace = null;

            // Extracted translations (mm)
            double camXmm = Double.NaN, camYmm = Double.NaN, camZmm = Double.NaN;
            double robXmm = Double.NaN, robYmm = Double.NaN, robZmm = Double.NaN;

            // Distances
            double camDist3Dmm = Double.NaN;
            double camDistZmm  = Double.NaN;

            double robDist3Dmm = Double.NaN;
            double robDistZmm  = Double.NaN;

            if (result != null && result.isValid()) {
                tx = result.getTx();
                ty = result.getTy();

                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (tags != null && !tags.isEmpty()) {
                    tagSeen = true;

                    // Prefer goal tags (20/24)
                    LLResultTypes.FiducialResult chosen = null;
                    for (LLResultTypes.FiducialResult t : tags) {
                        int id = (int) t.getFiducialId();
                        if (id == BLUE_GOAL_ID || id == RED_GOAL_ID) {
                            chosen = t;
                            break;
                        }
                    }
                    if (chosen == null) chosen = tags.get(0);
                    tagId = (int) chosen.getFiducialId();

                    // Get both poses if available
                    try { camPoseTargetSpace = chosen.getCameraPoseTargetSpace(); } catch (Exception ignored) {}
                    try { robotPoseTargetSpace = chosen.getRobotPoseTargetSpace(); } catch (Exception ignored) {}

                    // --- Camera pose translation (mm) ---
                    if (camPoseTargetSpace != null) {
                        Position p = camPoseTargetSpace.getPosition();
                        camXmm = DistanceUnit.MM.fromUnit(p.unit, p.x);
                        camYmm = DistanceUnit.MM.fromUnit(p.unit, p.y);
                        camZmm = DistanceUnit.MM.fromUnit(p.unit, p.z);

                        camDist3Dmm = Math.sqrt(camXmm*camXmm + camYmm*camYmm + camZmm*camZmm);
                        camDistZmm  = Math.abs(camZmm); // forward/back magnitude
                    }

                    // --- Robot pose translation (mm) ---
                    if (robotPoseTargetSpace != null) {
                        Position p = robotPoseTargetSpace.getPosition();
                        robXmm = DistanceUnit.MM.fromUnit(p.unit, p.x);
                        robYmm = DistanceUnit.MM.fromUnit(p.unit, p.y);
                        robZmm = DistanceUnit.MM.fromUnit(p.unit, p.z);

                        robDist3Dmm = Math.sqrt(robXmm*robXmm + robYmm*robYmm + robZmm*robZmm);
                        robDistZmm  = Math.abs(robZmm);
                    }
                }
            }

            telemetry.addData("Pipeline", APRILTAG_PIPELINE_INDEX);
            telemetry.addData("Tag Seen", tagSeen);
            telemetry.addData("Tag ID", tagId);
            telemetry.addData("tx (deg)", Double.isNaN(tx) ? "N/A" : String.format("%.2f", tx));
            telemetry.addData("ty (deg)", Double.isNaN(ty) ? "N/A" : String.format("%.2f", ty));

            telemetry.addLine("---- CameraPoseTargetSpace ----");
            if (!Double.isNaN(camXmm)) {
                telemetry.addData("CamX (mm)", String.format("%.1f", camXmm));
                telemetry.addData("CamY (mm)", String.format("%.1f", camYmm));
                telemetry.addData("CamZ (mm)", String.format("%.1f", camZmm));
                telemetry.addData("Cam Z-dist (mm)", String.format("%.1f", camDistZmm));
                telemetry.addData("Cam 3D-dist (mm)", String.format("%.1f", camDist3Dmm));
                if (SHOW_INCHES_TOO) {
                    telemetry.addData("Cam Z-dist (in)", String.format("%.2f", camDistZmm / 25.4));
                    telemetry.addData("Cam 3D-dist (in)", String.format("%.2f", camDist3Dmm / 25.4));
                }
            } else {
                telemetry.addLine("Cam pose: N/A (Full 3D OFF or pose not provided)");
            }

            telemetry.addLine("---- RobotPoseTargetSpace ----");
            if (!Double.isNaN(robXmm)) {
                telemetry.addData("RobX (mm)", String.format("%.1f", robXmm));
                telemetry.addData("RobY (mm)", String.format("%.1f", robYmm));
                telemetry.addData("RobZ (mm)", String.format("%.1f", robZmm));
                telemetry.addData("Rob Z-dist (mm)", String.format("%.1f", robDistZmm));
                telemetry.addData("Rob 3D-dist (mm)", String.format("%.1f", robDist3Dmm));
                if (SHOW_INCHES_TOO) {
                    telemetry.addData("Rob Z-dist (in)", String.format("%.2f", robDistZmm / 25.4));
                    telemetry.addData("Rob 3D-dist (in)", String.format("%.2f", robDist3Dmm / 25.4));
                }
            } else {
                telemetry.addLine("Robot pose: N/A (robot offsets not configured OR pose not provided)");
            }

            telemetry.addLine("---- Sanity Check (Robot - Camera) ----");
            if (!Double.isNaN(camXmm) && !Double.isNaN(robXmm)) {
                telemetry.addData("dX mm", String.format("%.1f", (robXmm - camXmm)));
                telemetry.addData("dY mm", String.format("%.1f", (robYmm - camYmm)));
                telemetry.addData("dZ mm", String.format("%.1f", (robZmm - camZmm)));
                telemetry.addLine("If dX/dY/dZ are ~0, your Limelight 'robot offset' is NOT set (robot pose == camera pose).");
            } else {
                telemetry.addLine("Need both poses to compare.");
            }

            telemetry.update();
        }

        limelight.stop();
    }
}
