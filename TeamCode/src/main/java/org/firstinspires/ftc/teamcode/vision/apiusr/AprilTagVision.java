package org.firstinspires.ftc.teamcode.vision.apiusr;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.vision.WebcamAprilTag;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

@TeleOp(name = "AprilTagVision", group = "Concept")
public class AprilTagVision extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;
    private static final String WEBCAM_NAME = "Webcam 1";

    private WebcamAprilTag aprilTagVision;

    @Override
    public void runOpMode() {
        aprilTagVision = new WebcamAprilTag(hardwareMap, USE_WEBCAM, WEBCAM_NAME);

        telemetry.addData(">", "Robot Ready. Press Play to start detecting AprilTags.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                List<AprilTagDetection> currentDetections = aprilTagVision.getDetections();
                telemetry.addData("AprilTags Detected", currentDetections.size());
                telemetry.addLine();
                processDetections(currentDetections);

                telemetry.update();

                sleep(20);
            }
        }

        aprilTagVision.close();
    }

    private void processDetections(List<AprilTagDetection> detections) {
        boolean foundId21 = false;
        boolean foundId22 = false;
        boolean foundId23 = false;

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                if (detection.id == 24 || detection.id == 20) {
                    if (detection.ftcPose != null) {
                        String direction = detection.ftcPose.bearing < 0 ? "右" : "左";
                        telemetry.addLine(String.format("ID %d 的偏移角度: %6.1f 度 (在摄像头中心%s侧)",
                                detection.id, detection.ftcPose.bearing, direction));
                    }
                }
                if (detection.id == 21) foundId21 = true;
                if (detection.id == 22) foundId22 = true;
                if (detection.id == 23) foundId23 = true;
            }
        }

        telemetry.addLine("\n--- 墓碑： ---");
        telemetry.addData("是否识别到 ID 21", foundId21 ? "是" : "否");
        telemetry.addData("是否识别到 ID 22", foundId22 ? "是" : "否");
        telemetry.addData("是否识别到 ID 23", foundId23 ? "是" : "否");
    }
}