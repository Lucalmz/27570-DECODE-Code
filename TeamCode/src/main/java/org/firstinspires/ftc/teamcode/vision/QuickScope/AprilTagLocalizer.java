package org.firstinspires.ftc.teamcode.vision.QuickScope;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * [坐标系转换版] 封装了AprilTag定位功能的独立类。
 * 此版本包含了将AprilTag默认的“中心原点”坐标系转换为我们期望的“左下角原点”坐标系的关键逻辑。
 */
public class AprilTagLocalizer {
    private static final Position cameraPosition = new Position(DistanceUnit.CM, 17, 15, 15.8, 0);
    private static final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 180, -57, 90, 0);

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private static final double HALF_FIELD_MM = 72 * 25.4;

    public AprilTagLocalizer(HardwareMap hwMap) {
        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.MM, AngleUnit.DEGREES)
                .setNumThreads(4)
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        aprilTag.setDecimation(1);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * 核心方法：获取当前机器人经过坐标系转换后的、正确的Pose2D。
     * @return 如果看到任何有效的AprilTag，则返回一个精确的Pose2D；否则返回null。
     */
    public Pose2D getRobotPose() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.robotPose != null) {
                double aprilTagX = detection.robotPose.getPosition().x;
                double aprilTagY = detection.robotPose.getPosition().y;
                double aprilTagYaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

                double finalX_mm = aprilTagY + HALF_FIELD_MM;
                double finalY_mm = -aprilTagX + HALF_FIELD_MM;
                double finalHeading_deg = aprilTagYaw - 90;

                return new Pose2D(
                        DistanceUnit.MM,
                        finalX_mm,
                        finalY_mm,
                        AngleUnit.DEGREES,
                        finalHeading_deg
                );
            }
        }

        return null;
    }

    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}