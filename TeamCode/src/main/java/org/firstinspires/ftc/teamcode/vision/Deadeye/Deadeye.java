package org.firstinspires.ftc.teamcode.vision.Deadeye;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class Deadeye {

    public static final double LL_VERTICAL_FOV_DEGREES = 49.7;

    private static final double CAMERA_TILT_DEGREES = 43; // 相机物理向下倾斜的角度(deg)
    private static final double CAMERA_HEIGHT_CM = 23.5;    // 相机镜头中心离地面的高度(cm)

    // 常量最后更新日期 2025/11/3
    private static final double ANCHOR_POINT_TY = -LL_VERTICAL_FOV_DEGREES / 2.0;

    private final Limelight3A limelight;
    private List<LLResultTypes.DetectorResult> greenDetections = new ArrayList<>();
    private List<LLResultTypes.DetectorResult> purpleDetections = new ArrayList<>();

    private final double[] anchorPointPhysicalCoordinates;

    private static final Comparator<LLResultTypes.DetectorResult> distanceComparator =
            Comparator.comparingDouble(LLResultTypes.DetectorResult::getTargetYDegrees);

    public Deadeye(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.anchorPointPhysicalCoordinates = calculateCoordinatesFromAngles(0, ANCHOR_POINT_TY);
        limelight.pipelineSwitch(0);
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public void update() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> allDetections = result.getDetectorResults();

            greenDetections.clear();
            purpleDetections.clear();

            for (LLResultTypes.DetectorResult detection : allDetections) {
                if ("green".equals(detection.getClassName())) {
                    greenDetections.add(detection);
                } else if ("purple".equals(detection.getClassName())) {
                    purpleDetections.add(detection);
                }
            }

            greenDetections.sort(distanceComparator);
            purpleDetections.sort(distanceComparator);
        }
    }

    public double[] calculateAlignmentError(LLResultTypes.DetectorResult targetDetection) {
        if (targetDetection == null || this.anchorPointPhysicalCoordinates == null) {
            return null;
        }

        double[] targetCoords = calculateCoordinates(targetDetection);
        if (targetCoords == null) {
            return null;
        }

        double errorX = targetCoords[0] - this.anchorPointPhysicalCoordinates[0];
        double errorY = targetCoords[1] - this.anchorPointPhysicalCoordinates[1];

        return new double[]{errorX, errorY};
    }

    public double[] calculateCoordinates(LLResultTypes.DetectorResult detection) {
        if (detection == null) {
            return null;
        }
        return calculateCoordinatesFromAngles(detection.getTargetXDegrees(), detection.getTargetYDegrees());
    }

    private double[] calculateCoordinatesFromAngles(double tx, double ty) {
        double totalPitchAngleDeg = CAMERA_TILT_DEGREES - ty;

        if (totalPitchAngleDeg <= 0 || totalPitchAngleDeg >= 90) {
            return null;
        }

        double totalPitchAngleRad = Math.toRadians(totalPitchAngleDeg);
        double yawAngleRad = Math.toRadians(tx);

        double y_cm = CAMERA_HEIGHT_CM / Math.tan(totalPitchAngleRad);
        double x_cm = y_cm * Math.tan(yawAngleRad);

        return new double[]{x_cm, y_cm};
    }

    public List<LLResultTypes.DetectorResult> getGreenDetections() {
        return greenDetections;
    }

    public List<LLResultTypes.DetectorResult> getPurpleDetections() {
        return purpleDetections;
    }

    public LLResultTypes.DetectorResult getClosestGreen() {
        return greenDetections.isEmpty() ? null : greenDetections.get(0);
    }

    public LLResultTypes.DetectorResult getClosestPurple() {
        return purpleDetections.isEmpty() ? null : purpleDetections.get(0);
    }

    public LLResultTypes.DetectorResult getSecondClosestPurple() {
        return purpleDetections.size() < 2 ? null : purpleDetections.get(1);
    }

    public LLResultTypes.DetectorResult getClosestTarget() {
        LLResultTypes.DetectorResult closestGreen = getClosestGreen();
        LLResultTypes.DetectorResult closestPurple = getClosestPurple();

        if (closestGreen == null && closestPurple != null) {
            return closestPurple;
        }
        if (closestPurple == null && closestGreen != null) {
            return closestGreen;
        }
        if (closestGreen == null && closestPurple == null) {
            return null;
        }

        if (closestGreen.getTargetYDegrees() > closestPurple.getTargetYDegrees()) {
            return closestGreen;
        } else {
            return closestPurple;
        }
    }

    public LLStatus getStatus() {
        return limelight.getStatus();
    }
}