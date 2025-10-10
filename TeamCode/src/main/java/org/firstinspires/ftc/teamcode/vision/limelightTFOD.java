package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class limelightTFOD {

    private static final double CAMERA_TILT_DEGREES = 25.0; // 相机物理向下倾斜的角度(deg)
    private static final double CAMERA_HEIGHT_CM = 25.0;    // 相机镜头中心离地面的高度(cm)

    private final Limelight3A limelight;
    private List<LLResultTypes.DetectorResult> greenDetections = new ArrayList<>();
    private List<LLResultTypes.DetectorResult> purpleDetections = new ArrayList<>();

    private static final Comparator<LLResultTypes.DetectorResult> distanceComparator =
            (d1, d2) -> Double.compare(d1.getTargetYDegrees(), d2.getTargetYDegrees());

    public limelightTFOD(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
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

    public double[] calculateCoordinates(LLResultTypes.DetectorResult detection) {
        if (detection == null) {
            return null;
        }

        double totalPitchAngleDeg = CAMERA_TILT_DEGREES - detection.getTargetYDegrees();

        if (totalPitchAngleDeg <= 0 || totalPitchAngleDeg >= 90) {
            return null;
        }

        double totalPitchAngleRad = Math.toRadians(totalPitchAngleDeg);
        double yawAngleRad = Math.toRadians(detection.getTargetXDegrees());

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

    public LLStatus getStatus() {
        return limelight.getStatus();
    }
}