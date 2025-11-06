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

    /**
     * 【已修改】更新检测数据。
     * 此方法现在会先清空旧数据，再填充新数据，解决了目标丢失后数据卡住的问题。
     */
    public void update() {
        LLResult result = limelight.getLatestResult();

        // 无论结果是否有效，都先清空上一次的列表，防止数据残留
        greenDetections.clear();
        purpleDetections.clear();

        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> allDetections = result.getDetectorResults();

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

    /**
     * 【新增方法】计算并返回离预设“锚点”物理距离最近的目标。
     *
     * @return 距离锚点最近的 LLResultTypes.DetectorResult 对象，如果没有检测到目标则返回 null。
     */
    public LLResultTypes.DetectorResult getTargetClosestToAnchor() {
        // 将绿色和紫色的检测结果合并到一个列表中
        List<LLResultTypes.DetectorResult> allDetections = new ArrayList<>();
        allDetections.addAll(greenDetections);
        allDetections.addAll(purpleDetections);

        if (allDetections.isEmpty() || this.anchorPointPhysicalCoordinates == null) {
            return null; // 如果没有任何检测结果，则返回 null
        }

        LLResultTypes.DetectorResult closestTarget = null;
        double minDistanceSquared = Double.MAX_VALUE; // 用于比较距离的最小值，初始化为最大值

        // 遍历所有检测到的目标
        for (LLResultTypes.DetectorResult detection : allDetections) {
            // 计算当前目标在地板上的物理坐标
            double[] targetCoords = calculateCoordinates(detection);

            if (targetCoords != null) {
                // 计算目标与锚点之间的误差（即物理距离）
                // 使用距离的平方进行比较可以避免开方运算，效率更高
                double errorX = targetCoords[0] - this.anchorPointPhysicalCoordinates[0];
                double errorY = targetCoords[1] - this.anchorPointPhysicalCoordinates[1];
                double distanceSquared = errorX * errorX + errorY * errorY;

                // 如果当前目标的距离比已记录的最小距离还要小，则更新最近目标
                if (distanceSquared < minDistanceSquared) {
                    minDistanceSquared = distanceSquared;
                    closestTarget = detection;
                }
            }
        }

        return closestTarget;
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
        if (closestGreen == null) { // && closestPurple == null
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