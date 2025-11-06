package org.firstinspires.ftc.teamcode.vision.QuickScope;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import java.util.concurrent.locks.ReentrantLock;

/**
 * 核心计算类，负责根据物理模型找到最佳发射方案。
 * 这个类的计算方法被设计为在后台线程中运行，以避免阻塞主OpMode循环。
 */
public class LaunchCalculator {

    public static final double GRAVITY_MS2 = 9.81;                 // 重力加速度 (m/s^2)
    public static final double AIR_DENSITY = 1.225;                // 空气密度 (kg/m^3)
    public static final double TARGET_HEIGHT_M = 1.065;            // 目标高度 (米)
    public static final double PROJECTILE_MASS_KG = 0.012;         // 弹丸质量 (kg)
    public static final double DRAG_COEFFICIENT = 0.25;            // 风阻系数
    public static final double CROSS_SECTIONAL_AREA_M2 = 0.00928;  // 横截面积 (m^2)

    public static final double MIN_ANGLE_DEG = 55.0;               // 最小搜索角度 (度)
    public static final double MAX_ANGLE_DEG = 90.0;               // 最大搜索角度 (度)
    public static final double ANGLE_SEARCH_STEP = 1.0;            // 角度搜索步长 (度)
    public static final double VELOCITY_SEARCH_STEP = 0.1;         // 速度搜索步长 (m/s)
    public static final int MAX_VELOCITY_TRIES = 500;              // 最大速度尝试次数
    public static final int BISECTION_ITERATIONS = 8;              // 二分法迭代次数，用于提高精度
    public static final double HIT_TOLERANCE_M = 0.055;            // 命中容差 (米)
    public static final double TIME_STEP_S = 0.006;                // 物理模拟的时间步长 (秒)

    public static final double MOTOR_RPM_LOSS_FACTOR_PERCENT = 0; // 估算电机RPM时的损耗因子 (%)
    public static final double FRICTION_WHEEL_DIAMETER_M = 0.072;    // 摩擦轮直径 (米)

    private volatile boolean isCalculating = false;
    private ReentrantLock lock = new ReentrantLock();

    private volatile LaunchSolution lastSolution = LaunchSolution.NO_SOLUTION;
    private volatile boolean newSolutionAvailable = false;

    public boolean isBusy() {
        return isCalculating;
    }

    public boolean hasNewSolution() {
        return newSolutionAvailable;
    }

    public LaunchSolution getSolution() {
        newSolutionAvailable = false;
        return lastSolution;
    }

    public void calculateAsync(LaunchParameters params) {
        lock.lock();
        try{
            isCalculating = true;
            lastSolution = findLaunchSolution(params);
            newSolutionAvailable = true;
            isCalculating = false;
        }finally {
            lock.unlock();
        }
    }

    public double calculateMotorRps(double velocityMs) {
        if (velocityMs <= 0) return 0;
        double theoreticalRps = velocityMs/ (Math.PI * FRICTION_WHEEL_DIAMETER_M);
        return theoreticalRps * (1.0 + MOTOR_RPM_LOSS_FACTOR_PERCENT);
    }

    public static class LaunchParameters {
        public final double distanceM;
        public final double vehicleSpeedMs;
        public final double vehicleDirectionRad;
        public final double targetDirectionRad;

        public LaunchParameters(double distanceM, double vehicleSpeedMs, double vehicleDirectionRad, double targetDirectionRad) {
            this.distanceM = distanceM;
            this.vehicleSpeedMs = vehicleSpeedMs;
            this.vehicleDirectionRad = vehicleDirectionRad;
            this.targetDirectionRad = targetDirectionRad;
        }
    }

    private double runSimulationForAngleAndVelocity(double launchAngleDeg, double initialVelocityMs, double distanceM) {
        double angleRad = toRadians(launchAngleDeg);
        double vx = initialVelocityMs * cos(angleRad);
        double vy = initialVelocityMs * sin(angleRad);
        double x = 0.0, y = 0.0;

        double dragFactor = 0.5 * AIR_DENSITY * DRAG_COEFFICIENT * CROSS_SECTIONAL_AREA_M2;
        double gravityForceY = -PROJECTILE_MASS_KG * GRAVITY_MS2;

        double prevX = 0.0, prevY = 0.0;

        while (true) {
            if ((vx <= 0 && x < distanceM) || (y < 0 && vy < 0)) {
                return -1.0;
            }

            prevX = x;
            prevY = y;

            double vSq = vx * vx + vy * vy;
            if (vSq == 0) return -1.0;
            double v = sqrt(vSq);

            double dragForce = dragFactor * vSq;
            double ax = -dragForce * (vx / v) / PROJECTILE_MASS_KG;
            double ay = (gravityForceY - dragForce * (vy / v)) / PROJECTILE_MASS_KG;

            vx += ax * TIME_STEP_S;
            vy += ay * TIME_STEP_S;
            x += vx * TIME_STEP_S;
            y += vy * TIME_STEP_S;

            if (x >= distanceM) {
                if (x - prevX == 0) return y;
                double fraction = (distanceM - prevX) / (x - prevX);
                return prevY + (y - prevY) * fraction;
            }
        }
    }

    private double estimateInitialVelocity(double angleDeg, double targetX, double targetY) {
        double angleRad = toRadians(angleDeg);
        double cosA = cos(angleRad);
        double tanA = tan(angleRad);
        double denominator = 2 * (cosA * cosA) * (targetX * tanA - targetY);
        if (denominator <= 0) return -1;
        return sqrt((GRAVITY_MS2 * targetX * targetX) / denominator);
    }

    private LaunchSolution findLaunchSolution(LaunchParameters params) {
        if (params.distanceM <= 0) return LaunchSolution.NO_SOLUTION;

        double vVehicleX = params.vehicleSpeedMs * cos(params.vehicleDirectionRad);
        double vVehicleY = params.vehicleSpeedMs * sin(params.vehicleDirectionRad);

        LaunchSolution bestSolution = null;
        double lastMinLauncherVelocity = Double.POSITIVE_INFINITY;

        for (double projectileVerticalAngle = MIN_ANGLE_DEG; projectileVerticalAngle <= MAX_ANGLE_DEG; projectileVerticalAngle += ANGLE_SEARCH_STEP) {
            double predictedV = estimateInitialVelocity(projectileVerticalAngle, params.distanceM, TARGET_HEIGHT_M);
            if (predictedV < 0) continue;

            double lowV = predictedV, highV = 0.0;
            boolean foundBracket = false;
            for (int i = 0; i < MAX_VELOCITY_TRIES; i++) {
                double testV = lowV + i * VELOCITY_SEARCH_STEP;
                double hitH = runSimulationForAngleAndVelocity(projectileVerticalAngle, testV, params.distanceM);
                if (hitH > TARGET_HEIGHT_M) {
                    highV = testV;
                    lowV = Math.max(predictedV, testV - VELOCITY_SEARCH_STEP);
                    foundBracket = true;
                    break;
                }
            }
            if (!foundBracket) continue;

            for (int i = 0; i < BISECTION_ITERATIONS; i++) {
                double midV = (lowV + highV) / 2.0;
                if (midV <= 0) break;
                double midH = runSimulationForAngleAndVelocity(projectileVerticalAngle, midV, params.distanceM);
                if (midH > TARGET_HEIGHT_M) {
                    highV = midV;
                } else {
                    lowV = midV;
                }
            }

            double projectileTotalVelocity = highV;
            double finalH = runSimulationForAngleAndVelocity(projectileVerticalAngle, projectileTotalVelocity, params.distanceM);

            if (abs(finalH - TARGET_HEIGHT_M) <= HIT_TOLERANCE_M) {
                double vProjectileH_mag = projectileTotalVelocity * cos(toRadians(projectileVerticalAngle));

                double vProjectileHX = vProjectileH_mag * cos(params.targetDirectionRad);
                double vProjectileHY = vProjectileH_mag * sin(params.targetDirectionRad);

                double vLauncherHX = vProjectileHX - vVehicleX;
                double vLauncherHY = vProjectileHY - vVehicleY;

                double vLauncherH_mag = sqrt(vLauncherHX * vLauncherHX + vLauncherHY * vLauncherHY);
                double aimAzimuthRad = atan2(vLauncherHY, vLauncherHX);

                double vLauncherV = projectileTotalVelocity * sin(toRadians(projectileVerticalAngle));

                double launcherVelocity = sqrt(vLauncherH_mag * vLauncherH_mag + vLauncherV * vLauncherV);
                double launcherPitchDeg = toDegrees(atan2(vLauncherV, vLauncherH_mag));

                if (launcherVelocity < lastMinLauncherVelocity) {
                    lastMinLauncherVelocity = launcherVelocity;
                    bestSolution = new LaunchSolution(
                            launcherVelocity,
                            launcherPitchDeg,
                            toDegrees(aimAzimuthRad),
                            0,
                            true
                    );
                } else {
                    break;
                }
            }
        }
        return bestSolution != null ? bestSolution : LaunchSolution.NO_SOLUTION;
    }
}