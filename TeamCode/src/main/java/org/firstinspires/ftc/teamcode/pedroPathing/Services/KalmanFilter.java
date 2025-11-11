package org.firstinspires.ftc.teamcode.pedroPathing.Services;

public class KalmanFilter {
    private double x = 0; // 状态估计值
    private double p = 1; // 估计的不确定性
    private final double q; // 过程噪声
    private final double r; // 测量噪声

    public KalmanFilter(double initialValue, double initialUncertainty, double processNoise, double measurementNoise) {
        this.x = initialValue;
        this.p = initialUncertainty;
        this.q = processNoise;
        this.r = measurementNoise;
    }

    public double update(double measurement) {
        p += q;
        double k = p / (p + r);
        x += k * (measurement - x);
        p *= (1 - k);
        return x;
    }
}