package org.firstinspires.ftc.teamcode.vision.QuickScope;

/**
 * 一个数据类，用于存储发射方案的计算结果。
 * 这是一个不可变对象，一旦创建，其值就不能被修改。
 */
public class LaunchSolution {
    public final double launcherVelocity;
    public final double launcherPitch;
    public final double aimAzimuth;
    public final double timeOfFlight;
    public final boolean isValid;

    public LaunchSolution(double launcherVelocity, double launcherPitch, double aimAzimuth, double timeOfFlight, boolean isValid) {
        this.launcherVelocity = launcherVelocity;
        this.launcherPitch = launcherPitch;
        this.aimAzimuth = aimAzimuth;
        this.timeOfFlight = timeOfFlight;
        this.isValid = isValid;
    }

    public static final LaunchSolution NO_SOLUTION = new LaunchSolution(0, 0, 0, 0, false);
}