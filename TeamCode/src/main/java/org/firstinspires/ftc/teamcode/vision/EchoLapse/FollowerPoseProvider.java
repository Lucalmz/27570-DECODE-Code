package org.firstinspires.ftc.teamcode.vision.EchoLapse;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class FollowerPoseProvider implements IPoseProvider {
    Follower follower;
    public FollowerPoseProvider (Follower follower){
        this.follower = follower;
    }
    /**
     * 更新从硬件读取的最新数据。在每次循环中都应调用此方法。
     */
    @Override
    public void update() {
        follower.update();
    }

    /**
     * 设置机器人当前的位置
     *
     * @param pose 类型为Pose2D的pose
     */
    @Override
    public void setPose(Pose2D pose) {
        follower.setPose(new Pose(pose.getY(DistanceUnit.INCH),-pose.getX(DistanceUnit.INCH),pose.getHeading(AngleUnit.RADIANS)));
    }

    /**
     * 初始化硬件
     */
    @Override
    public void initialize() {
        follower.breakFollowing();
        reset();
    }

    /**
     * 获取机器人当前在X轴上的位置。
     *
     * @param unit 距离单位。
     * @return X轴坐标。
     */
    @Override
    public double getX(DistanceUnit unit) {
        if(unit == DistanceUnit.INCH){
            return follower.getPose().getY();
        }
        double x = follower.getPose().getY()*25.4;
        if(unit == DistanceUnit.MM){
            return x;
        }
        if(unit == DistanceUnit.CM){
            return x/10;
        }
        if(unit == DistanceUnit.METER){
            return x/1000;
        }
        throw new IllegalArgumentException("UNKNOWN UNIT");
    }

    /**
     * 获取机器人当前在Y轴上的位置。
     *
     * @param unit 距离单位。
     * @return Y轴坐标。
     */
    @Override
    public double getY(DistanceUnit unit) {
        if(unit == DistanceUnit.INCH){
            return follower.getPose().getX();
        }
        double x = follower.getPose().getX()*25.4;
        if(unit == DistanceUnit.MM){
            return x;
        }
        if(unit == DistanceUnit.CM){
            return x/10;
        }
        if(unit == DistanceUnit.METER){
            return x/1000;
        }
        throw new IllegalArgumentException("UNKNOWN UNIT");
    }

    /**
     * 获取机器人当前的航向角。
     *
     * @param unit 角度单位。
     * @return 航向角。
     */
    @Override
    public double getHeading(AngleUnit unit) {
        if(unit == AngleUnit.RADIANS){
            return follower.getHeading();
        }
        return Math.toDegrees(follower.getHeading());
    }

    /**
     * 获取机器人当前在X轴上的速度。
     *
     * @param unit 距离单位。
     * @return X轴方向的速度。
     */
    @Override
    public double getXVelocity(DistanceUnit unit) {
        if(unit == DistanceUnit.INCH){
            return follower.getVelocity().getYComponent();
        }
        double x = follower.getVelocity().getYComponent()*25.4;
        if(unit == DistanceUnit.MM){
            return x;
        }
        if(unit == DistanceUnit.CM){
            return x/10;
        }
        if(unit == DistanceUnit.METER){
            return x/1000;
        }
        throw new IllegalArgumentException("UNKNOWN UNIT");
    }

    /**
     * 获取机器人当前在Y轴上的速度。
     *
     * @param unit 距离单位。
     * @return Y轴方向的速度。
     */
    @Override
    public double getYVelocity(DistanceUnit unit) {
        if(unit == DistanceUnit.INCH){
            return follower.getVelocity().getXComponent();
        }
        double x = follower.getVelocity().getXComponent()*25.4;
        if(unit == DistanceUnit.MM){
            return x;
        }
        if(unit == DistanceUnit.CM){
            return x/10;
        }
        if(unit == DistanceUnit.METER){
            return x/1000;
        }
        throw new IllegalArgumentException("UNKNOWN UNIT");
    }

    /**
     * 获取机器人当前的角速度。
     *
     * @param unit 角度单位。
     * @return 围绕Z轴旋转的角速度。
     */
    @Override
    public double getHeadingVelocity(AngleUnit unit) {
        if(unit == AngleUnit.RADIANS){
            return follower.getAngularVelocity();
        }
        return Math.toDegrees(follower.getAngularVelocity());
    }

    /**
     * 重置位置和IMU。
     */
    @Override
    public void reset() {
        follower.breakFollowing();
        follower.setPose(new Pose(0,0,0));
        follower.update();
    }

    /**
     * 获取设备的运行状态。
     *
     * @return 状态字符串。
     */
    @Override
    public String getDeviceStatus() {
        return "";
    }

    /**
     * 获取设备的数据更新频率。
     *
     * @return 频率 (Hz)。
     */
    @Override
    public double getUpdateFrequency() {
        return 0;
    }
}