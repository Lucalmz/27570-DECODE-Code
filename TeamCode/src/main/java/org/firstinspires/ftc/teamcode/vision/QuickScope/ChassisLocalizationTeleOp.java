// Filename: ChassisLocalizationTeleOp.java
package org.firstinspires.ftc.teamcode.vision.QuickScope; // 使用您指定的包名

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.vision.EchoLapse.PinpointPoseProvider; // 确保路径正确

import java.util.Locale;

@TeleOp(name = "Archer 自动瞄准系统 (V8.0 - 单线程阻塞版)", group = "Main")
public class ChassisLocalizationTeleOp extends LinearOpMode {

    private AprilTagLocalizer aprilTagLocalizer;
    private PinpointPoseProvider pinpointPoseProvider;

    // --- 修改: 移除了多线程相关的成员 ---
    private ArcherLogic archerLogic;

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. 初始化设备
        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap);
        pinpointPoseProvider = new PinpointPoseProvider(hardwareMap, "odo");
        pinpointPoseProvider.initialize();

        // --- 修改: 只需初始化计算逻辑对象，无需创建线程 ---
        archerLogic = new ArcherLogic();

        telemetry.addLine("初始化完成，请将机器人对准AprilTag");
        telemetry.addData("模式", "仅在开始时进行一次AprilTag校准");
        telemetry.update();

        // 2. 等待开始
        while (!isStarted() && !isStopRequested()) {
            Pose2D initialPose = aprilTagLocalizer.getRobotPose();
            if (initialPose != null) {
                telemetry.addLine(">> 已成功检测到AprilTag! <<");
                telemetry.addData("检测到的初始位置", formatPose(initialPose));
                telemetry.addLine("松开手柄，随时可以按Start开始...");
            } else {
                telemetry.addLine("正在寻找AprilTag...");
                telemetry.addLine("请将摄像头对准场上的AprilTag");
            }
            telemetry.update();
            sleep(20);
        }

        if (isStopRequested()) {
            return; // 如果在初始化时就停止，直接退出
        }

        // 3. 初始校准
        telemetry.addLine("正在进行最后一次AprilTag初始校准...");
        telemetry.update();

        Pose2D aprilTagInitialPose = aprilTagLocalizer.getRobotPose();
        if (aprilTagInitialPose != null) {
            pinpointPoseProvider.setPose(aprilTagInitialPose);
            telemetry.addLine(">> 校准成功! AprilTag坐标已设置为里程计初始值 <<");
            telemetry.addData("初始坐标", formatPose(aprilTagInitialPose));
        } else {
            telemetry.addLine(">> 未找到AprilTag! 里程计将从(0,0,0)开始 <<");
        }
        telemetry.update();
        sleep(1000);

        // 4. 关闭AprilTag检测以节省资源
        aprilTagLocalizer.close();

        // 5. 主循环
        String targetAlliance = "Red"; // 默认目标为红色联盟

        while (opModeIsActive()) {

            // --- A. 获取机器人实时状态 (输入) ---
            pinpointPoseProvider.update();
            double robotX_cm = -pinpointPoseProvider.getX(DistanceUnit.CM);
            double robotY_cm = pinpointPoseProvider.getY(DistanceUnit.CM);

            double normalizationX = robotX_cm / 365.76;
            double normalizationY = robotY_cm / 365.76;

            double cartesianVelX_m_s = -pinpointPoseProvider.getXVelocity(DistanceUnit.MM) / 1000.0;
            double cartesianVelY_m_s = pinpointPoseProvider.getYVelocity(DistanceUnit.MM) / 1000.0;
            double speed_m_s = Math.hypot(cartesianVelX_m_s, cartesianVelY_m_s);
            double direction_rad = Math.atan2(cartesianVelY_m_s, cartesianVelX_m_s);
            double direction_deg = Math.toDegrees(direction_rad);
            if (direction_deg < 0) direction_deg += 360;

            // --- B. 获取手柄输入 ---
            if (gamepad1.x) {
                targetAlliance = "Blue";
            }
            if (gamepad1.b) {
                targetAlliance = "Red";
            }
            if (gamepad1.a) {
                pinpointPoseProvider.reset();
                telemetry.addLine("里程计已手动重置为 (0,0,0)!");
            }
            // 您的底盘驾驶代码应该放在这里...

            // --- C & D. 直接进行阻塞式计算 ---
            // 将所有实时状态打包成一个对象
            CalculationParams currentParams = new CalculationParams(
                    normalizationX,
                    normalizationY,
                    speed_m_s,
                    direction_deg,
                    targetAlliance
            );

            // 直接调用计算方法。主循环会在此处暂停，直到计算完成。
            LaunchSolution solution = archerLogic.calculateSolution(currentParams);

            // --- E. 遥测数据显示 ---
            telemetry.addLine("--- Archer自动瞄准系统 (单线程版) ---");
            telemetry.addData("当前目标", "%s Alliance (按X/B切换)", targetAlliance);

            if (solution != null) {
                telemetry.addLine(">> 方案已解算 <<");
                telemetry.addData("发射电机转速 (RPM)", "%.0f", solution.motorRpm);
                telemetry.addData("偏航角 (Yaw)", "%.2f deg", solution.aimAzimuthDeg);
                telemetry.addData("俯仰角 (Pitch)", "%.2f deg", solution.launcherAngle);

                // TODO: 在这里将计算结果应用到您的硬件
                // shooterMotor.setRPM(solution.motorRpm);
                // turretServo.setAngle(solution.aimAzimuthDeg);
                // pitchServo.setAngle(solution.launcherAngle);

            } else {
                telemetry.addLine(">> 目标超出射程或无解 <<");
            }

            telemetry.addLine("\n--- 机器人状态 ---");
            String positionData = String.format(Locale.US, "{X_norm: %.3f, Y_norm: %.3f}",
                    normalizationX,
                    normalizationY
            );
            String velocityData = String.format(Locale.US,"{Speed: %.2f m/s, Dir: %.1f deg}",
                    speed_m_s,
                    direction_deg
            );
            telemetry.addData("归一化坐标", positionData);
            telemetry.addData("实时速度", velocityData);
            telemetry.addData("Pinpoint状态", pinpointPoseProvider.getDeviceStatus());
            telemetry.update();

            sleep(20);
        }

        // 6. 结束时无需清理线程
    }

    private String formatPose(Pose2D pose) {
        if (pose == null) return "null";
        return String.format(Locale.US, "X: %.2f cm, Y: %.2f cm, H: %.2f deg",
                pose.getX(DistanceUnit.CM), pose.getY(DistanceUnit.CM), pose.getHeading(AngleUnit.DEGREES));
    }
}