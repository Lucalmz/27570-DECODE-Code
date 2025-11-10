// Filename: ChassisLocalizationTeleOp.java
package org.firstinspires.ftc.teamcode.vision.QuickScope; // 使用您指定的包名

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx; // 新增: 导入DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.vision.EchoLapse.IPoseProvider;
import org.firstinspires.ftc.teamcode.vision.EchoLapse.PinpointPoseProvider; // 确保路径正确

import java.util.Locale;

@TeleOp(name = "Archer 自动瞄准系统 (V8.1 - 带电机控制)", group = "Main") // 修改: 版本号更新
public class ChassisLocalizationTeleOp extends LinearOpMode {

    private AprilTagLocalizer aprilTagLocalizer;
    private IPoseProvider pinpointPoseProvider;
    private Servo LeftPitch;
    private Servo RightPitch;
    private Servo RightBoard;
    private Servo LeftBoard;

    // --- 新增: 声明发射电机 ---
    private DcMotorEx LeftShooter;
    private DcMotorEx RightShooter;
    private DcMotorEx InhaleMotor;
    private DcMotorEx IntakeMotor;


    // --- 新增: 电机物理参数常量 ---
    public static final double SHOOTER_TICKS_PER_REV = 28;
    public static final double SHOOTER_GEAR_RATIO = 1.0; // 1:1

    private ArcherLogic archerLogic;

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. 初始化设备
        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap);
        pinpointPoseProvider = new PinpointPoseProvider(hardwareMap, "odo");
        RightPitch = hardwareMap.get(Servo.class, "RightPitch");
        LeftPitch = hardwareMap.get(Servo.class, "LeftPitch");
        RightBoard = hardwareMap.get(Servo.class, "RightBoard");
        LeftBoard = hardwareMap.get(Servo.class, "LeftBoard");
        pinpointPoseProvider.initialize();

        // --- 新增: 初始化和配置发射电机 ---
        LeftShooter = hardwareMap.get(DcMotorEx.class, "LeftShooter");
        RightShooter = hardwareMap.get(DcMotorEx.class, "RightShooter");
        InhaleMotor = hardwareMap.get(DcMotorEx.class, "InhaleMotor");
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");

        // 为确保发射物能直线飞出，通常一个电机需要反向
        // 请根据您的实际安装情况调整 FORWARD 或 REVERSE
        LeftShooter.setDirection(DcMotor.Direction.REVERSE);
        RightShooter.setDirection(DcMotor.Direction.REVERSE);

        // 设置为使用编码器进行速度控制
        LeftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // 设置PIDF系数 (P, I, D, F)
        // 注意: setVelocityPIDFCoefficients的参数顺序是 P, I, D, F
        // 您提供的LeftShooter: p160, i0, f18, d90 -> P=160, I=0, D=90, F=18
        LeftShooter.setVelocityPIDFCoefficients(160, 0, 90, 18);
        // 您提供的RightShooter: p170, i0, f15.85, d20 -> P=170, I=0, D=20, F=15.85
        RightShooter.setVelocityPIDFCoefficients(170, 0, 20, 15.85);


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
            return;
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
        String targetAlliance = "Red";

        while (opModeIsActive()) {

            // --- A. 获取机器人实时状态 (输入) ---
            pinpointPoseProvider.update();
            double robotX_cm = -pinpointPoseProvider.getX(DistanceUnit.CM);
            double robotY_cm = pinpointPoseProvider.getY(DistanceUnit.CM);

            double normalizationX = robotX_cm / 365.76;
            double normalizationY = robotY_cm / 365.76;

            double cartesianVelX_m_s = -pinpointPoseProvider.getXVelocity(DistanceUnit.MM) / 1000.0;
            double cartesianVelY_m_s = pinpointPoseProvider.getYVelocity(DistanceUnit.MM)
                    / 1000.0;
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
            CalculationParams currentParams = new CalculationParams(
                    normalizationX,
                    normalizationY,
                    speed_m_s,
                    direction_deg,
                    targetAlliance
            );

            LaunchSolution solution = archerLogic.calculateSolution(currentParams);

            // --- E. 遥测数据显示与硬件控制 ---
            telemetry.addLine("--- Archer自动瞄准系统 (V8.1) ---");
            telemetry.addData("当前目标", "%s Alliance (按X/B切换)", targetAlliance);

            double finalpitch = 90 - solution.launcherAngle;
            double leftservoposition = finalpitch * 0.028571428571428 - 0.05;
            double rightservoposition = 1 - leftservoposition - 0.007;

            if (solution != null) {
                telemetry.addLine(">> 方案已解算 <<");
                telemetry.addData("发射电机转速 (RPM)", "%.0f", solution.motorRpm);
                telemetry.addData("偏航角 (Yaw)", "%.2f deg", solution.aimAzimuthDeg);
                telemetry.addData("俯仰角 (Pitch)", "%.2f deg", finalpitch);
                telemetry.addData("LeftPitch","%.5f deg", leftservoposition);
                telemetry.addData("RightPitch","%.5f deg", rightservoposition);
                LeftBoard.setPosition(0.82);
                RightBoard.setPosition(0.618);
                LeftPitch.setPosition(leftservoposition);
                RightPitch.setPosition(rightservoposition);

                // --- 修改: 在这里将计算结果应用到您的硬件 ---
                double targetRpm = solution.motorRpm;
                double ticksPerSecond = rpmToTicksPerSecond(targetRpm);

                // 设置电机目标速度
                LeftShooter.setVelocity(ticksPerSecond);
                RightShooter.setVelocity(ticksPerSecond);
                IntakeMotor.setPower(1);
                InhaleMotor.setPower(0.55);

                // --- 新增: 在遥测中显示电机目标和当前速度，方便调试 ---
                telemetry.addData("电机目标速度 (Ticks/Sec)", "%.2f", ticksPerSecond);
                telemetry.addData("LeftShooter 当前速度", "%.2f Ticks/s", LeftShooter.getVelocity());
                telemetry.addData("RightShooter 当前速度", "%.2f Ticks/s", RightShooter.getVelocity());

            } else {
                telemetry.addLine(">> 目标超出射程或无解 <<");

                // --- 新增: 如果没有解，则停止电机 ---
                LeftShooter.setVelocity(0);
                RightShooter.setVelocity(0);
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

        // --- 新增: 确保在OpMode结束时电机停止 ---
        LeftShooter.setVelocity(0);
        RightShooter.setVelocity(0);
    }

    private String formatPose(Pose2D pose) {
        if (pose == null) return "null";
        return String.format(Locale.US, "X: %.2f cm, Y: %.2f cm, H: %.2f deg",
                pose.getX(DistanceUnit.CM), pose.getY(DistanceUnit.CM), pose.getHeading(AngleUnit.DEGREES));
    }

    // --- 新增: 辅助方法，将RPM转换为Ticks per Second ---
    /**
     * 将电机转速 (RPM) 转换为SDK使用的编码器刻度/秒 (Ticks per Second)
     * @param rpm Revolutions Per Minute
     * @return Ticks Per Second
     */
    private static double rpmToTicksPerSecond(double rpm) {
        return (rpm * SHOOTER_TICKS_PER_REV * SHOOTER_GEAR_RATIO) / 60.0;
    }
}