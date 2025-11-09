// Filename: ChassisLocalizationTeleOp.java
package org.firstinspires.ftc.teamcode.vision.QuickScope; // 使用您指定的包名

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.vision.EchoLapse.PinpointPoseProvider; // 确保路径正确

import java.util.Locale;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.atomic.AtomicReference;

@TeleOp(name = "Archer 自动瞄准系统 (V8.0)", group = "Main")
public class ChassisLocalizationTeleOp extends LinearOpMode {

    private AprilTagLocalizer aprilTagLocalizer;
    private PinpointPoseProvider pinpointPoseProvider;

    // --- 新增: Archer计算相关 ---
    private ArcherLogic archerLogic;
    private Thread calculationThread;
    // 使用线程安全的AtomicReference来存储最新的计算结果, 避免主线程和计算线程同时读写造成冲突
    private final AtomicReference<LaunchSolution> latestSolution = new AtomicReference<>(null);
    // 使用线程安全的队列来从主线程向计算线程传递参数
    private final BlockingQueue<CalculationParams> calculationQueue = new ArrayBlockingQueue<>(1);
    // --- Archer计算相关结束 ---


    @Override
    public void runOpMode() throws InterruptedException {
        // 1. 初始化设备
        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap);
        pinpointPoseProvider = new PinpointPoseProvider(hardwareMap, "odo");
        pinpointPoseProvider.initialize();

        // --- 新增: 初始化Archer逻辑和后台线程 ---
        // 这是至关重要的一步：将计算密集型任务放到后台线程。
        // 这样可以防止主控制循环(while opModeIsActive)被阻塞，确保机器人手柄操作流畅。
        archerLogic = new ArcherLogic();
        calculationThread = new Thread(() -> {
            while (!Thread.currentThread().isInterrupted()) {
                try {
                    // take()方法会阻塞(等待)，直到队列中有新的参数可以取出
                    CalculationParams params = calculationQueue.take();
                    // 调用纯数学计算核心
                    LaunchSolution solution = archerLogic.calculateSolution(params);
                    // 将计算结果存放到线程安全的位置，供主线程读取
                    latestSolution.set(solution);
                } catch (InterruptedException e) {
                    // 当OpMode结束时，我们会中断这个线程，这里是退出循环的逻辑
                    Thread.currentThread().interrupt(); // 恢复中断状态
                    break;
                }
            }
        });
        calculationThread.start(); // 启动后台计算线程
        // --- 初始化结束 ---

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
            calculationThread.interrupt(); // 如果在初始化时就停止，确保线程被关闭
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
        String targetAlliance = "Red"; // 默认目标为红色联盟

        while (opModeIsActive()) {

            // --- A. 获取机器人实时状态 (输入) ---
            pinpointPoseProvider.update();
            // 场地总宽度为12英尺 = 365.76厘米. 将cm单位的坐标转换为(0.0 - 1.0)的归一化坐标
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
            // 使用X键选择蓝色联盟，B键选择红色联盟
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
            // double drive = -gamepad1.left_stick_y;
            // ...


            // --- C. 发送参数给计算核心 ---
            // 将所有实时状态打包成一个对象
            CalculationParams currentParams = new CalculationParams(
                    normalizationX,
                    normalizationY,
                    speed_m_s,
                    direction_deg,
                    targetAlliance
            );
            // 清空队列并放入最新的参数。这确保后台线程总是在处理最新的机器人状态，避免延迟。
            calculationQueue.clear();
            calculationQueue.offer(currentParams);


            // --- D. 获取并使用计算结果 (输出) ---
            LaunchSolution solution = latestSolution.get(); // 从线程安全区获取最新结果，这个操作极快，不会阻塞

            // --- E. 遥测数据显示 ---
            telemetry.addLine("--- Archer自动瞄准系统 ---");
            telemetry.addData("当前目标", "%s Alliance (按X/B切换)", targetAlliance);

            if (solution != null) {
                telemetry.addLine(">> 方案已解算 <<");
                telemetry.addData("发射电机转速 (RPM)", "%.0f", solution.motorRpm);
                telemetry.addData("偏航角 (Yaw)", "%.2f deg", solution.aimAzimuthDeg);
                telemetry.addData("俯仰角 (Pitch)", "%.2f deg", solution.launcherAngle);

                // TODO: 在这里将计算结果应用到您的硬件
                // 例如:
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

        // 6. 结束时清理
        calculationThread.interrupt(); // OpMode结束，确保后台线程被正确关闭
    }

    private String formatPose(Pose2D pose) {
        if (pose == null) return "null";
        return String.format(Locale.US, "X: %.2f cm, Y: %.2f cm, H: %.2f deg",
                pose.getX(DistanceUnit.CM), pose.getY(DistanceUnit.CM), pose.getHeading(AngleUnit.DEGREES));
    }
}