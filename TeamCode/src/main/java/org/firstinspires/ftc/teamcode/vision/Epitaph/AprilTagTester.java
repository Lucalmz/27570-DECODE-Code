package org.firstinspires.ftc.teamcode.vision.Epitaph;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.vision.QuickScope.AprilTagLocalizer;

/**
 * 这是一个独立的测试OpMode，专门用于验证AprilTagLocalizer类的功能。
 *
 * 功能：
 * 1. 实时显示通过 getSpikeMarkLocation() 检测到的Spike Mark位置 (1, 2, 或 3)。
 * 2. 实时显示通过 getRobotPose() 计算出的机器人场地坐标。
 *
 * 如何使用：
 * 1. 编译并部署此OpMode到机器人控制器。
 * 2. 在司机站上选择 "AprilTagTester" OpMode。
 * 3. 初始化(Init)并运行(Start)OpMode。
 * 4. 将摄像头对准ID为21, 22, 23的AprilTag，观察 "Spike Mark Location" 的变化。
 * 5. 将摄像头对准场地上的任意AprilTag，观察 "Robot Pose" 是否输出合理的坐标。
 */
@TeleOp(name = "AprilTag Tester", group = "Vision")
public class AprilTagTester extends LinearOpMode {

    // 声明我们要测试的类
    private AprilTagLocalizer aprilTagLocalizer;

    @Override
    public void runOpMode() {
        // 1. 初始化
        // 在OpMode启动时创建AprilTagLocalizer的实例
        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap);

        telemetry.addLine("AprilTagLocalizer Initialized.");
        telemetry.addLine("Point camera at tags and press START.");
        telemetry.update();

        // 等待司机按下开始按钮
        waitForStart();

        // 2. 主循环
        while (opModeIsActive()) {
            // --- 测试新功能: getSpikeMarkLocation() ---
            int spikeLocation = aprilTagLocalizer.getSpikeMarkLocation();

            telemetry.addLine("--- Spike Mark Detection ---");
            if (spikeLocation == 0) {
                telemetry.addData("Spike Mark Location", "Not Found (Searching for ID 21, 22, 23)");
            } else {
                // 使用 %d 占位符来显示整数值
                telemetry.addData("Spike Mark Location FOUND!", "Position %d", spikeLocation);
            }
            telemetry.addLine(); // 添加一个空行以作分隔

            // --- 测试原有功能: getRobotPose() ---
            Pose2D robotPose = aprilTagLocalizer.getRobotPose();

            telemetry.addLine("--- Robot Pose Localization ---");
            // 必须检查robotPose是否为null，因为如果看不到任何tag，它会返回null
            if (robotPose != null) {
                // 使用 %.2f 格式化字符串，让坐标只显示两位小数，更易读
                String poseString = String.format(
                        "X: %.2f mm, Y: %.2f mm, Heading: %.2f deg",
                        robotPose.getX(DistanceUnit.MM),
                        robotPose.getY(DistanceUnit.MM),
                        robotPose.getHeading(AngleUnit.DEGREES)
                );
                telemetry.addData("Robot Pose", poseString);
            } else {
                telemetry.addLine("Robot Pose: No valid AprilTag detected for localization.");
            }
            telemetry.addLine(); // 添加一个空行

            // 更新遥测数据到司机站屏幕
            telemetry.update();

            // 让CPU休息一下，避免循环过快消耗过多资源
            sleep(20);
        }

        // 3. 清理
        // 当OpMode停止时（用户按下Stop按钮），确保关闭VisionPortal
        aprilTagLocalizer.close();
    }
}