// 文件名: PinpointAutoAimOpMode.java
package org.firstinspires.ftc.teamcode.vision.QuickScope;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.vision.EchoLapse.GoBildaPinpointDriver;

import java.util.Locale;

@TeleOp(name = "Pinpoint 自瞄 OpMode (全自动版)", group = "Linear OpMode")
public class PinpointAutoAimOpMode extends LinearOpMode {

    private GoBildaPinpointDriver odo;
    private LaunchCalculator calculator;
    private LaunchSolution latestSolution = LaunchSolution.NO_SOLUTION;
    private AprilTagLocalizer localizer;

    private static final Pose2D BLUE_ALLIANCE_TARGET = new Pose2D(DistanceUnit.MM, 927.1, 3657.6, AngleUnit.DEGREES, 0);
    private static final Pose2D RED_ALLIANCE_TARGET = new Pose2D(DistanceUnit.MM, 2730.5, 3657.6, AngleUnit.DEGREES, 0);

    private final Pose2D currentTarget = BLUE_ALLIANCE_TARGET;
    private final String currentTargetName = "Blue";

    @Override
    public void runOpMode() {
        // --- 初始化硬件、里程计、计算器、定位器 (与之前版本相同) ---
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(85.0, -180.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        calculator = new LaunchCalculator();
        localizer = new AprilTagLocalizer(hardwareMap);

        // --- 初始化定位循环 (与之前版本相同) ---
        while (!isStarted() && !isStopRequested()) {
            Pose2D initialPose = localizer.getRobotPose();
            if (initialPose != null) {
                odo.setPosition(initialPose);
                telemetry.addData("状态", "定位成功！可以开始！");
                telemetry.addData("初始位置 (X, Y, H)", String.format(Locale.US, "%.1f, %.1f, %.1f°",
                        initialPose.getX(DistanceUnit.MM),
                        initialPose.getY(DistanceUnit.MM),
                        initialPose.getHeading(AngleUnit.DEGREES)));
            } else {
                telemetry.addData("状态", "正在等待定位...");
                telemetry.addData("提示", "请将机器人摄像头对准任一AprilTag！");
            }
            telemetry.update();
            sleep(50);
        }

        if (isStopRequested()) {
            localizer.close();
            return;
        }

        while (opModeIsActive()) {
            odo.update();
            Pose2D currentPose = odo.getPosition();

            if (!calculator.isBusy()) {
                double deltaX = currentTarget.getX(DistanceUnit.MM) - currentPose.getX(DistanceUnit.MM);
                double deltaY = currentTarget.getY(DistanceUnit.MM) - currentPose.getY(DistanceUnit.MM);
                double distanceM = Math.hypot(deltaX, deltaY) / 1000.0;
                double targetDirectionRad = Math.atan2(deltaY, deltaX);

                double currentVelX_Mms = odo.getVelX(DistanceUnit.MM);
                double currentVelY_Mms = odo.getVelY(DistanceUnit.MM);
                double vehicleSpeedMs = Math.hypot(currentVelX_Mms, currentVelY_Mms) / 1000.0;
                double vehicleDirectionRad = Math.atan2(currentVelY_Mms, currentVelX_Mms);

                LaunchCalculator.LaunchParameters params = new LaunchCalculator.LaunchParameters(
                        distanceM, vehicleSpeedMs, vehicleDirectionRad, targetDirectionRad);

                calculator.calculateAsync(params);
            }

            if (calculator.hasNewSolution()) {
                latestSolution = calculator.getSolution();
            }

            if (latestSolution.isValid) {
            }

            displayTelemetry(currentPose);
        }

        localizer.close();
    }

    private void displayTelemetry(Pose2D pos) {
        telemetry.addData("--- 机器人状态 (坐标系: 左下角原点) ---", "");
        telemetry.addData("位置 (X, Y, H)", String.format(Locale.US, "%.1f, %.1f, %.1f°",
                pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES)));
        telemetry.addData("速度 (X, Y)", String.format(Locale.US, "%.2f, %.2f m/s",
                odo.getVelX(DistanceUnit.MM) / 1000.0, odo.getVelY(DistanceUnit.MM) / 1000.0));

        telemetry.addData("\n--- 自瞄系统 (全自动模式) ---", "");
        telemetry.addData("当前目标", currentTargetName + String.format(Locale.US, " (%.0f, %.0f)",
                currentTarget.getX(DistanceUnit.MM), currentTarget.getY(DistanceUnit.MM)));
        telemetry.addData("计算器状态", calculator.isBusy() ? "计算中..." : "空闲");

        if (latestSolution.isValid) {
            telemetry.addData("\n--- 发射方案 [有效] ---", "");
            telemetry.addData("俯仰角 (Pitch)", String.format(Locale.US, "%.2f°", latestSolution.launcherPitch));
            telemetry.addData("绝对航向角 (Azimuth)", String.format(Locale.US, "%.2f°", latestSolution.aimAzimuth));
            telemetry.addData("发射速度", String.format(Locale.US, "%.2f m/s", latestSolution.launcherVelocity));
            telemetry.addData("估算RPM", String.format(Locale.US, "%.0f", calculator.calculateMotorRpm(latestSolution.launcherVelocity)));
        } else {
            telemetry.addData("\n--- 发射方案 [无效] ---", "");
            telemetry.addData("提示", "当前位置无解或未计算");
        }
        telemetry.update();
    }
}