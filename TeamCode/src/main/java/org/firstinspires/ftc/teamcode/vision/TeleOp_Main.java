// 文件名: TeleOp_Main.java
package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

// 导入所有必需的自定义类
import org.firstinspires.ftc.teamcode.vision.Deadeye.Deadeye;
import org.firstinspires.ftc.teamcode.vision.EchoLapse.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.vision.QuickScope.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.vision.QuickScope.LaunchCalculator;
import org.firstinspires.ftc.teamcode.vision.QuickScope.LaunchSolution;

import java.util.Locale;

@TeleOp(name = "TeleOp Persistent Localization", group = "Linear Opmode")
public class TeleOp_Main extends LinearOpMode {

    // --- 硬件设备声明 ---
    private DcMotor LeftFrontMotor, LeftBehindMotor, RightFrontMotor, RightBehindMotor;
    private DcMotor InhaleMotor, IntakeMotor;
    private CRServo ClassifyServo;
    private Deadeye deadeye;
    private GoBildaPinpointDriver odo;
    private AprilTagLocalizer localizer;
    private LaunchCalculator calculator;
    private LaunchSolution latestSolution = LaunchSolution.NO_SOLUTION;
    private Servo RightPitch, LeftPitch;

    // --- 状态变量 ---
    private boolean isIntakeMode = false;
    private boolean rightBumperPreviouslyPressed = false;
    private boolean isLaunchMode = false;
    private boolean leftBumperPreviouslyPressed = false;
    private double lastClassifyServoPower = 0.0;
    private boolean isLocalized = false;

    // --- 常量与调节参数 ---
    private static final double SERVO_ACTIVATION_DISTANCE_CM = 15.0;
    private static final double INTAKE_Y_GAIN = 0.03;
    private static final double INTAKE_X_GAIN = 0.04;
    private static final Pose2D BLUE_ALLIANCE_TARGET = new Pose2D(DistanceUnit.MM, 927.1, 3657.6, AngleUnit.DEGREES, 0);
    private static final Pose2D RED_ALLIANCE_TARGET = new Pose2D(DistanceUnit.MM, 2730.5, 3657.6, AngleUnit.DEGREES, 0);
    private final Pose2D currentTarget = BLUE_ALLIANCE_TARGET;

    // --- PD控制器参数 ---
    private static final double HEADING_P_GAIN = 0.0; // 您可能需要重新微调这些值
    private static final double HEADING_D_GAIN = 0.00; // D增益通常比P增益小
    private double lastHeadingError = 0;
    private ElapsedTime pidTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        initializeHardware();

        // --- 初始化阶段的定位尝试 (仍然保留) ---
        while (!isStarted() && !isStopRequested()) {
            Pose2D initialPose = localizer.getRobotPose();
            if (initialPose != null) {
                odo.setPosition(initialPose);
                isLocalized = true;
                gamepad1.stopRumble();
                telemetry.addData("状态", "定位成功！随时可以开始！");
                telemetry.addData("初始位置 (X, Y, H)", String.format(Locale.US, "%.1f, %.1f, %.1f°",
                        initialPose.getX(DistanceUnit.MM), initialPose.getY(DistanceUnit.MM), initialPose.getHeading(AngleUnit.DEGREES)));
            } else {
                isLocalized = false;
                gamepad1.rumble(0.5, 0.5, 200);
                telemetry.addData("状态", "正在等待定位... 请微调机器人位置");
            }
            telemetry.update();
            sleep(50);
        }

        if (isStopRequested()) {
            localizer.close();
            return;
        }

        waitForStart();

        // --- 主循环 ---
        while (opModeIsActive()) {
            // 首先更新传感器
            odo.update();
            deadeye.update();

            if (!isLocalized) {
                Pose2D currentPose = localizer.getRobotPose();
                if (currentPose != null) {
                    odo.setPosition(currentPose);
                    isLocalized = true;
                    gamepad1.stopRumble();
                } else {
                    gamepad1.rumble(0.5, 0.5, 50);
                }
            }

            // 模式切换
            handleToggles();

            // 获取手柄输入 (原始的、以机器人为中心的输入)
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_trigger - gamepad1.left_trigger;

            // 如果尚未定位，则减速
            if (!isLocalized) {
                drive *= 0.5;
                strafe *= 0.5;
            }

            // --- 【核心修改】全局无头模式 (Field-Centric Drive) ---
            // 获取当前已经校正过的航向角（弧度）
            double currentHeadingRad = odo.getHeading(AngleUnit.RADIANS);
            // 根据航向角旋转手柄输入向量，得到场地坐标系下的期望运动方向
            double rotatedStrafe = strafe * Math.cos(-currentHeadingRad) - drive * Math.sin(-currentHeadingRad);
            double rotatedDrive = strafe * Math.sin(-currentHeadingRad) + drive * Math.cos(-currentHeadingRad);

            // 根据模式执行相应逻辑
            if (isLaunchMode) {
                // 在发射模式中，手动转向被PD控制器覆盖
                turn = runLaunchModeLogic();
                // 瞄准时忽略行驶输入，以提高精度
                rotatedDrive = 0;
                rotatedStrafe = 0;
            } else if (isIntakeMode) {
                // 吸入模式现在也受益于无头模式的驾驶
                double[] intakePowers = runIntakeModeLogic(rotatedDrive, rotatedStrafe);
                rotatedDrive = intakePowers[0];
                rotatedStrafe = intakePowers[1];
            } else {
                runManualModeLogic();
            }

            // 底盘驱动和功率归一化 (使用旋转后的值)
            double leftFrontPower = rotatedDrive + rotatedStrafe + turn;
            double rightFrontPower = rotatedDrive - rotatedStrafe - turn;
            double leftBehindPower = rotatedDrive - rotatedStrafe + turn;
            double rightBehindPower = rotatedDrive + rotatedStrafe - turn;

            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBehindPower));
            max = Math.max(max, Math.abs(rightBehindPower));
            if (max > 1.0) {
                leftFrontPower /= max; rightFrontPower /= max;
                leftBehindPower /= max; rightBehindPower /= max;
            }

            LeftFrontMotor.setPower(leftFrontPower);
            RightFrontMotor.setPower(rightFrontPower);
            LeftBehindMotor.setPower(leftBehindPower);
            RightBehindMotor.setPower(rightBehindPower);

            // 显示遥测数据
            displayTelemetry();
        }

        localizer.close();
    }

    // --- 【方法已修改】---
    private double runLaunchModeLogic() {
        Pose2D currentPose = odo.getPosition();
        if (!calculator.isBusy()) {
            double deltaX = currentTarget.getX(DistanceUnit.MM) - currentPose.getX(DistanceUnit.MM);
            double deltaY = currentTarget.getY(DistanceUnit.MM) - currentPose.getY(DistanceUnit.MM);
            double distanceM = Math.hypot(deltaX, deltaY) / 1000.0;
            double targetDirectionRad = Math.atan2(deltaY, deltaX);
            double vehicleSpeedMs = Math.hypot(odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM)) / 1000.0;
            double vehicleDirectionRad = Math.atan2(odo.getVelY(DistanceUnit.MM), odo.getVelX(DistanceUnit.MM));
            LaunchCalculator.LaunchParameters params = new LaunchCalculator.LaunchParameters(
                    distanceM, vehicleSpeedMs, vehicleDirectionRad, targetDirectionRad);
            new Thread(() -> calculator.calculateAsync(params)).start();
        }

        if (calculator.hasNewSolution()) {
            latestSolution = calculator.getSolution();
        }
        if (latestSolution.isValid) {
            setLauncherPitch(latestSolution.launcherPitch);
        }

        InhaleMotor.setPower(0);
        IntakeMotor.setPower(0);
        ClassifyServo.setPower(0);

        if (latestSolution.isValid) {
            // --- 【核心修改】修正后的偏航计算和PD控制 ---
            // 1. 将计算器输出的标准坐标系角度(0=X轴)转换为我们的里程计坐标系角度(0=Y轴)
            double targetAzimuthInOdoFrame = latestSolution.aimAzimuth - 90;
            // 2. 因为发射器在车尾，机器人车头需要指向目标方向再偏移180度
            double targetHeading = targetAzimuthInOdoFrame - 180;

            double currentHeading = odo.getHeading(AngleUnit.DEGREES);
            double headingError = AngleUnit.normalizeDegrees(targetHeading - currentHeading);

            // 正确计算时间间隔 dt
            double dt = pidTimer.seconds();
            pidTimer.reset();

            // 避免在第一次循环时因为 dt=0 而导致除零错误
            double derivative = 0;
            if (dt > 0) {
                derivative = (headingError - lastHeadingError) / dt;
            }

            double pidOutput = (headingError * HEADING_P_GAIN) + (derivative * HEADING_D_GAIN);
            lastHeadingError = headingError;

            return Range.clip(pidOutput, -1.0, 1.0);
        } else {
            // 如果没有有效解，重置PID控制器并返回0转向力
            resetPIDController();
            return 0.0;
        }
    }

    // (runIntakeModeLogic, runManualModeLogic, initializeHardware 等方法保持不变)
    private double[] runIntakeModeLogic(double currentDrive, double currentStrafe) {
        InhaleMotor.setPower(0.6);
        IntakeMotor.setPower(1.0);
        double newDrive = currentDrive;
        double newStrafe = currentStrafe;
        LLResultTypes.DetectorResult target = deadeye.getTargetClosestToAnchor();
        if (target != null) {
            double[] errors = deadeye.calculateAlignmentError(target);
            if (errors != null) {
                newDrive = errors[1] * INTAKE_Y_GAIN;
                newStrafe = errors[0] * INTAKE_X_GAIN;
                double errorDist = Math.sqrt(errors[0] * errors[0] + errors[1] * errors[1]);
                if (errorDist < SERVO_ACTIVATION_DISTANCE_CM) {
                    if ("green".equals(target.getClassName())) {
                        lastClassifyServoPower = 1.0;
                    } else if ("purple".equals(target.getClassName())) {
                        lastClassifyServoPower = -1.0;
                    }
                }
            }
        }
        ClassifyServo.setPower(lastClassifyServoPower);
        return new double[]{newDrive, newStrafe};
    }

    private void runManualModeLogic() {
        InhaleMotor.setPower(0);
        IntakeMotor.setPower(0);
        lastClassifyServoPower = 0.0;
        ClassifyServo.setPower(0.0);
        setLauncherPitch(LaunchCalculator.MIN_ANGLE_DEG);
    }

    private void initializeHardware() {
        LeftFrontMotor = hardwareMap.get(DcMotor.class, "LeftFrontMotor");
        LeftBehindMotor = hardwareMap.get(DcMotor.class, "LeftBehindMotor");
        RightFrontMotor = hardwareMap.get(DcMotor.class, "RightFrontMotor");
        RightBehindMotor = hardwareMap.get(DcMotor.class, "RightBehindMotor");
        LeftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        LeftBehindMotor.setDirection(DcMotor.Direction.FORWARD);
        RightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        RightBehindMotor.setDirection(DcMotor.Direction.REVERSE);
        LeftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBehindMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBehindMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        InhaleMotor = hardwareMap.get(DcMotor.class, "InhaleMotor");
        IntakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        ClassifyServo = hardwareMap.get(CRServo.class, "ClassifyServo");
        InhaleMotor.setDirection(DcMotor.Direction.FORWARD);
        IntakeMotor.setDirection(DcMotor.Direction.FORWARD);

        deadeye = new Deadeye(hardwareMap);
        deadeye.start();

        RightPitch = hardwareMap.get(Servo.class, "RightPitch");
        LeftPitch = hardwareMap.get(Servo.class, "LeftPitch");

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(85.0, -180.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        localizer = new AprilTagLocalizer(hardwareMap);
        calculator = new LaunchCalculator();
    }

    private void handleToggles() {
        boolean rightBumper = gamepad1.right_bumper;
        boolean leftBumper = gamepad1.left_bumper;
        if (rightBumper && !rightBumperPreviouslyPressed) {
            isIntakeMode = !isIntakeMode;
            isLaunchMode = false;
        }
        if (leftBumper && !leftBumperPreviouslyPressed) {
            isLaunchMode = !isLaunchMode;
            isIntakeMode = false;
            if (isLaunchMode) {
                resetPIDController();
            }
        }
        rightBumperPreviouslyPressed = rightBumper;
        leftBumperPreviouslyPressed = leftBumper;
    }

    private void resetPIDController() {
        lastHeadingError = 0;
        pidTimer.reset();
    }

    private void setLauncherPitch(double pitchAngle) {
        double normalizedPosition = (pitchAngle - LaunchCalculator.MIN_ANGLE_DEG) / (LaunchCalculator.MAX_ANGLE_DEG - LaunchCalculator.MIN_ANGLE_DEG);
        normalizedPosition = Range.clip(normalizedPosition, 0.0, 1.0);
        RightPitch.setPosition(normalizedPosition);
        LeftPitch.setPosition(1.0 - normalizedPosition);
    }

    private void displayTelemetry() {
        telemetry.addData("--- 模式 ---", (isLaunchMode ? "发射模式 (自动瞄准)" : (isIntakeMode ? "吸入模式 (辅助)" : "手动驾驶 (无头模式)")));
        telemetry.addData("定位状态", isLocalized ? "成功" : "正在搜索AprilTag (速度减半)");

        if (isLaunchMode && latestSolution.isValid) {
            telemetry.addData("--- 自瞄系统 ---", "");
            double targetAzimuthInOdoFrame = latestSolution.aimAzimuth - 90;
            double targetHeading = targetAzimuthInOdoFrame - 180;
            telemetry.addData("目标航向", String.format(Locale.US, "%.1f°", AngleUnit.normalizeDegrees(targetHeading)));
            telemetry.addData("当前航向", String.format(Locale.US, "%.1f°", odo.getHeading(AngleUnit.DEGREES)));
            telemetry.addData("目标俯仰", String.format(Locale.US, "%.1f°", latestSolution.launcherPitch));
        } else if (isLaunchMode) {
            telemetry.addData("--- 自瞄系统 ---", "当前位置无解");
        }

        Pose2D currentPose = odo.getPosition();
        telemetry.addData("--- 里程计 ---", "");
        telemetry.addData("位置 (X, Y, H)", String.format(Locale.US, "%.1f, %.1f, %.1f°",
                currentPose.getX(DistanceUnit.MM), currentPose.getY(DistanceUnit.MM), currentPose.getHeading(AngleUnit.DEGREES)));

        telemetry.update();
    }
}