package org.firstinspires.ftc.teamcode.vision.EchoLapse;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Locale;

@TeleOp(name="goBILDA Pinpoint 示例", group="Linear OpMode")

public class SensorGoBildaPinpointExample extends LinearOpMode {

    GoBildaPinpointDriver odo;

    double oldTime = 0;


    @Override
    public void runOpMode() {

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        /*
        设置里程计模块相对于里程计计算机追踪中心的相对位置。
        X 轴模块偏移量指的是 X 轴（前进方向）里程计模块与追踪中心的横向距离。
        中心点左侧为正数，右侧为负数。
        Y 轴模块偏移量指的是 Y 轴（侧移方向）里程计模块与追踪中心的前后距离。
        中心点前方为正数，后方为负数。
         */
        odo.setOffsets(85.0, -180.0, DistanceUnit.MM);

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);


        /*
        设置两个里程计模块的计数方向。
        当机器人向前移动时，X 轴（前进方向）模块的读数应该增加。
        当机器人向左侧移时，Y 轴（侧移方向）模块的读数应该增加。
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED );

        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X 偏移量", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y 偏移量", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("设备版本号:", odo.getDeviceVersion());
        telemetry.addData("航向角缩放因子", odo.getYawScalar());
        telemetry.update();

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            odo.update();

            if (gamepad1.a){
                odo.resetPosAndIMU();
            }

            if (gamepad1.b){
                odo.recalibrateIMU();
            }
            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("位置", data);

            /*
            获取机器人的当前速度（x 和 y 单位为毫米/秒，航向角速度单位为度/秒），并将其打印出来。
             */
            String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));

            telemetry.addData("速度", velocity);
            telemetry.addData("状态", odo.getDeviceStatus());
            telemetry.addData("Pinpoint 频率", odo.getFrequency());
            telemetry.addData("REV Hub 频率: ", frequency);
            telemetry.update();
        }
    }}