package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.library.ObjectLib.aprilTagLocalizer;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.ObjectLib.archerLogic;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.ObjectLib.gamepad;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.StatesLib.latestSolution;

import com.bear27570.yuan.BotFactory.Gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.vision.EchoLapse.FollowerPoseProvider;
import org.firstinspires.ftc.teamcode.vision.EchoLapse.IPoseProvider;
import org.firstinspires.ftc.teamcode.vision.QuickScope.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.vision.QuickScope.ArcherLogic;
import org.firstinspires.ftc.teamcode.vision.QuickScope.CalculationParams;

@TeleOp
public class TestTeleOp extends OpMode {
    IPoseProvider provider;
    Pose2D pose;
    boolean isFirstPoseAvailable;
    Follower follower;

    public void init() {
        follower = Constants.createAdvancedFollower(hardwareMap);
        provider = new FollowerPoseProvider(follower);
        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap);
        archerLogic = new ArcherLogic();
        gamepad = GamepadEx.GetGamepadEx(gamepad1);
    }

    public void start() {
        provider.initialize();
        pose = aprilTagLocalizer.getRobotPose();
        follower.breakFollowing();
    }

    public void loop() {
        provider.update();
        follower.update();
        gamepad.update();
        if (pose != null && !isFirstPoseAvailable) {
            follower.startTunerStyleHybridDrive(Math.toRadians(-90));
            provider.setPose(pose);
            isFirstPoseAvailable = true;
        }
        if (isFirstPoseAvailable&&!gamepad.left_bumper.Pressed()) {
            telemetry.addData("Provider Pose", "(%.0f,%.0f)", provider.getX(DistanceUnit.CM), provider.getY(DistanceUnit.CM));
            telemetry.addData("Follower Pose", "(%.0f,%.0f)", follower.getPose().getX(), follower.getPose().getY());
            double robotX_cm = -provider.getX(DistanceUnit.CM);
            double robotY_cm = provider.getY(DistanceUnit.CM);

            double normalizationX = robotX_cm / 365.76;
            double normalizationY = robotY_cm / 365.76;
            telemetry.addData("NormalizationX", normalizationX);
            telemetry.addData("NormalizationY", normalizationY);

            double cartesianVelX_m_s = -provider.getXVelocity(DistanceUnit.METER);
            double cartesianVelY_m_s = provider.getYVelocity(DistanceUnit.METER);
            double speed_m_s = Math.hypot(cartesianVelX_m_s, cartesianVelY_m_s);
            double direction_rad = Math.atan2(cartesianVelY_m_s, cartesianVelX_m_s);
            double direction_deg = Math.toDegrees(direction_rad);

            telemetry.addData("Speed", speed_m_s);
            telemetry.addData("speeddegree", direction_deg);
            if (direction_deg < 0) direction_deg += 360;
            CalculationParams currentParams = new CalculationParams(
                    normalizationX,
                    normalizationY,
                    speed_m_s,
                    direction_deg,
                    "Red"
            );
            if (gamepad.right_bumper.Pressed()) {
                latestSolution.set(archerLogic.calculateSolution(currentParams));
                telemetry.addLine(">> 方案已解算 <<");
                telemetry.addData("发射电机转速 (RPM)", "%.0f", latestSolution.get().motorRpm);
                telemetry.addData("偏航角 (Yaw)", "%.2f deg", latestSolution.get().aimAzimuthDeg);
                telemetry.addData("俯仰角 (Pitch)", "%.2f deg", latestSolution.get().launcherAngle);
            }
            if(latestSolution.get()!=null) {
                follower.updateTunerStyleHybridDrive(-gamepad.left_stick_y.PressPosition(), -gamepad.left_stick_x.PressPosition(),Math.toRadians(latestSolution.get().aimAzimuthDeg));
            }

        }
        pose = aprilTagLocalizer.getRobotPose();
    }
}