package org.firstinspires.ftc.teamcode.pedroPathing.library;

import static com.bear27570.yuan.BotFactory.Model.Action.Armed;
import static com.bear27570.yuan.BotFactory.Model.Action.Lock;
import static com.bear27570.yuan.BotFactory.Model.Action.PullIn;
import static com.bear27570.yuan.BotFactory.Model.Action.Shoot;
import static com.bear27570.yuan.BotFactory.Model.Action.Stop;
import static com.bear27570.yuan.BotFactory.Model.Action.Up;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.ConstantLib.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.ObjectLib.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.StatesLib.*;

import androidx.annotation.NonNull;

import com.bear27570.yuan.BotFactory.Model.Action;
import com.bear27570.yuan.BotFactory.Model.ConflictPolicy;
import com.bear27570.yuan.BotFactory.Model.Priority;
import com.bear27570.yuan.BotFactory.ThreadManagement.Task;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.pedroPathing.Services.Calculator;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Models.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Models.Target;
import org.firstinspires.ftc.teamcode.vision.QuickScope.CalculationParams;
import org.firstinspires.ftc.teamcode.vision.QuickScope.LaunchSolution;

public class AlgorithmLib {
    private static Pose2D currentTarget;

    public static Runnable ShootGreen() {
        Inhale.VelocityAct(PullIn);
        RightBoard.act(Action.Shoot);
        try {
            Thread.sleep(SHOOT_ONE_BALL_TIME);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        RightBoard.act(Action.Lock);
        Inhale.VelocityAct(Stop);
        return null;
    }

    public static Runnable PopGreen() {
        RightBoard.act(Action.Shoot);
        try {
            Thread.sleep(SHOOT_ONE_BALL_TIME);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        RightBoard.act(Action.Lock);
        return null;
    }

    public static Runnable ShootPurple() {
        Inhale.VelocityAct(PullIn);
        LeftBoard.act(Action.Shoot);
        try {
            Thread.sleep(SHOOT_ONE_BALL_TIME);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        LeftBoard.act(Action.Lock);
        return null;
    }

    public static Runnable PopPurple() {
        LeftBoard.act(Action.Shoot);
        try {
            Thread.sleep(SHOOT_ONE_BALL_TIME);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        LeftBoard.act(Action.Lock);
        return null;
    }

    public static Runnable Aim() {
        PitchServo.SetTemporaryPosition(Calculator.DegreeToPitchServo(latestSolution.get().launcherAngle));
        Shooter.setVelocity(latestSolution.get().motorRpm / 60);
        return null;
    }

    public static Runnable ShootAll() {
        Inhale.VelocityAct(Action.PullIn);
        LeftBoard.act(Action.Shoot);
        RightBoard.act(Action.Shoot);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        LeftBoard.act(Action.Lock);
        RightBoard.act(Action.Lock);
        Inhale.VelocityAct(PullIn);

        return null;
    }

    public static Runnable updatePosition(){
        Pose2D pose = null;
        while (pose == null) {
            pose = aprilTagLocalizer.getRobotPose();
            gamepad.rumble(0.5, 0.5, 60);
            gamepad.update();
            try {
                Thread.sleep(100);
            }catch (InterruptedException e){
                throw new RuntimeException();
            }
        }
        gamepad.stopRumble();
        aprilTagLocalizer.close();
        pinpointPoseProvider.setPose(pose);
        return null;
    }

    public static Runnable IntakeStruct() {
        PitchServo.act(Up);
        LeftBoard.act(Lock);
        RightBoard.act(Lock);
        Shooter.VelocityAct(Armed);
        IntakeMotor.VelocityAct(PullIn);
        Inhale.VelocityAct(PullIn);
        return null;
    }

    public static Runnable checkShooterVelocity() {
        while (Shooter.getVelocity() < latestSolution.get().motorRpm / 60) {
            gamepad.rumble(1, 1, 60);
            try {
                Thread.sleep(50);
            }catch (InterruptedException e){
                throw new RuntimeException();
            }
        }
        gamepad.stopRumble();
        return null;
    }

    public static Runnable getNewLaunch(Telemetry telemetry) {
        pinpointPoseProvider.update();
        double robotX_cm = pinpointPoseProvider.getX(DistanceUnit.CM);
        double robotY_cm = pinpointPoseProvider.getY(DistanceUnit.CM);

        double normalizationX = robotX_cm / 365.76;
        double normalizationY = robotY_cm / 365.76;
        telemetry.addData("NormalizationX", normalizationX);
        telemetry.addData("NormalizationY", normalizationY);

        double cartesianVelX_m_s = pinpointPoseProvider.getXVelocity(DistanceUnit.METER);
        double cartesianVelY_m_s = pinpointPoseProvider.getYVelocity(DistanceUnit.METER);
        double speed_m_s = Math.hypot(cartesianVelX_m_s, cartesianVelY_m_s);
        double direction_rad = Math.atan2(cartesianVelY_m_s, cartesianVelX_m_s);
        double direction_deg = Math.toDegrees(direction_rad);
        if (direction_deg < 0) direction_deg += 360;
        CalculationParams currentParams = new CalculationParams(
                normalizationX,
                normalizationY,
                speed_m_s,
                direction_deg,
                alliance.name()
        );
        latestSolution.set(archerLogic.calculateSolution(currentParams));
        return null;
    }
}
