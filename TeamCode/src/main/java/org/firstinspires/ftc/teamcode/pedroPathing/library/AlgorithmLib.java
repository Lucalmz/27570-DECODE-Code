package org.firstinspires.ftc.teamcode.pedroPathing.library;

import static com.bear27570.yuan.BotFactory.Model.Action.Armed;
import static com.bear27570.yuan.BotFactory.Model.Action.Lock;
import static com.bear27570.yuan.BotFactory.Model.Action.Out;
import static com.bear27570.yuan.BotFactory.Model.Action.PullIn;
import static com.bear27570.yuan.BotFactory.Model.Action.PullOut;
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
import org.firstinspires.ftc.teamcode.vision.EchoLapse.PinpointPoseProvider;
import org.firstinspires.ftc.teamcode.vision.QuickScope.CalculationParams;
import org.firstinspires.ftc.teamcode.vision.QuickScope.LaunchSolution;

public class AlgorithmLib {
    private static Pose2D currentTarget;

    public static void ShootGreen() {
        try {
            Inhale.VelocityAct(Out);
            Thread.sleep(MOVE_BACK_TIME);
            Inhale.VelocityAct(PullIn);
            LeftBoard.act(Action.Shoot);

            Thread.sleep(SHOOT_ONE_BALL_TIME);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        LeftBoard.act(Action.Lock);
        Inhale.VelocityAct(Stop);
    }

    public static void PopGreen() {
        LeftBoard.act(Action.Shoot);
        try {
            Thread.sleep(SHOOT_ONE_BALL_TIME);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        LeftBoard.act(Action.Lock);
    }

    public static void ShootPurple() {
        try {
            Inhale.VelocityAct(Out);
            Thread.sleep(MOVE_BACK_TIME);
            RightBoard.act(Action.Shoot);
            Inhale.VelocityAct(PullIn);
            Thread.sleep(SHOOT_ONE_BALL_TIME);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        RightBoard.act(Action.Lock);
        Inhale.VelocityAct(Stop);
    }

    public static void PopPurple() {
        RightBoard.act(Shoot);
        try {
            Thread.sleep(SHOOT_ONE_BALL_TIME);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        RightBoard.act(Lock);
    }

    public static void ShootAll() {
        try {
            Inhale.VelocityAct(Out);
            Thread.sleep(MOVE_BACK_TIME);
            LeftBoard.act(Shoot);
            RightBoard.act(Shoot);
            Inhale.VelocityAct(PullIn);
            Thread.sleep((long)(SHOOT_ONE_BALL_TIME * 2.5));
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        LeftBoard.act(Lock);
        RightBoard.act(Lock);
        Inhale.VelocityAct(Stop);
    }

    public static void PopAll() {
        LeftBoard.act(Shoot);
        RightBoard.act(Shoot);
        try {
            Thread.sleep((long)(SHOOT_ONE_BALL_TIME * 2.5));
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        LeftBoard.act(Lock);
        RightBoard.act(Lock);
    }

    public static Runnable Aim() {
        PitchServo.SetTemporaryPosition(Calculator.DegreeToPitchServo(latestSolution.get().launcherAngle));
        Shooter.setVelocity(latestSolution.get().motorRpm / 60);
        return null;
    }

    public static Runnable IntakeStruct() {
        PitchServo.act(Up);
        LeftBoard.act(Lock);
        RightBoard.act(Lock);
        IntakeMotor.VelocityAct(PullIn);
        Inhale.VelocityAct(PullIn);
        Shooter.VelocityActInSlowSet(Armed,500);
        return null;
    }

    public static void checkShooterVelocity() {
        while (Shooter.getVelocity() < latestSolution.get().motorRpm / 60) {
            gamepad.rumble(1, 1, 60);
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                throw new RuntimeException();
            }
        }
        gamepad.stopRumble();
    }

    public static void setNewPosition() {
        currentPose = null;
        while (currentPose == null) {
            currentPose = aprilTagLocalizer.getRobotPose();
            gamepad.rumble(0.8, 0.8, 50);
        }
        gamepad.stopRumble();
        pinpointPoseProvider.setPose(currentPose);
    }

    public static void getNewLaunch() {
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
        CalculationParams currentParams = new CalculationParams(
                normalizationX, normalizationY, speed_m_s, direction_deg, alliance.name());
        latestSolution.set(archerLogic.calculateSolution(currentParams));
    }
}
