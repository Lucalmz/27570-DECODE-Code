package org.firstinspires.ftc.teamcode.pedroPathing.library;

import static org.firstinspires.ftc.teamcode.pedroPathing.library.ConstantLib.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.ObjectLib.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.StatesLib.*;

import androidx.annotation.NonNull;

import com.bear27570.yuan.BotFactory.Model.Action;

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
    public static Runnable ShootGreen(){
        Inhale.moveDistance(SHOOT_ONE_BALL_DIST,0.8);
        RightBoard.act(Action.Shoot);
        try {
            Thread.sleep(SHOOT_ONE_BALL_TIME);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        RightBoard.act(Action.Lock);
        return null;
    }
    public static Runnable PopGreen(){
        RightBoard.act(Action.Shoot);
        try {
            Thread.sleep(SHOOT_ONE_BALL_TIME);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        RightBoard.act(Action.Lock);
        return null;
    }
    public static Runnable ShootPurple(){
        Inhale.moveDistance(SHOOT_ONE_BALL_DIST,0.8);
        LeftBoard.act(Action.Shoot);
        try {
            Thread.sleep(SHOOT_ONE_BALL_TIME);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        LeftBoard.act(Action.Lock);
        return null;
    }
    public static Runnable PopPurple(){
        LeftBoard.act(Action.Shoot);
        try {
            Thread.sleep(SHOOT_ONE_BALL_TIME);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        LeftBoard.act(Action.Lock);
        return null;
    }
    public static Runnable ShootAll(){
        Inhale.VelocityAct(Action.PullIn);
        LeftBoard.act(Action.Shoot);
        RightBoard.act(Action.Shoot);
        try {
            Thread.sleep(500);
        }catch (InterruptedException e){
            throw new RuntimeException(e);
        }
        LeftBoard.act(Action.Lock);
        RightBoard.act(Action.Lock);
        return null;
    }
    public static void getNewLaunch() throws Exception {
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
                normalizationX,
                normalizationY,
                speed_m_s,
                direction_deg,
                alliance.name()
        );
        latestSolution.set(archerLogic.calculateSolution(currentParams));
        if(latestSolution==null){
            throw new Exception("Solution is null");
        }
    }
}
