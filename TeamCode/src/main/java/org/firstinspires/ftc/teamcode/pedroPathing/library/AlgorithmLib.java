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
import org.firstinspires.ftc.teamcode.vision.QuickScope.LaunchSolution;

public class AlgorithmLib {
    private static Pose2D currentTarget;

    /**
     * Blocked calculator
     * @return launch solution
     */
    public static LaunchSolution CalculateNewLaunch(){

        return null;
    }
    public static Runnable Aim(LaunchSolution solution){
        if(solution == null){
            return null;
        }

        return null;
    }
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
}
