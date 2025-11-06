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
import org.firstinspires.ftc.teamcode.vision.QuickScope.LaunchCalculator;
import org.firstinspires.ftc.teamcode.vision.QuickScope.LaunchSolution;

public class AlgorithmLib {
    private static Pose2D currentTarget;

    /**
     * Blocked calculator
     * @return launch solution
     */
    public static LaunchSolution CalculateNewLaunch(){
        switch (alliance){
            case Red:
                currentTarget = RED_ALLIANCE_TARGET;
                break;
            case Blue:
                currentTarget = BLUE_ALLIANCE_TARGET;
                break;
        }
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
        return calculator.getSolution();
    }
    public static Runnable Aim(LaunchSolution solution){
        if(solution == null){
            return null;
        }
        double PitchPos = Calculator.DegreeToPitchServo(solution.launcherPitch);
        Shooter.setVelocity(calculator.calculateMotorRps(solution.launcherVelocity));
        if(PitchPos<0.2){
            LeftBoard.act(Action.Back);
            RightBoard.act(Action.Back);
            try {
                Thread.sleep(100);
            }catch (InterruptedException e){
                throw new RuntimeException(e);
            }
        }
        PitchServo.SetTemporaryPosition(PitchPos);
        return null;
    }
    public static Runnable ShootGreen(){
        return null;
    }
    public static Runnable ShootPurple(){
        return null;
    }
    public static Runnable ShootAll(){
        Inhale.act(Action.PullIn);
        LeftBoard.act(Action.Shoot);
        RightBoard.act(Action.Shoot);
        try {
            Thread.sleep(500);
        }catch (InterruptedException e){
            throw new RuntimeException(e);
        }
        return null;
    }
}
