package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.bear27570.yuan.BotFactory.Model.MotorType;
import com.bear27570.yuan.BotFactory.Motor.MotorEx;
import com.bear27570.yuan.BotFactory.Services.MotorVelocityCalculator;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import static com.bear27570.yuan.BotFactory.Model.Action.*;

import static org.firstinspires.ftc.teamcode.pedroPathing.library.ObjectLib.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.StatesLib.*;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Models.Target;
import org.firstinspires.ftc.teamcode.pedroPathing.Services.Calculator;
import org.firstinspires.ftc.teamcode.pedroPathing.library.CustomOpMode;

@TeleOp
public class TestShooter extends CustomOpMode {
    private double Value = 0;
    private static final double VELOCITY = 0;

    public void init(){
        target = Target.TELEOP;
        super.init();
    }
    public void start(){
        super.start();
        //Inhale.VelocityAct(PullIn);
    }
    public void loop(){
        //super.loop();
        gamepad.update();
        Value += gamepad.left_trigger.PressPosition()*0.1;
        Value -= gamepad.right_trigger.PressPosition()*0.1;
        if(gamepad.circle.Pressed()){
            LeftBoard.act(Shoot);
            RightBoard.act(Shoot);
        }
        Shooter.setVelocity(Value);
        telemetryM.addData("Velocity 1",Shooter.getOneVelocity(0));
        telemetryM.addData("Velocity 2",Shooter.getOneVelocity(1));
        telemetryM.addData("Average Velocity",Shooter.getVelocity());
        telemetryM.addData("Applied velocity",Value*28);
        telemetryM.addData("Tick_Per_Round",Shooter.getTick_per_round());
        telemetryM.addData("Status", "Running");
        telemetryM.addData("follower X",follower.getPose().getX());
        telemetryM.addData("follower Y",follower.getPose().getY());
        telemetryM.addLine("InputMpS:${Value}");
        telemetryM.update(telemetry);
        follower.update();
    }
}