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
import org.firstinspires.ftc.teamcode.pedroPathing.library.CustomOpMode;

@TeleOp
public class TestIntake extends CustomOpMode {
    private double Value = 0;
    private static final double VELOCITY = 0;

    public void init(){
        target = Target.TELEOP;
        super.init();
    }
    public void loop(){
        super.loop();
        Value += gamepad.left_trigger.PressPosition()*0.1;
        Value -= gamepad.right_trigger.PressPosition()*0.1;

        Inhale.setVelocity(Value);
        telemetry.addData("Velocity",Inhale.getVelocity());
        telemetry.addData("Applied velocity",Value);
        telemetry.addData("Tick_Per_Round",Inhale.getTick_per_round());
        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}