package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.bear27570.yuan.BotFactory.Gamepad.GamepadEx;
import com.bear27570.yuan.BotFactory.Model.MotorType;
import com.bear27570.yuan.BotFactory.Motor.MotorEx;
import com.bear27570.yuan.BotFactory.Services.MotorVelocityCalculator;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
@Configurable
public class TestShooter extends OpMode {
    private double Value = 0;

    public static double Kp = 190,Ki = 0.25,Kd = 65,kf = 18.3;
    private DcMotorEx Shooter;

    public void init(){
        Shooter = hardwareMap.get(DcMotorEx.class,"LeftShooter");
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gamepad = GamepadEx.GetGamepadEx(gamepad1);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(Kp,Ki,Kd,kf));
    }
    public void loop(){
        gamepad.update();
        Value += gamepad.left_trigger.PressPosition()*0.2;
        Value -= gamepad.right_trigger.PressPosition()*0.2;
        if(gamepad.circle.Pressed()){
            LeftBoard.act(Shoot);
            RightBoard.act(Shoot);
        }
        Shooter.setVelocity(Value*28);
        telemetryM.addData("Velocity 1",Shooter.getVelocity());
        telemetryM.addData("Applied velocity",Value*28);
        telemetryM.addData("Status", "Running");
        telemetryM.addLine("InputMpS:${Value}");
        telemetryM.update(telemetry);
    }
}