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
public class TestMotor extends CustomOpMode {
    private MotorEx motor1;
    private MotorEx motor2;
    private static final double VELOCITY = 4;

    public void init(){
        target = Target.TELEOP;
        super.init();
        motor1 = new MotorEx.MotorBuilder("Left_Shooter", MotorType.goBILDA,1,0,false,hardwareMap)
                .addMotor("Right_Shooter",false)
                .build();
        motor1.Init();
    }
    public void loop(){
        super.loop();
        if(gamepad.circle.Pressed()){
            motor1.setVelocity(MotorVelocityCalculator.OutputVelocityToRoundPerSec(VELOCITY,0.72));
        }
        if(gamepad.cross.Pressed()){
            motor1.setVelocity(0);
        }
        if(gamepad.square.Pressed()){
            motor1.setVelocity(MotorVelocityCalculator.OutputVelocityToRoundPerSec(-VELOCITY,0.72));
        }
        if(gamepad.triangle.Pressed()){
            motor1.setVelocity(4);
        }
        telemetry.addData("Velocity",motor1.getVelocity());
        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}