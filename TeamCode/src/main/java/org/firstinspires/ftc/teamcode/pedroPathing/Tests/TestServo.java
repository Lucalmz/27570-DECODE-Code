package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import static org.firstinspires.ftc.teamcode.pedroPathing.library.ObjectLib.gamepad;

import com.bear27570.yuan.BotFactory.Interface.ServoEx;
import com.bear27570.yuan.BotFactory.Motor.MotorEx;
import static com.bear27570.yuan.BotFactory.Model.Action.*;
import com.bear27570.yuan.BotFactory.Servo.ServoBuilders;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.library.CustomOpMode;
@TeleOp
public class TestServo extends CustomOpMode {
    private ServoEx servo;
    private static final double VELOCITY = 50;

    public void init(){
        super.init();
        servo = new ServoBuilders.NormalCRServoBuilder("TestServo",Backward,0,false,180,hardwareMap)
                .addAction(Forward,1)
                .build();
        servo.Init();
    }
    public void loop(){
        super.loop();
        if(gamepad.circle.Pressed()){
            servo.act(Forward);
        }
        if(gamepad.cross.Pressed()){
            servo.StopVelTurning();
        }
        if(gamepad.square.Pressed()){
            servo.act(Backward);
        }
        telemetry.addData("Velocity",servo.getVelocity());
        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}
