package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import static com.bear27570.yuan.BotFactory.Model.ConflictPolicy.IGNORE;
import static com.bear27570.yuan.BotFactory.Model.Priority.LOW;

import com.bear27570.yuan.BotFactory.Gamepad.GamepadEx;
import com.bear27570.yuan.BotFactory.Model.Action;
import com.bear27570.yuan.BotFactory.Motor.MotorEx;
import com.bear27570.yuan.BotFactory.Servo.ServoEx;
import com.bear27570.yuan.BotFactory.ThreadManagement.Task;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.library.HardwareLib.*;
import static com.bear27570.yuan.BotFactory.Model.Action.*;

@TeleOp(name = "ClassifyTest")
public class MotorTest extends OpMode {
    @Override
    public void init(){
        TestMotor1 = new MotorEx.MotorBuilder("TestMotor1",0,false,hardwareMap)
                .addMotor("TestMotor2",false)
                .build();
        TestServo = new ServoEx.ServoBuilder("TestServo",Back,0,false,hardwareMap)
                .addAction(Turned,0.33)
                .setSwitcher(Turned,Back)
                .build();
        gamepad = GamepadEx.GetGamepadEx(gamepad1);
    }
    public void loop(){
        if(gamepad.cross.Pressed()){
            Manager.submit(new Task.TaskBuilder(LOW,IGNORE).runs(()->{
                TestMotor1.setPower(1);
            }).build());
        }
        if (gamepad.circle.Pressed()){
            Manager.submit(new Task.TaskBuilder(LOW,IGNORE).runs(()->{
                TestMotor1.setPower(0);
            }).build());
        }
        if (gamepad.square.Pressed()){
            Manager.submit(new Task.TaskBuilder(LOW,IGNORE).runs(()->{
                TestServo.Switch();
            }).build());
        }
        gamepad.update();
    }
}
