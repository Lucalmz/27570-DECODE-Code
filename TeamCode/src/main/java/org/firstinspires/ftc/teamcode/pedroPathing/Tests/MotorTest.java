package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import static com.bear27570.yuan.BotFactory.Model.ConflictPolicy.IGNORE;
import static com.bear27570.yuan.BotFactory.Model.Priority.LOW;

import com.bear27570.yuan.BotFactory.Gamepad.GamepadEx;
import com.bear27570.yuan.BotFactory.Motor.MotorEx;
import com.bear27570.yuan.BotFactory.Servo.ServoEx;
import com.bear27570.yuan.BotFactory.ThreadManagement.Task;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.library.ObjectLib.*;
import static com.bear27570.yuan.BotFactory.Model.Action.*;

import org.firstinspires.ftc.teamcode.pedroPathing.library.CustomOpMode;

@TeleOp(name = "ClassifyTest")
public class MotorTest extends CustomOpMode {
    boolean state;
    boolean state2;
    @Override
    public void init(){
        TestMotor1 = new MotorEx.MotorBuilder("TestMotor1",0,false,hardwareMap)
                .addMotor("TestMotor2",false)
                .build();
        TestMotor2 = new MotorEx.MotorBuilder("TestMotor3",0,false,hardwareMap)
                .build();
        TestMotor3 = new MotorEx.MotorBuilder("TestMotor4",0,false,hardwareMap)
                .build();
        TestServo = new ServoEx.ServoBuilder("TestServo",Back,0,false,hardwareMap)
                .addAction(Turned,0.33)
                .setSwitcher(Turned,Back)
                .build();
    }
    public void loop(){
        if(gamepad.cross.Pressed()){
            Manager.submit(new Task.TaskBuilder(LOW,IGNORE).runs(()->{
                if(state){
                    TestMotor1.setPower(1);
                    state=!state;
                    return;
                }
                TestMotor1.setPower(0);
                state=!state;
            }).build());
        }
        if (gamepad.circle.Pressed()){
            Manager.submit(new Task.TaskBuilder(LOW,IGNORE).runs(()->{
                if(state2){
                    TestMotor2.setPower(1);
                    state2=!state2;
                    return;
                }
                TestMotor2.setPower(-1);
                state2=!state2;
            }).build());
        }
        if (gamepad.square.Pressed()){
            Manager.submit(new Task.TaskBuilder(LOW,IGNORE).runs(()->{
                TestServo.Switch();
            }).build());
        }
        if(gamepad.dpad_up.Pressed()){
            Manager.submit(new Task.TaskBuilder(LOW,IGNORE).runs(()->{
                TestMotor3.setPower(1);
            }).build());
        }
        if(gamepad.dpad_down.Pressed()){
            Manager.submit(new Task.TaskBuilder(LOW,IGNORE).runs(()->{
                TestMotor3.setPower(0);
            }).build());
        }
    }
}
