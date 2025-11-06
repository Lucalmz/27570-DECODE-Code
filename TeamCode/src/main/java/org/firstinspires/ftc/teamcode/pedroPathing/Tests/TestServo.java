package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import static org.firstinspires.ftc.teamcode.pedroPathing.library.ObjectLib.gamepad;

import com.bear27570.yuan.BotFactory.Interface.ServoEx;
import static com.bear27570.yuan.BotFactory.Model.Action.*;
import com.bear27570.yuan.BotFactory.Servo.ServoBuilders;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.library.CustomOpMode;
@TeleOp
public class TestServo extends CustomOpMode {
    private ServoEx servo;
    private double Value = 0;

    public void init(){
        super.init();
        servo = new ServoBuilders.PWMServoBuilder("LeftBoard",0.5,false,hardwareMap)
                .build();
        servo.Init();
    }
    public void loop(){
        super.loop();
        Value += gamepad.left_trigger.PressPosition()*0.1;
        Value -= gamepad.right_trigger.PressPosition()*0.1;
        servo.SetTemporaryPosition(Value);
        telemetry.addData("Velocity",servo.getVelocity());
        telemetry.addData("Status", "Running");
        telemetry.update();
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
