package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import static org.firstinspires.ftc.teamcode.pedroPathing.library.ObjectLib.gamepad;

import com.bear27570.yuan.BotFactory.Interface.ServoEx;
import com.bear27570.yuan.BotFactory.Model.MotorType;
import com.bear27570.yuan.BotFactory.Motor.MotorEx;
import com.bear27570.yuan.BotFactory.Servo.ServoBuilders;

import org.firstinspires.ftc.teamcode.pedroPathing.library.CustomOpMode;

public class TestMotor extends CustomOpMode {
    private MotorEx motor;
    private double Value = 0;

    public void init(){
        super.init();
         motor = new MotorEx.MotorBuilder("IntakeMotor", MotorType.goBILDA,5.2,0,false,hardwareMap)
                 .build();
         motor.Init();
    }
    public void loop(){
        super.loop();
        Value += gamepad.left_trigger.PressPosition()*0.1;
        Value -= gamepad.right_trigger.PressPosition()*0.1;
        motor.setVelocity(Value);
        telemetry.addData("Velocity",motor.getVelocity());
        telemetry.addData("Status", "Running");
        telemetry.update();
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
