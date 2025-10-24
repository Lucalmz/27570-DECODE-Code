package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.bear27570.yuan.BotFactory.Motor.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import static com.bear27570.yuan.BotFactory.Model.Action.*;

import static org.firstinspires.ftc.teamcode.pedroPathing.library.ObjectLib.*;

import androidx.annotation.MainThread;

import org.firstinspires.ftc.teamcode.pedroPathing.library.CustomOpMode;

@TeleOp

public class TestMotor extends CustomOpMode {
    private MotorEx motor1;
    private MotorEx motor2;

    public void init(){
        super.init();
        motor1 = new MotorEx.MotorBuilder("Left_Shooter",0,false,hardwareMap)
                .addMotor("Right_Shooter",false)
                .build();
    }
    public void loop(){
        super.loop();
        motor1.setVelocity(5800*gamepad.left_stick_y.PressPosition());
        telemetry.addData("Velocity",gamepad.left_stick_y.PressPosition());
        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}