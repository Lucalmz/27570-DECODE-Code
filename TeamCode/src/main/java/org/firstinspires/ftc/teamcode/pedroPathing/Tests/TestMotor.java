package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import static org.firstinspires.ftc.teamcode.pedroPathing.library.ObjectLib.gamepad;

import com.bear27570.yuan.BotFactory.Gamepad.GamepadEx;
import com.bear27570.yuan.BotFactory.Interface.DcMotorIO;
import com.bear27570.yuan.BotFactory.Interface.ServoEx;
import com.bear27570.yuan.BotFactory.Model.MotorType;
import com.bear27570.yuan.BotFactory.Motor.MotorEx;
import com.bear27570.yuan.BotFactory.Servo.ServoBuilders;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.library.CustomOpMode;
@TeleOp
public class TestMotor extends OpMode {
    private DcMotorEx LFM,LBM,RFM,RBM;

    public void init(){
        LFM = hardwareMap.get(DcMotorEx.class,"LeftFrontMotor");
        LBM = hardwareMap.get(DcMotorEx.class,"LeftBehindMotor");
        RFM = hardwareMap.get(DcMotorEx.class,"RightFrontMotor");
        RBM = hardwareMap.get(DcMotorEx.class,"RightBehindMotor");
        gamepad = GamepadEx.GetGamepadEx(gamepad1);
    }
    public void loop(){
        gamepad.update();
        if(gamepad.cross.Pressed()){
            LFM.setPower(0.7);
        }
        if(gamepad.circle.Pressed()){
            LBM.setPower(0.7);
        }

        if(gamepad.triangle.Pressed()){
            RFM.setPower(0.7);
        }
        if(gamepad.square.Pressed()){
            RBM.setPower(0.7);
        }
        if(gamepad.cross.justReleased()){
            LFM.setPower(0);
        }
        if(gamepad.circle.justReleased()){
            LBM.setPower(0);
        }

        if(gamepad.triangle.justReleased()){
            RFM.setPower(0);
        }
        if(gamepad.square.justReleased()){
            RBM.setPower(0);
        }
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
