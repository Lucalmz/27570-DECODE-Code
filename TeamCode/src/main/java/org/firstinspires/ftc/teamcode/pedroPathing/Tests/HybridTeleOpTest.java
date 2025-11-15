package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import static org.firstinspires.ftc.teamcode.pedroPathing.library.ObjectLib.gamepad;

import com.bear27570.yuan.BotFactory.Gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@TeleOp(name = "HybridTeleOpTest", group = "Pedro Pathing")
public class HybridTeleOpTest extends OpMode {
    Follower follower;
    double Value = 0;
    public void init(){
        follower = Constants.createAdvancedFollower(hardwareMap);
        follower.startTunerStyleHybridDrive(Math.toRadians(-90));
        gamepad = GamepadEx.GetGamepadEx(gamepad1);
    }
    public void loop(){
        gamepad.update();
        Value += gamepad.left_trigger.PressPosition()*0.1;
        Value -= gamepad.right_trigger.PressPosition()*0.1;
        follower.updateTunerStyleHybridDrive(-gamepad1.left_stick_x,-gamepad1.left_stick_y,Value);
    }
}
