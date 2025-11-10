package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@TeleOp(name = "HybridTeleOpTest", group = "Pedro Pathing")
public class HybridTeleOpTest extends OpMode {
    Follower follower;
    public void init(){
        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive();
    }
    public void loop(){
        follower.updateTunerStyleHybridDrive(gamepad1.left_stick_x,gamepad1.left_stick_y,0);
    }
}
