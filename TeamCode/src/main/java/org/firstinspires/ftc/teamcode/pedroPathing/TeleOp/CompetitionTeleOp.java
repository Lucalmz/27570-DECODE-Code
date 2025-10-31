package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.library.ObjectLib.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.StatesLib.*;

import com.pedropathing.telemetry.SelectableOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Models.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Models.Target;
import org.firstinspires.ftc.teamcode.pedroPathing.library.CustomOpMode;

public class CompetitionTeleOp extends CustomOpMode{
    @Override
    public void init(){
        target = Target.TELEOP;
        super.init();
    }

    @Override
    public void init_loop() {
        super.init_loop();
        gamepad.update();
        telemetryM.addData("Select running alliance! Current alliance:",alliance.toString());
        telemetryM.update();
        if(gamepad.left_bumper.Pressed()){
            alliance = Alliance.Red;
        }
        if(gamepad.right_bumper.Pressed()){
            alliance = Alliance.Blue;
        }
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();
    }

    @Override
    public void stop() {
        super.stop();
    }
}