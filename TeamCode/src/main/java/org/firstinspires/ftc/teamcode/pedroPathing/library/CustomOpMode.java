package org.firstinspires.ftc.teamcode.pedroPathing.library;

import com.bear27570.yuan.AdvantageCoreLib.Logging.Logger;
import com.bear27570.yuan.BotFactory.Gamepad.GamepadEx;
import com.bear27570.yuan.BotFactory.Interface.PeriodicRunnable;
import com.bear27570.yuan.BotFactory.ThreadManagement.TaskManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.ObjectLib.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.StatesLib.*;

import org.firstinspires.ftc.teamcode.pedroPathing.Models.Target;
import org.firstinspires.ftc.teamcode.pedroPathing.Services.IOStream;

import java.util.ArrayList;
import java.util.List;

public class CustomOpMode extends OpMode {
    private final List<PeriodicRunnable> periodicList = new ArrayList<>();
    public void init(){
        Logger.initialize(false,System::nanoTime);
        gamepad = GamepadEx.GetGamepadEx(gamepad1);
        Manager = TaskManager.getInstance();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        ioStream = new IOStream(hardwareMap.appContext);
        logger = Logger.getINSTANCE();
        switch (target){
            case TELEOP:

                break;
            case AUTONOMOUS:

                break;
        }
    }
    public void init_loop(){

    }
    public void start(){

    }
    public void loop(){
        gamepad.update();
        telemetryM.update();
        /*
        switch (target){
            case TELEOP:
                double coefficient = gamepad.left_trigger.PressPosition();
                if(coefficient>0.95){
                    follower.holdPoint(follower.getPose());
                    return;
                }
                if(coefficient<0.05){
                    follower.startTeleopDrive(false);
                    follower.setTeleOpDrive(Math.pow(gamepad.left_stick_y.PressPosition(),3),Math.pow(gamepad.left_stick_x.PressPosition(),3),Math.pow(gamepad.right_stick_x.PressPosition(),3),true);
                    return;
                }
                follower.startTeleopDrive(false);
                follower.setTeleOpDrive(Math.pow(gamepad.left_stick_y.PressPosition(),3)*(1-coefficient),Math.pow(gamepad.left_stick_x.PressPosition(),3)*(1-coefficient),Math.pow(gamepad.right_stick_x.PressPosition(),3)*(1-coefficient),true);
                break;
            case AUTONOMOUS:
                break;
        }*/
    }
    public void stop(){
        logger.close();
    }
}