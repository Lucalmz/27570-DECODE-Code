package org.firstinspires.ftc.teamcode.pedroPathing.library;

import com.bear27570.yuan.AdvantageCoreLib.Logging.Logger;
import com.bear27570.yuan.BotFactory.Gamepad.GamepadEx;
import com.bear27570.yuan.BotFactory.Interface.PeriodicRunnable;
import com.bear27570.yuan.BotFactory.ThreadManagement.TaskManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.ObjectLib.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.StatesLib.target;

import org.firstinspires.ftc.teamcode.pedroPathing.Models.Target;

import java.util.ArrayList;
import java.util.List;

public class CustomOpMode extends OpMode {
    private final List<PeriodicRunnable> periodicList = new ArrayList<>();
    public void init(){
        Logger.initialize(true,System::nanoTime);
        gamepad = GamepadEx.GetGamepadEx(gamepad1);
        Manager = TaskManager.getInstance();
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
    }
    public void stop(){
        logger.close();
    }
}