package org.firstinspires.ftc.teamcode.pedroPathing.library;

import static com.bear27570.yuan.BotFactory.Model.Action.*;

import com.bear27570.yuan.AdvantageCoreLib.Logging.Logger;
import com.bear27570.yuan.BotFactory.Gamepad.GamepadEx;
import com.bear27570.yuan.BotFactory.Interface.PeriodicRunnable;
import com.bear27570.yuan.BotFactory.Model.Action;
import com.bear27570.yuan.BotFactory.Model.MotorType;
import com.bear27570.yuan.BotFactory.Motor.MotorEx;
import com.bear27570.yuan.BotFactory.Servo.ServoBuilders;
import com.bear27570.yuan.BotFactory.ThreadManagement.TaskManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import static org.firstinspires.ftc.teamcode.pedroPathing.library.ObjectLib.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.StatesLib.*;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Models.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Models.Target;
import org.firstinspires.ftc.teamcode.pedroPathing.Services.IOStream;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class CustomOpMode extends OpMode {
    private final List<PeriodicRunnable> periodicList = new ArrayList<>();
    public void init(){
        Logger.initialize(false,System::nanoTime);
        follower = Constants.createFollower(hardwareMap);
        gamepad = GamepadEx.GetGamepadEx(gamepad1);
        Manager = TaskManager.getInstance();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        ioStream = new IOStream(hardwareMap.appContext);
        if(Objects.equals(ioStream.getData("Alliance"), "Red")){
            alliance = Alliance.Red;
        }else{
            alliance = Alliance.Blue;
        }
        logger = Logger.getINSTANCE();
        IntakeMotor = new MotorEx.MotorBuilder("IntakeMotor", MotorType.goBILDA,5.2,0,false,hardwareMap)
                .addVelocityAction(PullIn,1)
                .addVelocityAction(Stop,0)
                .addVelocityAction(Out,-1)
                .build();
        Shooter = new MotorEx.MotorBuilder("RightShooter",MotorType.goBILDA,1,0,false,hardwareMap,new PIDFCoefficients(0,0,0,0),null)
                .addMotorWithVelPIDF("LeftShooter",false,new PIDFCoefficients(0,0,0,0))
                .addVelocityAction(Armed,2)
                .build();
        Inhale = new MotorEx.MotorBuilder("InhaleMotor",MotorType.goBILDA,5.2,0,false,hardwareMap)
                .addVelocityAction(PullIn,1)
                .addVelocityAction(Stop,0)
                .addVelocityAction(Out,-1)
                .build();
        ClassifyServo = new ServoBuilders.NormalCRServoBuilder("ClassifyServo", Stop,0,false,0.09,hardwareMap)
                .addAction(Purple,1)
                .addAction(Green,0)
                .build();
        LeftBoard = new ServoBuilders.PWMServoBuilder("LeftBoard",0,false,hardwareMap)
                .addAction(Lock,0.8)
                .addAction(Shoot,0.5)
                .addAction(Back,0)
                .build();
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
        ioStream.saveData("Alliance",alliance.toString());
    }
    public void loop(){
        gamepad.update();
        telemetryM.update();
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
        }
    }
    public void stop(){
        logger.close();
    }
}