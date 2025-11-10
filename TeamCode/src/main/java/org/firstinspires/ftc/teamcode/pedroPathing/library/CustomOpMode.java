package org.firstinspires.ftc.teamcode.pedroPathing.library;

import static com.bear27570.yuan.BotFactory.Model.Action.*;

import com.bear27570.yuan.AdvantageCoreLib.Logging.Logger;
import com.bear27570.yuan.BotFactory.Gamepad.GamepadEx;
import com.bear27570.yuan.BotFactory.Interface.Lockable;
import com.bear27570.yuan.BotFactory.Interface.PeriodicRunnable;
import com.bear27570.yuan.BotFactory.Model.Action;
import com.bear27570.yuan.BotFactory.Model.MotorType;
import com.bear27570.yuan.BotFactory.Model.VirtualLock;
import com.bear27570.yuan.BotFactory.Motor.MotorEx;
import com.bear27570.yuan.BotFactory.Servo.ServoBuilders;
import com.bear27570.yuan.BotFactory.ThreadManagement.TaskManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import static org.firstinspires.ftc.teamcode.pedroPathing.library.ObjectLib.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.StatesLib.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.ConstantLib.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Models.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Models.Mode;
import org.firstinspires.ftc.teamcode.pedroPathing.Services.IOStream;
import org.firstinspires.ftc.teamcode.vision.Deadeye.Deadeye;
import org.firstinspires.ftc.teamcode.vision.EchoLapse.PinpointPoseProvider;
import org.firstinspires.ftc.teamcode.vision.QuickScope.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.vision.QuickScope.ArcherLogic;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class CustomOpMode extends OpMode {
    private final List<PeriodicRunnable> periodicList = new ArrayList<>();
    private double[] assistCoefficients = {0.0,0.0};
    public void init(){
        Logger.initialize(false,System::nanoTime);
        follower = Constants.createFollower(hardwareMap);
        gamepad = GamepadEx.GetGamepadEx(gamepad1);
        Manager = TaskManager.getInstance();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        ioStream = new IOStream(hardwareMap.appContext);
        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap);
        pinpointPoseProvider = new PinpointPoseProvider(hardwareMap, "odo");
        pinpointPoseProvider.initialize();
        archerLogic = new ArcherLogic();
        gamepadRumbleLock = new VirtualLock("gamepadRumbleLock");
        calculatorLock = new VirtualLock("calculatorLock");
        DeadeyeLock = new VirtualLock("DeadeyeLock");
        deadeye = new Deadeye(hardwareMap);
        deadeye.start();

        if(Objects.equals(ioStream.getData("Alliance"), "Red")){
            alliance = Alliance.Red;
        }else{
            alliance = Alliance.Blue;
        }
        logger = Logger.getINSTANCE();
        IntakeMotor = new MotorEx.MotorBuilder("IntakeMotor",MotorType.goBILDA,5.2,0,true,hardwareMap,new PIDFCoefficients(150,10,30,25),null)
                .addVelocityAction(PullIn,15)
                .addVelocityAction(Stop,0)
                .addVelocityAction(Out,-15)
                .build();
        Shooter = new MotorEx.MotorBuilder("RightShooter",MotorType.goBILDA,1,0,true,hardwareMap,new PIDFCoefficients(180,0,20,15.85),null)
                .addMotorWithVelPIDF("LeftShooter",true,new PIDFCoefficients(160,0,90,18))
                .addVelocityAction(Armed,7)
                .build();
        Inhale = new MotorEx.MotorBuilder("InhaleMotor",MotorType.goBILDA,5.2,0,false,hardwareMap,new PIDFCoefficients(17,3,8,13),null)
                .addVelocityAction(PullIn,6)
                .addVelocityAction(Stop,0)
                .addVelocityAction(Out,-6)
                .build();
        LeftBoard = new ServoBuilders.PWMServoBuilder("LeftBoard",Lock,0.53,false,hardwareMap)
                .addAction(Shoot,0.795)
                .addAction(Back,0.97)
                .build();

        RightBoard = new ServoBuilders.PWMServoBuilder("RightBoard",Lock,0.90,false,hardwareMap)
                .addAction(Shoot,0.65)
                .addAction(Back,0.4)
                .build();

        PitchServo = new ServoBuilders.PWMServoBuilder("RightPitch",Up,1,true,hardwareMap)
                .addServo("LeftPitch",false)
                .addAction(Down,1)
                .build();
        switch (target){
            case TELEOP:
                mode = Mode.IntakeMode;
                IntakeMotor.Init();
                Inhale.Init();
                Shooter.Init();
                LeftBoard.act(Lock);
                RightBoard.act(Lock);
                PitchServo.act(Up);
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
        pinpointPoseProvider.update();
        switch (target){
            case TELEOP:
                double coefficient = gamepad.left_trigger.PressPosition();
                if(mode == Mode.IntakeMode||mode==Mode.Manual_Shooting) {
                    follower.update();
                    double forward = (-gamepad.left_stick_y.PressPosition())*(1-ConstantLib.ASSISTANT_STRENGTH)+assistCoefficients[0]*ConstantLib.ASSISTANT_STRENGTH;
                    double strafe = (-gamepad.left_stick_x.PressPosition())*(1-ConstantLib.ASSISTANT_STRENGTH)+assistCoefficients[1]*ConstantLib.ASSISTANT_STRENGTH;
                    if (coefficient > 0.95) {
                        if(follower.isTeleopDrive())
                            follower.breakFollowing();
                        follower.holdPoint(follower.getPose());
                        return;
                    }
                    if (!follower.isTeleopDrive()) {
                        follower.startTeleOpDrive();
                    }
                    if (coefficient < 0.05) {
                        follower.setTeleOpDrive(Math.pow(-gamepad.left_stick_y.PressPosition(), 3),
                                Math.pow(-gamepad.left_stick_x.PressPosition(), 3),
                                Math.pow(-gamepad.right_stick_x.PressPosition(), 3),
                                true);
                        return;
                    }
                    follower.setTeleOpDrive(Math.pow(-gamepad.left_stick_y.PressPosition(), 3) * (1 - coefficient),
                            Math.pow(-gamepad.left_stick_x.PressPosition(), 3) * (1 - coefficient),
                            Math.pow(-gamepad.right_stick_x.PressPosition(), 3) * (1 - coefficient),
                            true);
                    break;
                }
                if(mode == Mode.ShootingMode){
                    if (coefficient > 0.95) {
                        if(follower.isHybridDrive())
                            follower.breakFollowing();
                        follower.holdPoint(follower.getPose());
                        return;
                    }
                    if (!follower.isHybridDrive()) {
                        follower.startTunerStyleHybridDrive();
                    }
                    if (coefficient < 0.05) {
                        follower.updateTunerStyleHybridDrive(Math.pow(-gamepad.left_stick_y.PressPosition(), 3),
                                Math.pow(-gamepad.left_stick_x.PressPosition(), 3),
                                Math.toRadians(latestSolution.get().aimAzimuthDeg-90)
                        );
                        return;
                    }
                    follower.updateTunerStyleHybridDrive(Math.pow(-gamepad.left_stick_y.PressPosition()*(1-coefficient), 3),
                            Math.pow(-gamepad.left_stick_x.PressPosition()*(1-coefficient), 3),
                            Math.toRadians(latestSolution.get().aimAzimuthDeg-90)
                    );
                    break;
                }
                case AUTONOMOUS:
                        break;
        }
    }
    public void stop(){
        logger.close();
    }
}