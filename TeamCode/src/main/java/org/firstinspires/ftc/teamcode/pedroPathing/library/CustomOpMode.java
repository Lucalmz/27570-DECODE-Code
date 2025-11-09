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
import org.firstinspires.ftc.teamcode.pedroPathing.Models.Target;
import org.firstinspires.ftc.teamcode.pedroPathing.Services.IOStream;
import org.firstinspires.ftc.teamcode.vision.EchoLapse.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.vision.QuickScope.AprilTagLocalizer;

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
        localizer = new AprilTagLocalizer(hardwareMap);
        gamepadRumbleLock = new VirtualLock("gamepadRumbleLock");
        calculatorLock = new VirtualLock("calculatorLock");
        DeadeyeLock = new VirtualLock("DeadeyeLock");
        IntakeMotorLock = new VirtualLock("IntakeMotorLock");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(85.0, -180.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        if(Objects.equals(ioStream.getData("Alliance"), "Red")){
            alliance = Alliance.Red;
        }else{
            alliance = Alliance.Blue;
        }
        logger = Logger.getINSTANCE();
        IntakeMotor = hardwareMap.get(DcMotor.class,"IntakeMotor");
        Shooter = new MotorEx.MotorBuilder("RightShooter",MotorType.goBILDA,1,0,true,hardwareMap,new PIDFCoefficients(180,0,20,15.85),null)
                .addMotorWithVelPIDF("LeftShooter",true,new PIDFCoefficients(160,0,90,18))
                .addVelocityAction(Armed,7)
                .build();
        Inhale = new MotorEx.MotorBuilder("InhaleMotor",MotorType.goBILDA,5.2,0,false,hardwareMap,new PIDFCoefficients(17,3,8,13),null)
                .addVelocityAction(PullIn,6)
                .addVelocityAction(Stop,0)
                .addVelocityAction(Out,-6)
                .build();
        ClassifyServo = new ServoBuilders.CRServoBuilder("ClassifyServo", Stop,0,false,hardwareMap)
                .addAction(Purple,-1)
                .addAction(Green,1)
                .build();

        LeftBoard = new ServoBuilders.PWMServoBuilder("LeftBoard",Lock,0.45,false,hardwareMap)
                .addAction(Shoot,0.795)
                .addAction(Back,0.97)
                .build();

        RightBoard = new ServoBuilders.PWMServoBuilder("RightBoard",Lock,0.98,false,hardwareMap)
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
                IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        follower.update();
        odo.update();
        switch (target){
            case TELEOP:
                double coefficient = gamepad.left_trigger.PressPosition();
                if(mode == Mode.IntakeMode||mode==Mode.Manual_Shooting) {
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
                        follower.setTeleOpDrive(Math.pow(forward, 3), Math.pow(strafe, 3), Math.pow(-gamepad.right_stick_x.PressPosition(), 3), true);
                        return;
                    }
                    follower.setTeleOpDrive(Math.pow(forward, 3) * (1 - coefficient), Math.pow(strafe, 3) * (1 - coefficient), Math.pow(-gamepad.right_stick_x.PressPosition(), 3) * (1 - coefficient), true);
                    break;
                }
                if(mode == Mode.ShootingMode){
                    if (coefficient > 0.95) {
                        if(follower.isHybridDriveRunning())
                            follower.breakFollowing();
                        follower.holdPoint(follower.getPose());
                        return;
                    }
                    if (!follower.isHybridDriveRunning()) {
                        //follower.startHybridDrive(calculator.getSolution().aimAzimuth+90);
                    }
                    if (coefficient < 0.05) {
                        //follower.setHybridDriveInputs(Math.pow(-gamepad.left_stick_y.PressPosition(), 3), Math.pow(-gamepad.left_stick_x.PressPosition(), 3), Math.toRadians(calculator.getSolution().aimAzimuth+90));
                        return;
                    }
                    //follower.setHybridDriveInputs(Math.pow(-gamepad.left_stick_y.PressPosition()*coefficient, 3), Math.pow(-gamepad.left_stick_x.PressPosition()*coefficient, 3), Math.toRadians(calculator.getSolution().aimAzimuth+90));
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