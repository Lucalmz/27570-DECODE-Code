package org.firstinspires.ftc.teamcode.pedroPathing.library;

import static com.bear27570.yuan.BotFactory.Model.Action.*;

import com.bear27570.yuan.AdvantageCoreLib.Logging.Logger;
import com.bear27570.yuan.BotFactory.Gamepad.GamepadEx;
import com.bear27570.yuan.BotFactory.Model.MotorType;
import com.bear27570.yuan.BotFactory.Model.VirtualLock;
import com.bear27570.yuan.BotFactory.Motor.MotorEx;
import com.bear27570.yuan.BotFactory.Servo.ServoBuilders;
import com.bear27570.yuan.BotFactory.ThreadManagement.TaskManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.TouchSensor;

import static org.firstinspires.ftc.teamcode.pedroPathing.library.ObjectLib.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.StatesLib.*;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Models.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Models.Mode;
import org.firstinspires.ftc.teamcode.pedroPathing.Services.Calculator;
import org.firstinspires.ftc.teamcode.pedroPathing.Services.IOStream;
import org.firstinspires.ftc.teamcode.pedroPathing.Services.TouchSensorCounter;
import org.firstinspires.ftc.teamcode.vision.Deadeye.Deadeye;
import org.firstinspires.ftc.teamcode.vision.EchoLapse.FollowerPoseProvider;
import org.firstinspires.ftc.teamcode.vision.QuickScope.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.vision.QuickScope.ArcherLogic;

import java.util.Objects;

public class CustomOpMode extends OpMode {
    public void init() {
        Logger.initialize(false, System::nanoTime);
        follower = Constants.createAdvancedFollower(hardwareMap);
        gamepad = GamepadEx.GetGamepadEx(gamepad1);
        Manager = TaskManager.getInstance();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        ioStream = new IOStream(hardwareMap.appContext);
        archerLogic = new ArcherLogic();
        gamepadRumbleLock = new VirtualLock("gamepadRumbleLock");
        calculatorLock = new VirtualLock("calculatorLock");
        DeadeyeLock = new VirtualLock("DeadeyeLock");
        fetchingLocalizerLock = new VirtualLock("fetchingLocalizerLock");
        deadeye = new Deadeye(hardwareMap);
        deadeye.start();

        if (Objects.equals(ioStream.getData("Alliance"), "Red")) {
            alliance = Alliance.Red;
        } else {
            alliance = Alliance.Blue;
        }
        logger = Logger.getINSTANCE();
        touchSensor = new TouchSensorCounter(hardwareMap.get(TouchSensor.class,"TouchSensor"),50);
        IntakeMotor = new MotorEx.MotorBuilder("IntakeMotor", MotorType.goBILDA, 5.2, 0, false, hardwareMap, new PIDFCoefficients(150, 10, 30, 25), null)
                .addVelocityAction(PullIn, 20)
                .addVelocityAction(Stop, 0)
                .addVelocityAction(Out, -20)
                .build();
        ClassifyServo = new ServoBuilders.CRServoBuilder("ClassifyServo",Stop,0,false,hardwareMap)
                .addAction(Purple,1)
                .addAction(Green,-1)
                .build();
        Shooter = new MotorEx.MotorBuilder("RightShooter", MotorType.goBILDA, 1, 0, true, hardwareMap, new PIDFCoefficients(180, 0.27, 60, 18.3), null)
                .addMotorWithVelPIDF("LeftShooter", true, new PIDFCoefficients(180, 0.7, 13.5, 40))
                .addVelocityAction(Armed, 7)
                .addVelocityAction(ShootPose1, 23)
                .addVelocityAction(ShootPose2,30)
                .addVelocityAction(AutoShootPose,26)
                .build();
        Inhale = new MotorEx.MotorBuilder("InhaleMotor", MotorType.goBILDA, 5.2, 0, false, hardwareMap, new PIDFCoefficients(17, 3, 8, 13), null)
                .addVelocityAction(PullIn, 6)
                .addVelocityAction(Stop, 0)
                .addVelocityAction(Out, -6)
                .addVelocityAction(Shoot,13)
                .build();
        LeftBoard = new ServoBuilders.PWMServoBuilder("LeftBoard", Lock, 0.53, false, hardwareMap)
                .addAction(Shoot, 0.82)
                .addAction(Back, 0.97)
                .build();

        RightBoard = new ServoBuilders.PWMServoBuilder("RightBoard", Lock, 0.90, false, hardwareMap)
                .addAction(Shoot, 0.618)
                .addAction(Back, 0.4)
                .build();

        PitchServo = new ServoBuilders.PWMServoBuilder("LeftPitch", Up, 1, false, hardwareMap)
                .addServo("RightPitch", true)
                .addAction(Down, 0)
                .addAction(AutoShootPose, Calculator.DegreeToPitchServo(60))
                .setPositionDifference(0.007)
                .build();
        switch (target) {
            case TELEOP:
                follower.setStartingPose(new Pose(0, 0, 0));
                follower.startTeleopDrive();
                aprilTagLocalizer = new AprilTagLocalizer(hardwareMap);
                pinpointPoseProvider = new FollowerPoseProvider(follower);
                pinpointPoseProvider.initialize();
                mode = Mode.IntakeMode;
                IntakeMotor.Init();
                Inhale.Init();
                Shooter.Init();
                LeftBoard.act(Lock);
                RightBoard.act(Lock);
                PitchServo.act(Up);
                ManualShootRatio=0;
                break;
            case AUTONOMOUS:
                IntakeMotor.Init();
                Inhale.Init();
                Shooter.Init();
                LeftBoard.act(Lock);
                RightBoard.act(Lock);
                PitchServo.act(AutoShootPose);
                break;
        }
    }

    public void init_loop() {
    }

    public void start() {
        ioStream.saveData("Alliance", alliance.name());
        HeadingForFieldInAlliance = 135;
        if (alliance == Alliance.Blue) {
            HeadingForFieldInAlliance = 70;
        }
    }

    public void loop() {
        switch (target) {
            case TELEOP:
                gamepad.update();
                telemetryM.update();
                pinpointPoseProvider.update();
                touchSensor.update();
                double coefficient = gamepad.left_trigger.PressPosition();
                if (mode == Mode.IntakeMode || mode == Mode.Manual_Shooting) {
                    follower.update();
                    if (coefficient > 0.95) {
                        if (follower.isTeleopDrive())
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
                if (mode == Mode.ShootingMode) {
                    if (coefficient > 0.95) {
                        if (follower.isHybridDrive())
                            follower.breakFollowing();
                        follower.holdPoint(follower.getPose());
                        return;
                    }
                    if (!follower.isHybridDrive()) {
                        follower.startTunerStyleHybridDrive(Math.toRadians(HeadingForFieldInAlliance));
                    }
                    if (coefficient < 0.05) {
                        follower.updateTunerStyleHybridDrive(Math.pow(-gamepad.left_stick_y.PressPosition(), 3),
                                Math.pow(gamepad.left_stick_x.PressPosition(), 3),
                                Math.toRadians(filter.update(latestSolution.get().aimAzimuthDeg))
                        );
                        return;
                    }
                    follower.updateTunerStyleHybridDrive(Math.pow(-gamepad.left_stick_y.PressPosition() * (1 - coefficient), 3),
                            Math.pow(gamepad.left_stick_x.PressPosition() * (1 - coefficient), 3),
                            Math.toRadians(filter.update(latestSolution.get().aimAzimuthDeg))
                    );
                    break;
                }
            case AUTONOMOUS:
                break;
        }
    }

    public void stop() {
        logger.close();
    }
}