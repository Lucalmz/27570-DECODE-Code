package org.firstinspires.ftc.teamcode.pedroPathing.library;

import com.bear27570.yuan.AdvantageCoreLib.Logging.Logger;
import com.bear27570.yuan.BotFactory.Gamepad.GamepadEx;
import com.bear27570.yuan.BotFactory.Interface.Lockable;
import com.bear27570.yuan.BotFactory.Interface.ServoEx;
import com.bear27570.yuan.BotFactory.Motor.MotorEx;
import com.bear27570.yuan.BotFactory.Servo.CRServoEx;
import com.bear27570.yuan.BotFactory.ThreadManagement.TaskManager;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.HeadingInterpolator;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Services.IOStream;
import org.firstinspires.ftc.teamcode.vision.Deadeye.Deadeye;
import org.firstinspires.ftc.teamcode.vision.EchoLapse.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.vision.EchoLapse.PinpointPoseProvider;
import org.firstinspires.ftc.teamcode.vision.QuickScope.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.vision.QuickScope.ArcherLogic;
import org.firstinspires.ftc.teamcode.vision.QuickScope.LaunchSolution;

import java.util.concurrent.atomic.AtomicReference;

public class ObjectLib {
    public static MotorEx Shooter;
    public static DcMotor IntakeMotor;
    public static MotorEx Inhale;
    public static CRServoEx ClassifyServo;
    public static ServoEx PitchServo;
    public static ServoEx LeftBoard;
    public static ServoEx RightBoard;
    public static GamepadEx gamepad;
    public static Logger logger;
    public static IOStream ioStream;
    public static TaskManager Manager;
    public static Follower follower;
    public static TelemetryManager telemetryM;
    public static AprilTagLocalizer aprilTagLocalizer;
    public static PinpointPoseProvider pinpointPoseProvider;
    public static ArcherLogic archerLogic;
    public static Deadeye deadeye;
    public static Lockable gamepadRumbleLock,calculatorLock,DeadeyeLock,IntakeMotorLock;
    /*public static HeadingInterpolator VisionCalculatedHeading = closestPoint ->
            MathFunctions.normalizeAngle(Math.toRadians(90));*/
    private ObjectLib (){}
}