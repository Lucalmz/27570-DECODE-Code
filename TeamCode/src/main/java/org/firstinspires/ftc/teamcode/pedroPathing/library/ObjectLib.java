package org.firstinspires.ftc.teamcode.pedroPathing.library;

import com.bear27570.yuan.AdvantageCoreLib.Logging.Logger;
import com.bear27570.yuan.BotFactory.Gamepad.GamepadEx;
import com.bear27570.yuan.BotFactory.Motor.MotorEx;
import com.bear27570.yuan.BotFactory.ThreadManagement.TaskManager;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.HeadingInterpolator;

import org.firstinspires.ftc.teamcode.pedroPathing.Services.IOStream;

public class ObjectLib {
    public static MotorEx Shooter;
    public static GamepadEx gamepad;
    public static Logger logger;
    public static IOStream ioStream;
    public static TaskManager Manager;
    public static Follower follower;
    public static TelemetryManager telemetryM;
    public static HeadingInterpolator VisionCalculatedHeading = closestPoint ->
            MathFunctions.normalizeAngle(Math.toRadians(90));
    private ObjectLib (){}
}