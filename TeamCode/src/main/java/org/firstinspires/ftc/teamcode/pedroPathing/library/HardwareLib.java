package org.firstinspires.ftc.teamcode.pedroPathing.library;

import com.bear27570.yuan.BotFactory.Gamepad.GamepadEx;
import com.bear27570.yuan.BotFactory.Motor.MotorEx;
import com.bear27570.yuan.BotFactory.Servo.ServoEx;
import com.bear27570.yuan.BotFactory.ThreadManagement.TaskManager;

public class HardwareLib {
    public static MotorEx TestMotor1;
    public static MotorEx TestMotor2;
    public static ServoEx TestServo;
    public static GamepadEx gamepad;
    public static TaskManager Manager = TaskManager.getInstance();
}