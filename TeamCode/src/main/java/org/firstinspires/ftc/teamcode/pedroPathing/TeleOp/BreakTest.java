package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.bear27570.yuan.BotFactory.Gamepad.GamepadEx;
import com.bear27570.yuan.BotFactory.ThreadManagement.Task;
import com.bear27570.yuan.BotFactory.ThreadManagement.TaskManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import static com.bear27570.yuan.BotFactory.Model.Priority.*;
import static com.bear27570.yuan.BotFactory.Model.ConflictPolicy.*;

public class BreakTest extends OpMode{
    private DcMotorEx motor;
    private GamepadEx gamepad;
    private TaskManager taskManager;
    private Task task;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        gamepad = GamepadEx.GetGamepadEx(gamepad1);
        try {
            task  = new Task.TaskBuilder(LOW,IGNORE)
                    .runs(inLoop())
                    .build();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void loop() {
        if(gamepad.circle.Pressed()){
            taskManager.submit(task);
        }
    }
    private Runnable inLoop() throws InterruptedException {
        double targetBreak = gamepad1.left_trigger;
        double power = gamepad1.left_stick_y;
        Thread.sleep(100000);
        setMotorPower(targetBreak,power);
        return null;
    }
    private void setMotorPower(double Break,double power) {
        if(Break > 0.96){
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            return;
        }
        if(Break < 0.06){
            motor.setPower(power);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        motor.setPower(power*(1-Break));
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}