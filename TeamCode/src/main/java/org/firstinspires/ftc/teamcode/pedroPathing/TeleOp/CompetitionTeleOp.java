package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.library.AlgorithmLib.Aim;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.AlgorithmLib.CalculateNewLaunch;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.AlgorithmLib.ShootAll;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.AlgorithmLib.ShootGreen;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.AlgorithmLib.ShootPurple;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.ObjectLib.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.StatesLib.*;

import static com.bear27570.yuan.BotFactory.Model.Action.*;

import com.bear27570.yuan.BotFactory.Model.ConflictPolicy;
import com.bear27570.yuan.BotFactory.Model.Priority;
import com.bear27570.yuan.BotFactory.ThreadManagement.Task;
import com.bear27570.yuan.BotFactory.ThreadManagement.TaskManager;
import com.pedropathing.geometry.Pose;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.pedroPathing.Models.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Models.Mode;
import org.firstinspires.ftc.teamcode.pedroPathing.Models.Target;
import org.firstinspires.ftc.teamcode.pedroPathing.Services.Calculator;
import org.firstinspires.ftc.teamcode.pedroPathing.library.CustomOpMode;
import org.firstinspires.ftc.teamcode.vision.Deadeye.Deadeye;

@TeleOp
public class CompetitionTeleOp extends CustomOpMode {
    Deadeye DeadeyeAPI;
    boolean flag;
    double[] coords;

    @Override
    public void init() {
        target = Target.TELEOP;
        super.init();
        DeadeyeAPI = new Deadeye(hardwareMap);
        DeadeyeAPI.start();
    }

    @Override
    public void init_loop() {
        super.init_loop();
        gamepad.update();
        telemetryM.addData("Select running alliance! Current alliance:", alliance.toString());
        telemetryM.update();
        telemetry.addData("Select running alliance! Current alliance:", alliance.toString());
        telemetry.update();
        if (gamepad.left_bumper.Pressed()) {
            alliance = Alliance.Red;
        }
        if (gamepad.right_bumper.Pressed()) {
            alliance = Alliance.Blue;
        }
    }

    @Override
    public void start() {
        super.start();
        currentPose = localizer.getRobotPose();
    }

    @Override
    public void loop() {
        super.loop();
        if (currentPose == null) {
            Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.IGNORE)
                    .require(gamepadRumbleLock)
                    .runs(() -> {
                        gamepad1.rumble(0.5, 0.5, 50);
                        currentPose = localizer.getRobotPose();
                    })
                    .build());
            flag = true;
        }
        if (flag && currentPose != null) {
            localizer.close();
            gamepad.stopRumble();
            follower.setPose(new Pose(currentPose.getX(DistanceUnit.INCH), currentPose.getY(DistanceUnit.INCH), currentPose.getHeading(AngleUnit.RADIANS)));
            follower.update();
            flag = false;
        }
        //射击模式切换
        if (gamepad.right_bumper.Pressed()) {
            Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                    .require(Inhale)
                    .require(IntakeMotorLock)
                    .runs(() -> {
                        Inhale.act(Stop);
                        IntakeMotor.setPower(0);
                    })
                    .build());
            mode = Mode.ShootingMode;
        }
        if (gamepad.left_bumper.Pressed()) {
            Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                    .require(Inhale)
                    .require(IntakeMotorLock)
                    .runs(() -> {
                        Inhale.act(Stop);
                        IntakeMotor.setPower(0);
                    })
                    .build());
            mode = Mode.Manual_Shooting;
        }
        if (gamepad.right_bumper.justReleased() || gamepad.left_bumper.justReleased()) {
            Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                    .require(PitchServo)
                    .require(LeftBoard)
                    .require(RightBoard)
                    .require(Shooter)
                    .runs(() -> {
                        PitchServo.act(Up);
                        LeftBoard.act(Lock);
                        RightBoard.act(Lock);
                        Shooter.act(Armed);
                    })
                    .build());
            mode = Mode.IntakeMode;
        }
        if (gamepad.left_bumper.isPressing() && gamepad.right_bumper.isPressing()) {
            return;
        }
        //模式逻辑
        switch (mode) {
            case ShootingMode:
                if (gamepad.circle.Pressed()) {
                    Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.IGNORE)
                            .require(LeftBoard)
                            .require(RightBoard)
                            .require(Inhale)
                            .runs(ShootAll())
                            .build());
                }/*
                if (Shooter.getVelocity() < calculator.calculateMotorRps(latestSolution.launcherVelocity)) {
                    Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                            .require(gamepadRumbleLock)
                            .runs(() -> {
                                gamepad.rumble(0.8, 0.8, 200);
                                try {
                                    Thread.sleep(200);
                                    gamepad.stopRumble();
                                    Thread.sleep(100);
                                } catch (InterruptedException e) {
                                    Thread.currentThread().interrupt();
                                    throw new RuntimeException(e);
                                }
                            })
                            .build());
                }
                Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.IGNORE)
                        .require(calculatorLock)
                        .runs(() -> {
                            latestSolution = CalculateNewLaunch();
                        })
                        .build());
                Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.IGNORE)
                        .require(PitchServo)
                        .require(Shooter)
                        .require(LeftBoard)
                        .require(RightBoard)
                        .runs(Aim(latestSolution))
                        .build());
                telemetry.addData("Launcher Pitch", calculator.getSolution().launcherPitch);
                telemetry.addData("Launcher Azimuth", calculator.getSolution().aimAzimuth);
                telemetry.addData("Launcher Velocity", calculator.getSolution().launcherVelocity);
                telemetry.addData("RobotPose", currentPose.toString());
                telemetry.addData("Position", odo.getPosition());
                telemetry.update();
                break;*/
            case Manual_Shooting:
                if (gamepad.circle.Pressed()) {
                    Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                            .require(LeftBoard)
                            .require(RightBoard)
                            .require(Inhale)
                            .runs(ShootAll())
                            .build());
                }
                if (gamepad.cross.Pressed()) {
                    Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                            .require(LeftBoard)
                            .require(Inhale)
                            .runs(ShootPurple())
                            .build());
                }
                if (gamepad.square.Pressed()) {
                    Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                            .require(RightBoard)
                            .require(Inhale)
                            .runs(ShootGreen())
                            .build());
                }
                break;

            case IntakeMode:
                Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.IGNORE)
                        .require(IntakeMotorLock)
                        .require(Inhale)
                        .runs(() -> {
                            IntakeMotor.setPower(1);
                            Inhale.setVelocity(5);
                        })
                        .build());
                break;
        }
    }

    @Override
    public void stop() {
        super.stop();
        DeadeyeAPI.stop();
    }
}