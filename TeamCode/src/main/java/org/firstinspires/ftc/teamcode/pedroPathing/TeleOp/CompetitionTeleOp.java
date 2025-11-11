package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.library.AlgorithmLib.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.ObjectLib.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.StatesLib.*;

import static com.bear27570.yuan.BotFactory.Model.Action.*;

import com.bear27570.yuan.BotFactory.Model.ConflictPolicy;
import com.bear27570.yuan.BotFactory.Model.Priority;
import com.bear27570.yuan.BotFactory.ThreadManagement.Task;
import com.bear27570.yuan.BotFactory.ThreadManagement.TaskManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Models.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Models.Mode;
import org.firstinspires.ftc.teamcode.pedroPathing.Models.Target;
import org.firstinspires.ftc.teamcode.pedroPathing.Services.Calculator;
import org.firstinspires.ftc.teamcode.pedroPathing.Services.KalmanFilter;
import org.firstinspires.ftc.teamcode.pedroPathing.library.AlgorithmLib;
import org.firstinspires.ftc.teamcode.pedroPathing.library.CustomOpMode;
import org.firstinspires.ftc.teamcode.vision.Deadeye.Deadeye;
import org.firstinspires.ftc.teamcode.vision.QuickScope.AprilTagLocalizer;

import java.util.IllegalFormatCodePointException;

@TeleOp
public class CompetitionTeleOp extends CustomOpMode {
    Deadeye DeadeyeAPI;

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
        Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                .require(PitchServo)
                .require(LeftBoard)
                .require(RightBoard)
                .require(Shooter)
                .require(IntakeMotor)
                .require(Inhale)
                .runs(IntakeStruct())
                .build());
        Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                .require(fetchingLocalizerLock)
                .runs(AlgorithmLib::setNewPosition)
                .build());
    }

    @Override
    public void loop() {
        super.loop();
        Manager.submit(new Task.TaskBuilder(Priority.HIGH, ConflictPolicy.IGNORE)
                .require(calculatorLock)
                .require(fetchingLocalizerLock)
                .runs(AlgorithmLib::getNewLaunch)
                .build());
        //射击模式切换
        if (gamepad.right_bumper.Pressed()) {
            Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                    .require(Inhale)
                    .require(IntakeMotor)
                    .runs(() -> {
                        Inhale.VelocityAct(Stop);
                        IntakeMotor.VelocityAct(Stop);
                    })
                    .build());
            Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                    .require(gamepadRumbleLock)
                    .runs(AlgorithmLib::checkShooterVelocity)
                    .build());
            filter = new KalmanFilter(latestSolution.get().aimAzimuthDeg,1,0.8,1);
            mode = Mode.ShootingMode;
        }
        if (gamepad.left_bumper.Pressed()) {
            Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                    .require(Inhale)
                    .require(IntakeMotor)
                    .require(Shooter)
                    .runs(() -> {
                        Inhale.VelocityAct(Stop);
                        IntakeMotor.VelocityAct(Stop);
                        Shooter.VelocityAct(Shoot);
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
                    .require(IntakeMotor)
                    .require(Inhale)
                    .runs(AlgorithmLib::IntakeStruct)
                    .build());
            Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                    .require(IntakeMotor)
                    .require(Inhale)
                    .runs(() -> {
                        IntakeMotor.VelocityAct(PullIn);
                        Inhale.VelocityAct(PullIn);
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
                Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.IGNORE)
                        .require(PitchServo)
                        .require(Shooter)
                        .runs(AlgorithmLib::Aim)
                        .build());
                if (gamepad.cross.Pressed()) {
                    Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                            .require(LeftBoard)
                            .require(Inhale)
                            .runs(AlgorithmLib::ShootGreen)
                            .build());
                }
                if (gamepad.square.Pressed()) {
                    Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                            .require(RightBoard)
                            .require(Inhale)
                            .runs(AlgorithmLib::ShootPurple)
                            .build());
                }
                if (gamepad.circle.Pressed()) {
                    Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                            .require(LeftBoard)
                            .require(RightBoard)
                            .require(Inhale)
                            .runs(AlgorithmLib::ShootAll)
                            .build());
                }
                break;
            case Manual_Shooting:

                if (gamepad.cross.Pressed()) {
                    Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                            .require(LeftBoard)
                            .require(Inhale)
                            .runs(AlgorithmLib::ShootGreen)
                            .build());
                }
                if (gamepad.square.Pressed()) {
                    Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                            .require(RightBoard)
                            .require(Inhale)
                            .runs(AlgorithmLib::ShootPurple)
                            .build());
                }
                break;

            case IntakeMode:
                Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.IGNORE)
                        .require(IntakeMotor)
                        .require(Inhale)
                        .runs(() -> {
                            IntakeMotor.VelocityAct(PullIn);
                            Inhale.VelocityAct(PullIn);
                        })
                        .build());
                Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.IGNORE)
                        .require(DeadeyeLock)
                        .runs(() -> {
                            deadeye.update();
                        })
                        .build()
                );
                if (gamepad.cross.Pressed()) {
                    Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                            .require(LeftBoard)
                            .runs(AlgorithmLib::PopGreen)
                            .build());
                }
                if (gamepad.square.Pressed()) {
                    Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                            .require(RightBoard)
                            .runs(AlgorithmLib::PopPurple)
                            .build());
                }
                break;
        }
    }

    @Override
    public void stop() {
        super.stop();
        DeadeyeAPI.stop();
        if (aprilTagLocalizer != null) {
            aprilTagLocalizer.close();
        }
    }
}