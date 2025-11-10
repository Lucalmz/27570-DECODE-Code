package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.library.AlgorithmLib.Aim;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.AlgorithmLib.IntakeStruct;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.AlgorithmLib.PopGreen;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.AlgorithmLib.PopPurple;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.AlgorithmLib.ShootAll;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.AlgorithmLib.ShootGreen;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.AlgorithmLib.ShootPurple;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.AlgorithmLib.checkShooterVelocity;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.AlgorithmLib.getNewLaunch;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.AlgorithmLib.updatePosition;
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
import org.firstinspires.ftc.teamcode.pedroPathing.library.CustomOpMode;
import org.firstinspires.ftc.teamcode.vision.Deadeye.Deadeye;

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
        currentPose = aprilTagLocalizer.getRobotPose();
        Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                .require(PitchServo)
                .require(LeftBoard)
                .require(RightBoard)
                .require(Shooter)
                .require(IntakeMotor)
                .require(Inhale)
                .runs(IntakeStruct())
                .build());
        Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.INTERRUPT)
                .require(gamepadRumbleLock)
                .require(fetchingLocalizerLock)
                .runs(updatePosition())
                .build());
    }

    @Override
    public void loop() {
        super.loop();
        Manager.submit(new Task.TaskBuilder(Priority.HIGH, ConflictPolicy.IGNORE)
                .require(calculatorLock)
                .require(fetchingLocalizerLock)
                .runs(() -> {
                    getNewLaunch();
                    telemetry.addLine(">> 方案已解算 <<");
                    telemetry.addData("发射电机转速 (RPM)", "%.0f", latestSolution.get().motorRpm);
                    telemetry.addData("偏航角 (Yaw)", "%.2f deg", latestSolution.get().aimAzimuthDeg);
                    telemetry.addData("俯仰角 (Pitch)", "%.2f deg", latestSolution.get().launcherAngle);
                    telemetry.addData("坐标","(%.2f,%.2f)",pinpointPoseProvider.getX(DistanceUnit.CM),pinpointPoseProvider.getY(DistanceUnit.CM));
                })
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
                    .runs(checkShooterVelocity())
                    .build());
            mode = Mode.ShootingMode;
        }
        if (gamepad.left_bumper.Pressed()) {
            Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                    .require(Inhale)
                    .require(IntakeMotor)
                    .runs(() -> {
                        Inhale.VelocityAct(Stop);
                        IntakeMotor.VelocityAct(Stop);
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
                    .runs(IntakeStruct())
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
        if (gamepad.circle.Pressed()) {
            Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                    .require(LeftBoard)
                    .require(RightBoard)
                    .require(Inhale)
                    .runs(ShootAll())
                    .build());
        }
        //模式逻辑
        switch (mode) {
            case ShootingMode:
                Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.IGNORE)
                        .require(PitchServo)
                        .require(Shooter)
                        .runs(Aim())
                        .build());
                if (gamepad.cross.Pressed()) {
                    Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                            .require(LeftBoard)
                            .require(Inhale)
                            .runs(ShootGreen())
                            .build());
                }
                if (gamepad.square.Pressed()) {
                    Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                            .require(RightBoard)
                            .require(Inhale)
                            .runs(ShootPurple())
                            .build());
                }
                break;
            case Manual_Shooting:

                if (gamepad.cross.Pressed()) {
                    Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                            .require(LeftBoard)
                            .require(Inhale)
                            .runs(ShootGreen())
                            .build());
                }
                if (gamepad.square.Pressed()) {
                    Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                            .require(RightBoard)
                            .require(Inhale)
                            .runs(ShootPurple())
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
                            .runs(PopGreen())
                            .build());
                }
                if (gamepad.square.Pressed()) {
                    Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                            .require(RightBoard)
                            .runs(PopPurple())
                            .build());
                }
                break;
        }
    }

    @Override
    public void stop() {
        super.stop();
        DeadeyeAPI.stop();
    }
}