package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.library.AlgorithmLib.PopGreen;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.AlgorithmLib.PopPurple;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.AlgorithmLib.ShootAll;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.AlgorithmLib.ShootGreen;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.AlgorithmLib.ShootPurple;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.AlgorithmLib.getNewLaunch;
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
        currentPose = aprilTagLocalizer.getRobotPose();
    }

    @Override
    public void loop() {
        super.loop();
        if (currentPose == null) {
            Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.IGNORE)
                    .require(gamepadRumbleLock)
                    .runs(() -> {
                        gamepad1.rumble(0.5, 0.5, 50);
                        currentPose = aprilTagLocalizer.getRobotPose();
                        System.out.println("坐标：" + currentPose.getY(DistanceUnit.CM));
                        pinpointPoseProvider.setPose(currentPose);
                    })
                    .build());
            flag = true;
        }
        if (currentPose != null) {
            aprilTagLocalizer.close();
            gamepad.stopRumble();
            follower.setPose(new Pose(currentPose.getX(DistanceUnit.INCH), currentPose.getY(DistanceUnit.INCH), currentPose.getHeading(AngleUnit.RADIANS)));
            follower.update();
            pinpointPoseProvider.setPose(currentPose);
            flag = false;
        }
        if (!flag) {
            Manager.submit(new Task.TaskBuilder(Priority.HIGH, ConflictPolicy.IGNORE)
                    .require(calculatorLock)
                    .runs(() -> {
                        try {
                            getNewLaunch();
                        } catch (Exception e) {
                            throw new RuntimeException(e);
                        }
                        if (latestSolution != null) {
                            telemetry.addLine(">> 方案已解算 <<");
                            telemetry.addData("发射电机转速 (RPM)", "%.0f", latestSolution.get().motorRpm);
                            telemetry.addData("偏航角 (Yaw)", "%.2f deg", latestSolution.get().aimAzimuthDeg);
                            telemetry.addData("俯仰角 (Pitch)", "%.2f deg", latestSolution.get().launcherAngle);
                            telemetry.addData("坐标 (X,Y)", "%.2f, %.2f", currentPose.getX(DistanceUnit.CM), currentPose.getY(DistanceUnit.CM));
                            telemetry.addData("Pinpoint坐标(X,Y)", "%.2f, %.2f", pinpointPoseProvider.getX(DistanceUnit.CM), pinpointPoseProvider.getY(DistanceUnit.CM));
                            telemetry.addData("follower信息", follower.getPose().toString());
                            telemetry.update();
                        } else {
                            telemetry.addLine("NULL");
                            telemetry.update();
                        }
                    })
                    .build());
        }
        //射击模式切换
        if (gamepad.right_bumper.Pressed()) {
            Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                    .require(Inhale)
                    .require(IntakeMotor)
                    .runs(() -> {
                        Inhale.act(Stop);
                        IntakeMotor.act(Stop);
                    })
                    .build());
            mode = Mode.ShootingMode;
        }
        if (gamepad.left_bumper.Pressed()) {
            Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                    .require(Inhale)
                    .require(IntakeMotor)
                    .runs(() -> {
                        Inhale.act(Stop);
                        IntakeMotor.act(Stop);
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
                        .runs(() -> {
                            PitchServo.SetTemporaryPosition(Calculator.DegreeToPitchServo(latestSolution.get().launcherAngle));
                            Shooter.setVelocity(latestSolution.get().motorRpm / 60);
                        })
                        .build());

                if (Shooter.getVelocity() < latestSolution.get().motorRpm / 60) {
                    Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                            .require(gamepadRumbleLock)
                            .runs(() -> {
                                gamepad.rumble(0.8, 0.8, 60);
                                try {
                                    Thread.sleep(40);
                                } catch (InterruptedException e) {
                                    Thread.currentThread().interrupt();
                                    throw new RuntimeException(e);
                                }
                            })
                            .build());
                }
                break;
            case Manual_Shooting:

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
                        .require(IntakeMotor)
                        .require(Inhale)
                        .runs(() -> {
                            IntakeMotor.act(PullIn);
                            Inhale.act(PullIn);
                        })
                        .build());
                Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.IGNORE)
                        .require(DeadeyeLock)
                        .runs(() -> {
                            deadeye.update();
                        })
                        .build()
                );
                if (gamepad.triangle.Pressed()) {
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