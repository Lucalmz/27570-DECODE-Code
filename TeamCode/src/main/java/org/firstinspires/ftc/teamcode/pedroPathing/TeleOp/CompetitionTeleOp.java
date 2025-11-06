package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.library.AlgorithmLib.Aim;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.AlgorithmLib.CalculateNewLaunch;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.AlgorithmLib.ShootAll;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.ObjectLib.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.StatesLib.*;

import static com.bear27570.yuan.BotFactory.Model.Action.*;

import com.bear27570.yuan.BotFactory.Model.ConflictPolicy;
import com.bear27570.yuan.BotFactory.Model.Priority;
import com.bear27570.yuan.BotFactory.ThreadManagement.Task;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.pedroPathing.Models.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Models.Target;
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
        LLResultTypes.DetectorResult targetBall = DeadeyeAPI.getTargetClosestToAnchor();
        if(targetBall!=null) {
            coords = DeadeyeAPI.calculateCoordinates(targetBall);
        }
        if (coords != null) {
            telemetry.addData("Target Coords", "x: %.1f cm, y: %.1f cm",
                    coords[0], coords[1]);
        }

        if (currentPose == null) {
            Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.IGNORE)
                    .require(gamepadRumbleLock)
                    .runs(() -> {
                        gamepad.rumble(0.2, 0.2, 1);
                        currentPose = localizer.getRobotPose();
                    })
                    .build());
            flag=true;
        }
        if(flag&&currentPose!=null){
            localizer.close();
            flag=false;
        }
        if (gamepad.right_bumper.justReleased()) {
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
        }
        //射击模式
        if (gamepad.right_bumper.Pressed()) {
            Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                    .require(LeftBoard)
                    .require(RightBoard)
                    .runs(() -> {
                        LeftBoard.act(Shoot);
                        RightBoard.act(Shoot);
                    })
                    .build());
        }
        if(gamepad.circle.Pressed()){
            Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.IGNORE)
                    .require(LeftBoard)
                            .require(RightBoard)
                            .require(Inhale)
                    .runs(ShootAll())
                    .build());
        }
        if (gamepad.right_bumper.isPressing()) {
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
            return;
        }
            Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.IGNORE)
                    .require(IntakeMotor)
                    .runs(() -> {
                        IntakeMotor.act(PullIn);
                        ClassifyServo.actWithVel(40);
                    })
                    .build());

    }

    @Override
    public void stop() {
        super.stop();
        DeadeyeAPI.stop();
    }
}