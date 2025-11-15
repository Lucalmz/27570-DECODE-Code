package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import static org.firstinspires.ftc.teamcode.pedroPathing.library.ObjectLib.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.StatesLib.*;

import com.bear27570.yuan.BotFactory.Model.Action;
import com.bear27570.yuan.BotFactory.Model.ConflictPolicy;
import com.bear27570.yuan.BotFactory.Model.Priority;
import com.bear27570.yuan.BotFactory.ThreadManagement.Task;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Models.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Models.Target;
import org.firstinspires.ftc.teamcode.pedroPathing.Services.FlipPose;
import org.firstinspires.ftc.teamcode.pedroPathing.library.AlgorithmLib;
import org.firstinspires.ftc.teamcode.pedroPathing.library.CustomOpMode;

@Autonomous
public class Auto12 extends CustomOpMode {
    private static Pose startPoint = new Pose(48, 135, Math.toRadians(90));
    private static Pose lookSequencePoint = new Pose(56, 120, Math.toRadians(60));
    private static Pose shootingPoint = new Pose(54.5, 87, Math.toRadians(130));
    private static Pose secondBallLineEnter = new Pose(36, 59, Math.toRadians(0));
    private static Pose toSecondBallLineControlPoint = new Pose(53.5, 59, Math.toRadians(0));
    private static Pose secondBallLineIT = new Pose(20, 59, Math.toRadians(0));
    private static Pose gateOpen = new Pose(17, 68, Math.toRadians(0));
    private static Pose gateOpenControlPoint = new Pose(20, 68, Math.toRadians(0));
    private static Pose gateOpenToShootControlPoint = new Pose(54, 62, Math.toRadians(0));
    private static Pose thirdBallLineEnter = new Pose(41.5, 35, Math.toRadians(0));
    private static Pose toThirdBallLineControlPoint = new Pose(53.5, 35, Math.toRadians(0));
    private static Pose thirdBallLineIT = new Pose(20, 35, Math.toRadians(0));
    private static Pose firstBallLineEnter = new Pose(41, 84, Math.toRadians(0));
    private static Pose toFirstBallLineControlPoint = new Pose(53.5, 84, Math.toRadians(0));
    private static Pose firstBallLineIT = new Pose(23, 84, Math.toRadians(0));
    private static Pose endPoint = new Pose(30, 90, Math.toRadians(90));
    private static int pathState;
    private PathChain shootFirstSet, intakeSecLineAndPushGate, shootSecondSet, getTopLineAndShoot, getBottomLineAndShoot, park;

    public void buildPaths() {
        shootFirstSet = follower.pathBuilder()
                .addPath(new BezierLine(startPoint, lookSequencePoint))
                .setLinearHeadingInterpolation(startPoint.getHeading(), lookSequencePoint.getHeading())
                .addPath(new BezierLine(lookSequencePoint, shootingPoint))
                .setLinearHeadingInterpolation(lookSequencePoint.getHeading(), shootingPoint.getHeading())
                .build();
        intakeSecLineAndPushGate = follower.pathBuilder()
                .addPath(new BezierCurve(shootingPoint, toSecondBallLineControlPoint, secondBallLineEnter))
                .setLinearHeadingInterpolation(shootingPoint.getHeading(), secondBallLineEnter.getHeading())
                .addPath(new BezierLine(secondBallLineEnter, secondBallLineIT))
                .setConstantHeadingInterpolation(secondBallLineIT.getHeading())
                .addParametricCallback(0.1, () -> follower.setMaxPower(0.5))
                .addParametricCallback(0.9, () -> follower.setMaxPower(1))
                .addPath(new BezierCurve(secondBallLineIT, gateOpenControlPoint, gateOpen))
                .setLinearHeadingInterpolation(secondBallLineIT.getHeading(), gateOpen.getHeading())
                .build();
        shootSecondSet = follower.pathBuilder()
                .addPath(new BezierCurve(gateOpen, gateOpenToShootControlPoint, shootingPoint))
                .setLinearHeadingInterpolation(gateOpen.getHeading(), shootingPoint.getHeading())
                .build();
        getTopLineAndShoot = follower.pathBuilder()
                .addPath(new BezierCurve(shootingPoint, toFirstBallLineControlPoint, firstBallLineEnter))
                .setLinearHeadingInterpolation(0, firstBallLineEnter.getHeading())
                .addPath(new BezierLine(firstBallLineEnter, firstBallLineIT))
                .setConstantHeadingInterpolation(firstBallLineIT.getHeading())
                .addParametricCallback(0.1, () -> follower.setMaxPower(0.75))
                .addParametricCallback(0.9, () -> follower.setMaxPower(1))
                .addPath(new BezierLine(firstBallLineIT, shootingPoint))
                .setLinearHeadingInterpolation(firstBallLineIT.getHeading(), shootingPoint.getHeading())
                .build();
        getBottomLineAndShoot = follower.pathBuilder()
                .addPath(new BezierCurve(shootingPoint, toThirdBallLineControlPoint, thirdBallLineEnter))
                .setLinearHeadingInterpolation(shootingPoint.getHeading(), thirdBallLineEnter.getHeading())
                .addPath(new BezierLine(thirdBallLineEnter, thirdBallLineIT))
                .setConstantHeadingInterpolation(thirdBallLineIT.getHeading())
                .addParametricCallback(0.1, () -> follower.setMaxPower(0.75))
                .addParametricCallback(0.9, () -> follower.setMaxPower(1))
                .addPath(new BezierLine(thirdBallLineIT, shootingPoint))
                .setLinearHeadingInterpolation(thirdBallLineIT.getHeading(), shootingPoint.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(shootingPoint, endPoint))
                .setLinearHeadingInterpolation(shootingPoint.getHeading(), endPoint.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(shootFirstSet);
                    follower.update();
                    pathState = 1;
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    AlgorithmLib.ShootWithSequence();
                    AlgorithmLib.IntakeStructInAuto();
                    follower.followPath(intakeSecLineAndPushGate);
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    AlgorithmLib.ShootingArmed();
                    follower.followPath(shootSecondSet);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    AlgorithmLib.ShootWithSequence();
                    AlgorithmLib.IntakeStructInAuto();
                    follower.turnToDegrees(0);
                    pathState = 4;
                }
                break;
            case 4:
                if (!(Math.abs(follower.getHeading()) >= Math.toRadians(10))) {
                    follower.followPath(getTopLineAndShoot);
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    AlgorithmLib.ShootingArmed();
                    AlgorithmLib.ShootWithSequence();
                    AlgorithmLib.IntakeStructInAuto();
                    follower.followPath(getBottomLineAndShoot);
                    pathState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    AlgorithmLib.ShootingArmed();
                    AlgorithmLib.ShootWithSequence();
                    AlgorithmLib.IntakeStructInAuto();
                    follower.followPath(park);
                    pathState = -1;
                }
                break;
        }
    }

    @Override
    public void init() {
        target = Target.AUTONOMOUS;
        pathState = 0;
        super.init();
        telemetry.addData("Current Alliance:", alliance.name());
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        if (alliance == Alliance.Red) {
            startPoint = FlipPose.flipPose(startPoint);
            lookSequencePoint = FlipPose.flipPose(lookSequencePoint);
            shootingPoint = FlipPose.flipPose(shootingPoint);
            secondBallLineEnter = FlipPose.flipPose(secondBallLineEnter);
            toSecondBallLineControlPoint = FlipPose.flipPose(toSecondBallLineControlPoint);
            secondBallLineIT = FlipPose.flipPose(secondBallLineIT);
            gateOpen = FlipPose.flipPose(gateOpen);
            gateOpenControlPoint = FlipPose.flipPose(gateOpenControlPoint);
            gateOpenToShootControlPoint = FlipPose.flipPose(gateOpenToShootControlPoint);
            thirdBallLineEnter = FlipPose.flipPose(thirdBallLineEnter);
            toThirdBallLineControlPoint = FlipPose.flipPose(toThirdBallLineControlPoint);
            thirdBallLineIT = FlipPose.flipPose(thirdBallLineIT);
            firstBallLineEnter = FlipPose.flipPose(firstBallLineEnter);
            toFirstBallLineControlPoint = FlipPose.flipPose(toFirstBallLineControlPoint);
            firstBallLineIT = FlipPose.flipPose(firstBallLineIT);
            endPoint = FlipPose.flipPose(endPoint);
        }
        follower.setStartingPose(startPoint);
        follower.update();
        buildPaths();
        Shooter.VelocityAct(Action.AutoShootPose);
        Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                .runs(AlgorithmLib::findSequence)
                .build());
    }

    @Override
    public void loop() {
        super.loop();
        autonomousPathUpdate();
        follower.update();
        telemetry.addData("Follower Pose", follower.getPose().toString());
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
