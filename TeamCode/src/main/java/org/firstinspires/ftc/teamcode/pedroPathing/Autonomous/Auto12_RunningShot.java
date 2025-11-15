package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import static org.firstinspires.ftc.teamcode.pedroPathing.library.ObjectLib.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.StatesLib.*;

import com.bear27570.yuan.BotFactory.Model.ConflictPolicy;
import com.bear27570.yuan.BotFactory.Model.Priority;
import com.bear27570.yuan.BotFactory.ThreadManagement.Task;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Models.Target;
import org.firstinspires.ftc.teamcode.pedroPathing.library.AlgorithmLib;
import org.firstinspires.ftc.teamcode.pedroPathing.library.CustomOpMode;

@Autonomous
public class Auto12_RunningShot extends CustomOpMode {
    private static final Pose startPoint = new Pose(48,134.5,Math.toRadians(90));
    private static final Pose lookSequencePoint = new Pose(48,125,Math.toRadians(60));
    private static final Pose ShootFirstSetPoint = new Pose(59,85,Math.toRadians(135));
    private static final Pose GetMidSetPoint = new Pose(43,60,0);
    private static final Pose FirstShootToMidControlPoint = new Pose(59,60);
    private static final Pose FinishMidSetPoint = new Pose(23,60,0);
    private static final Pose PushGatePoint = new Pose(17,70,0);
    private static final Pose MidToGateControlPoint = new Pose(22,70);
    private static final Pose ShootSecondSetPoint = new Pose(59,85,Math.toRadians(135));
    private static final Pose GateToSecondShootControlPoint = new Pose(40,65);
    private static final Pose GateToSecondShootSecondControlPoint = new Pose(59,70);
    private static final Pose FinishSecondShootPoint = new Pose(47,96,Math.toRadians(135));
    private static final Pose SecondShootControlPoint = new Pose(59,96);
    private static final Pose ToGetTopSetPoint = new Pose(40,85,0);
    private static final Pose ToGetTopSetControlPoint = new Pose(59,85);
    private static final Pose GetTopSetPoint = new Pose(23,85,0);
    private static final Pose ToShootTopSetPoint = new Pose(34,109,Math.toRadians(135));
    private static final Pose ToShootTopSetControlPoint = new Pose(23,109);
    private static final Pose FinishShootTopSetPoint = new Pose(68,75.5,Math.toRadians(135));
    private static final Pose ShootTopSetControlPoint = new Pose(68,109);
    private static final Pose ToGetBottomSetPoint = new Pose(40,35,0);
    private static final Pose ToGetBottomSetControlPoint = new Pose(68,35);
    private static final Pose GetBottomSetPoint = new Pose(23,35,0);
    private static final Pose ShootLastSetPoint = new Pose(64,80,Math.toRadians(135));
    private static final Pose EndPoint = new Pose(30,90,Math.toRadians(90));
    private static boolean isSequenceFound;
    private static int pathState;
    private PathChain shootFirstSet,getMidSetAndPushGate,shootSecondSet, getTopSetAndShootAndGetBottomSet,shootLastSet,park;
    public void buildPaths(){
        shootFirstSet = follower.pathBuilder()
                .addPath(new BezierLine(startPoint,lookSequencePoint))
                .setLinearHeadingInterpolation(startPoint.getHeading(),lookSequencePoint.getHeading())
                .addPath(new BezierLine(lookSequencePoint,ShootFirstSetPoint))
                .setHeadingInterpolation(VisionCalculatedHeading)
                .build();
        getMidSetAndPushGate = follower.pathBuilder()
                .addPath(new BezierCurve(ShootFirstSetPoint,FirstShootToMidControlPoint,GetMidSetPoint))
                .setLinearHeadingInterpolation(ShootFirstSetPoint.getHeading(),GetMidSetPoint.getHeading())
                .addPath(new BezierLine(GetMidSetPoint,FinishMidSetPoint))
                .setConstantHeadingInterpolation(FinishMidSetPoint.getHeading())
                .addPath(new BezierCurve(FinishMidSetPoint,MidToGateControlPoint,PushGatePoint))
                .setLinearHeadingInterpolation(FinishMidSetPoint.getHeading(),PushGatePoint.getHeading())
                .build();
        shootSecondSet = follower.pathBuilder()
                .addPath(new BezierCurve(PushGatePoint,GateToSecondShootControlPoint,GateToSecondShootSecondControlPoint,ShootSecondSetPoint))
                .setLinearHeadingInterpolation(PushGatePoint.getHeading(),ShootSecondSetPoint.getHeading())
                .addPath(new BezierCurve(ShootSecondSetPoint,SecondShootControlPoint,FinishSecondShootPoint))
                .setHeadingInterpolation(VisionCalculatedHeading)
                .build();
        getTopSetAndShootAndGetBottomSet = follower.pathBuilder()
                .addPath(new BezierCurve(FinishSecondShootPoint,ToGetTopSetControlPoint,ToGetTopSetPoint))
                .setLinearHeadingInterpolation(FinishSecondShootPoint.getHeading(),ToGetTopSetPoint.getHeading())
                .addPath(new BezierLine(ToGetTopSetPoint,GetTopSetPoint))
                .setConstantHeadingInterpolation(GetTopSetPoint.getHeading())
                .addPath(new BezierCurve(GetTopSetPoint,ToShootTopSetControlPoint,ToShootTopSetPoint))
                .setLinearHeadingInterpolation(GetTopSetPoint.getHeading(),ToShootTopSetPoint.getHeading())
                .addPath(new BezierCurve(ToShootTopSetPoint,ShootTopSetControlPoint,FinishShootTopSetPoint))
                .setHeadingInterpolation(VisionCalculatedHeading)
                .addPath(new BezierCurve(FinishShootTopSetPoint,ToGetBottomSetControlPoint,ToGetBottomSetPoint))
                .setLinearHeadingInterpolation(FinishShootTopSetPoint.getHeading(),ToGetBottomSetPoint.getHeading())
                .addPath(new BezierLine(ToGetBottomSetPoint,GetBottomSetPoint))
                .setConstantHeadingInterpolation(GetBottomSetPoint.getHeading())
                .build();
        shootLastSet = follower.pathBuilder()
                .addPath(new BezierLine(GetBottomSetPoint,ShootLastSetPoint))
                .setLinearHeadingInterpolation(GetBottomSetPoint.getHeading(),ShootLastSetPoint.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(ShootLastSetPoint,EndPoint))
                .setLinearHeadingInterpolation(ShootLastSetPoint.getHeading(),EndPoint.getHeading())
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(shootFirstSet);

                break;
        }
    }
    @Override
    public void init(){
        target = Target.AUTONOMOUS;
        super.init();
        buildPaths();
        follower.setStartingPose(startPoint);

    }
    @Override
    public void init_loop(){
        super.init_loop();
    }
    @Override
    public void start(){
        super.start();
    }
    @Override
    public void loop(){
        super.loop();
        Manager.submit(new Task.TaskBuilder(Priority.LOW, ConflictPolicy.QUEUE)
                .require(calculatorLock)
                .runs(AlgorithmLib::getNewLaunch)
                .build());
    }
    @Override
    public void stop(){
        super.stop();
    }
}