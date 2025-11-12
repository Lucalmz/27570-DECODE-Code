package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.library.CustomOpMode;

@Autonomous
public class Auto12 extends CustomOpMode {
    private static final Pose startPoint = new Pose(48,134.5,-90);
    private static final Pose lookSequencePoint = new Pose(48,125,-120);
    private static final Pose ShootFirstSetPoint = new Pose(59,85);
    private static final Pose GetMidSetPoint = new Pose(43,60,180);
    private static final Pose FirstShootToMidControlPoint = new Pose(59,60);
    private static final Pose FinishMidSetPoint = new Pose(23,60,180);
    private static final Pose PushGatePoint = new Pose(17,70);
    private static final Pose MidToGateControlPoint = new Pose(22,70);
    private static final Pose ShootSecondSetPoint = new Pose(59,85);
    private static final Pose GateToSecondShootControlPoint = new Pose(40,65);
    private static final Pose GateToSecondShootSecondControlPoint = new Pose(59,70);
    private static final Pose FinishSecondShootPoint = new Pose(47,96);
    private static final Pose SecondShootControlPoint = new Pose(59,96);
    private static final Pose ToGetTopSetPoint = new Pose(40,85,180);
    private static final Pose ToGetTopSetControlPoint = new Pose(59,85);
    private static final Pose GetTopSetPoint = new Pose(23,85,180);
    private static final Pose ToShootTopSetPoint = new Pose(34,109);
    private static final Pose ToShootTopSetControlPoint = new Pose(23,109);
    private static final Pose FinishShootTopSetPoint = new Pose(68,75.5);
    private static final Pose ShootTopSetControlPoint = new Pose(68,109);
    private static final Pose ToGetBottomSetPoint = new Pose(40,85,180);



    @Override
    public void init(){
        super.init();
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
    }
    @Override
    public void stop(){
        super.stop();
    }
}
