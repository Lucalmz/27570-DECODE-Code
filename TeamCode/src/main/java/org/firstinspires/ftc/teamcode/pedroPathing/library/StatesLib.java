package org.firstinspires.ftc.teamcode.pedroPathing.library;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Models.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Models.BallsInQueue;
import org.firstinspires.ftc.teamcode.pedroPathing.Models.Mode;
import org.firstinspires.ftc.teamcode.pedroPathing.Models.Target;
import org.firstinspires.ftc.teamcode.vision.QuickScope.LaunchSolution;

import java.util.concurrent.atomic.AtomicReference;

public class StatesLib {
    public static Target target = Target.TELEOP;
    public static Alliance alliance = Alliance.Red;
    public static BallsInQueue balls = BallsInQueue.getInstance();
    public static volatile AtomicReference<LaunchSolution> latestSolution = new AtomicReference<>();
    public static Mode mode;
    public static int ManualShootRatio;
    public static volatile int SequenceID = 3;
    public static volatile Pose2D currentPose;
    public static double HeadingForFieldInAlliance;
}
