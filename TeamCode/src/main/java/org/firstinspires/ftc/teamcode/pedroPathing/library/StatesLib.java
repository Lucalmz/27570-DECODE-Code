package org.firstinspires.ftc.teamcode.pedroPathing.library;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Models.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Models.BallsInQueue;
import org.firstinspires.ftc.teamcode.pedroPathing.Models.Target;
import org.firstinspires.ftc.teamcode.vision.QuickScope.LaunchSolution;

public class StatesLib {
    public static Target target = Target.TELEOP;
    public static Alliance alliance = Alliance.Red;
    public static BallsInQueue balls = BallsInQueue.getInstance();
    public static LaunchSolution latestSolution;
    public static Pose2D currentPose;
}
