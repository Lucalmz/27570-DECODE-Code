package org.firstinspires.ftc.teamcode.pedroPathing.Services;

import com.pedropathing.geometry.Pose;

public class FlipPose {
    public static Pose flipPose(Pose pose) {
        double newX = -pose.getX();
        double newY = pose.getY();
        double newHeading = Math.PI-pose.getHeading();
        return new Pose(newX, newY, newHeading);
    }
}
