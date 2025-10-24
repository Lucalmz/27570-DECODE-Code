package org.firstinspires.ftc.teamcode.pedroPathing.Services;

import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.pedroPathing.library.ObjectLib;

public class RobotMovementAPI {
    public static Vector getRobotVelocity(){
        return  ObjectLib.follower.getVelocity();
    }
    private RobotMovementAPI(){}
}
