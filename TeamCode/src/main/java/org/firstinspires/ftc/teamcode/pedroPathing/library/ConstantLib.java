package org.firstinspires.ftc.teamcode.pedroPathing.library;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
@Configurable
public class ConstantLib {
    public static final Pose2D BLUE_ALLIANCE_TARGET = new Pose2D(DistanceUnit.MM, 3657.6, 927.1, AngleUnit.DEGREES, 0);
    public static final Pose2D RED_ALLIANCE_TARGET = new Pose2D(DistanceUnit.MM, 3657.6, 2730.5, AngleUnit.DEGREES, 0);
    public static final double ASSISTANT_STRENGTH = 0.2;
    public static final int SHOOT_ONE_BALL_DIST = 200;
    public static final int SHOOT_ONE_BALL_TIME = 200;

    private ConstantLib(){}
}
