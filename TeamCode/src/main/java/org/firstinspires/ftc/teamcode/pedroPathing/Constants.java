package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13)
            .forwardZeroPowerAcceleration(-67.06408661644541)
            .lateralZeroPowerAcceleration(-81.65783673400453)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.22,0.001,0.01,0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(2.5,0.1,0.01,0.05))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01,0.001,0.0001,0.6,0.01))
            .centripetalScaling(0.005);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("RightFrontMotor")
            .rightRearMotorName("RightBehindMotor")
            .leftRearMotorName("LeftBehindMotor")
            .leftFrontMotorName("LeftFrontMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(67.64028474101872)
            .yVelocity(34.70118004506028);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-180)
            .strafePodX(85)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.8, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
