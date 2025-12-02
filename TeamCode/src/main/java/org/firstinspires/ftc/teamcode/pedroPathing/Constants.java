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

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(.5)
            .rightFrontMotorName("fr")
            .rightRearMotorName("rr")
            .leftRearMotorName("rl")
            .leftFrontMotorName("fl")
            .xVelocity(40.97963252030002)
            .yVelocity(34.48916337621495)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .hardwareMapName("pinpoint")
            .forwardPodY(-2)
            .strafePodX(-6.5)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .distanceUnit(DistanceUnit.INCH);

    public static PathConstraints pathConstraints = new PathConstraints(.995, 100, .6, .9);
    static FollowerConstants followerConstants = new FollowerConstants()
            .mass(7.5)
            .forwardZeroPowerAcceleration(-39.12102161911619)
            .lateralZeroPowerAcceleration(-50.15222548957224)
            .translationalPIDFCoefficients(new PIDFCoefficients(.2,.06,.01, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1,.01, 0, .01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(.006, 0 ,0.00001, .6, 0.01))
            .centripetalScaling(.003);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }


}
