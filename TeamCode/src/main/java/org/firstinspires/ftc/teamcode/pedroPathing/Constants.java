package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.components.WebcamProcessor;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(6) // tuning - mass in kg
            .forwardZeroPowerAcceleration(-34.62719) // tuning - forward deceleration
            .lateralZeroPowerAcceleration(-78.15554) // tuning - lateral deceleration
            ;
    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .maxPower(0.8)
            .leftFrontMotorName("lf")
            .rightFrontMotorName("rf")
            .leftRearMotorName("lr")
            .rightRearMotorName("rr")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(81) // tuning - forward max velocity
            .yVelocity(65) // tuning - sideway max velocity
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .hardwareMapName("pinpoint")
            .forwardPodY(-2)
            .strafePodX(-5)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(pinpointConstants)
                .mecanumDrivetrain(mecanumConstants)
                .build();
    }

    public static WebcamProcessor.Inputs webcamProcessorInputs = new WebcamProcessor.Inputs() {
        {
            cameraPosition = new Position(DistanceUnit.INCH, 3, 3, 3, 0);
            cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -5, 0, 0);
            detectionMaxAgeMs = 300;
            telemetryDetails = true;
            detectAllTags = true;
        }
    };
}
