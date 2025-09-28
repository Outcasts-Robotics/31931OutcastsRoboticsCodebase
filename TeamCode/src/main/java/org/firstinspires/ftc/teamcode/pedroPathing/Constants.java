package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.vision.WebcamProcessor;

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

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .twoWheelLocalizer(localizerConstants)
                .mecanumDrivetrain(mecanumConstants)
                .build();
    }

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("forward_encoder")
            .strafeEncoder_HardwareMapName("strafe_encoder")
            .forwardPodY(-8) // tuning - inches from center of rotation
            .strafePodX(-8)  // tuning - inches from center of rotation
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            ))
            .forwardTicksToInches(.001989436789)  // tuning - encoder
            .strafeTicksToInches(.001989436789)  // tuning - encoder
            ;

    public static WebcamProcessor.Inputs webcamProcessorInputs = new WebcamProcessor.Inputs() {
        {
            cameraPosition = new Position(DistanceUnit.INCH, 3, 3, 3, 0);
            cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -5, 0, 0);
            detectionMaxAgeMs = 300;
            telemetryDetails = true;
        }
    };
}
