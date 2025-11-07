package org.firstinspires.ftc.teamcode.tunings;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.teamcode.components.WebcamProcessor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@TeleOp
public class PinpointAprilTagTuner extends OpMode {
    private PinpointLocalizer pinpointLocalizer;
    private WebcamProcessor webcamProcessor;

    private boolean initPose = false;
    private AprilTagDetection lastDetection;

    @Override
    public void init() {
        pinpointLocalizer = new PinpointLocalizer(hardwareMap, Constants.pinpointConstants);
        pinpointLocalizer.resetIMU();
        webcamProcessor = new WebcamProcessor(hardwareMap, telemetry, Constants.webcamProcessorInputs);
        webcamProcessor.initialize();
    }

    @Override
    public void init_loop() {
        webcamProcessor.update();
        AprilTagDetection detection = webcamProcessor.getDetection();
        if (!initPose && detection != null) {
            telemetry.addLine("Detected " + detection.id);
            pinpointLocalizer.setPose(Utils.toPedro(
                    new Pose(detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS),
                            FTCCoordinates.INSTANCE)));
            initPose = true;
        }
        if (detection != null)
        {
            lastDetection = detection;
        }
        Pose pose = pinpointLocalizer.getPose();
        telemetry.addData("pinpointPose", String.format("X:%6.1f Y:%6.1f H:%6.1f %s",
                pose.getX(), pose.getY(), pose.getHeading(),
                pose.getCoordinateSystem().getClass().getSimpleName()));
    }

    @Override
    public void loop() {
        webcamProcessor.update();

        AprilTagDetection detection = webcamProcessor.getDetection();
        if (detection != null) {
            pinpointLocalizer.setPose(Utils.toPedro(detection.robotPose));
            lastDetection = webcamProcessor.getDetection();
        }

        if (lastDetection != null) {
            AprilTagPoseFtc camPose = lastDetection.ftcPose;
            telemetry.addData("cam_xyz", String.format("X:%.1f Y:%.1f Z:%.1f", camPose.x, camPose.y, camPose.z));
            telemetry.addData("cam_bre", String.format("B:%.1f R:%.1f E:%.1f", camPose.bearing, camPose.range, camPose.elevation));
            telemetry.addData("cam_rpy", String.format("R:%.1f P:%.1f Y:%.1f", camPose.roll, camPose.pitch, camPose.yaw));
        }

        pinpointLocalizer.update();

        Pose pose = pinpointLocalizer.getPose();
        telemetry.addData("pinpointPose", String.format("X:%6.1f Y:%6.1f H:%6.1f %s",
                pose.getX(), pose.getY(), pose.getHeading(),
                pose.getCoordinateSystem().getClass().getSimpleName()));
    }
}
