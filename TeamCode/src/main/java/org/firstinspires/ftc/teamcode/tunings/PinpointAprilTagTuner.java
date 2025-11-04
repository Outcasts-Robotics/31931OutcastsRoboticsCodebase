package org.firstinspires.ftc.teamcode.tunings;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.teamcode.components.WebcamProcessor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class PinpointAprilTagTuner extends OpMode {
    private PinpointLocalizer pinpointLocalizer;
    private WebcamProcessor webcamProcessor;

    private boolean initPose = false;

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
            pinpointLocalizer.setPose(new Pose(
                    detection.robotPose.getPosition().x,
                    detection.robotPose.getPosition().y,
                    detection.robotPose.getOrientation().getYaw(),
                    FTCCoordinates.INSTANCE));
            initPose = true;
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
        }

        pinpointLocalizer.update();

        Pose pose = pinpointLocalizer.getPose();
        telemetry.addData("pinpointPose", String.format("X:%6.1f Y:%6.1f H:%6.1f %s",
                pose.getX(), pose.getY(), pose.getHeading(),
                pose.getCoordinateSystem().getClass().getSimpleName()));
    }
}
