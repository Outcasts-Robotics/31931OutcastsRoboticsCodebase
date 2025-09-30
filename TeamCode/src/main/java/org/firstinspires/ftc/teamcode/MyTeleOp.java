package org.firstinspires.ftc.teamcode;


import static androidx.core.math.MathUtils.clamp;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utils.AutoAim;
import org.firstinspires.ftc.teamcode.utils.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.WebcamProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@TeleOp(name = "MyTeleOp", group = "TeleOp")
public class MyTeleOp extends OpMode {
    private Follower follower;
    private MecanumDrive mecanumDrive;
    private WebcamName webcam1;
    private WebcamProcessor webcamProcessor;
    private AutoAim autoAim;


    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        )));

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcamProcessor = new WebcamProcessor(webcam1, telemetry, Constants.webcamProcessorInputs);
        webcamProcessor.initialize();

        mecanumDrive = new MecanumDrive(hardwareMap, Constants.mecanumConstants, "imu", telemetry, 1);

//        follower = Constants.createFollower(hardwareMap);

        autoAim = new AutoAim(imu);
    }

    @Override
    public void init_loop() {
        if (webcamProcessor != null) webcamProcessor.initLoopUpdate(gamepad1);
    }

    @Override
    public void start() {
        telemetry.addData("Status", "Starting");
    }

    @Override
    public void loop() {
        if (webcamProcessor != null) webcamProcessor.loopUpdate();

        boolean autoAimEnabled = false;
        if (gamepad1.left_trigger > 0.2 && webcamProcessor != null) {
            autoAimEnabled = true;
            AprilTagDetection detection = webcamProcessor.getLastDetection();
            autoAim.loopUpdate(detection == null ? null : detection.ftcPose);
            double yawPid = autoAim.getYawPid();
            telemetry.addData("yawPid", yawPid);
            mecanumDrive.loopUpdate(gamepad1, yawPid);
        } else {
            // manual drive
            if (follower != null) {  // with pedro pathing
                float speedScale = 0.2f + (1 - gamepad1.right_trigger) * 0.8f;
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * speedScale,
                        gamepad1.left_stick_x * speedScale,
                        gamepad1.right_stick_x * speedScale,
                        gamepad1.left_bumper);
            } else if (mecanumDrive != null) {  // with manual driving
                mecanumDrive.loopUpdate(gamepad1, 0);
            }
        }

        telemetry.addData("autoAim", autoAimEnabled);
    }

    @Override
    public void stop() {
        telemetry.addData("Status", "Stopping");
    }
}
