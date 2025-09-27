package org.firstinspires.ftc.teamcode;


import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.vision.WebcamProcessor;


@TeleOp(name = "MyTeleOp", group = "TeleOp")
public class MyTeleOp extends OpMode {
    private Follower follower;
    private WebcamName webcam1;
    private WebcamProcessor webcamProcessor;


    @Override
    public void init() {
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcamProcessor = new WebcamProcessor(webcam1, telemetry, Constants.webcamProcessorInputs);
        webcamProcessor.initialize();

        follower = Constants.createFollower(hardwareMap);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        webcamProcessor.initLoopUpdate(gamepad1);
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        webcamProcessor.loopUpdate();

        if (gamepad1.left_trigger > 0.2) {
            // auto drive takes over
        } else {
            // manual drive
            float speedScale = 0.2f + (1 - gamepad1.right_trigger) * 0.8f;
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * speedScale,
                    gamepad1.left_stick_x * speedScale,
                    gamepad1.right_stick_x * speedScale,
                    gamepad1.left_bumper);
        }
//        follower.setPose(new Pose(robotPose.getPosition().x, robotPose.getPosition().y, robotPose.getOrientation().getYaw()));
    }

    @Override
    public void stop() {
        telemetry.addData("Status", "Stopped");
    }

}
