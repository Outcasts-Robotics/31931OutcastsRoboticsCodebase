package org.firstinspires.ftc.teamcode;


import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utils.AutoAim;
import org.firstinspires.ftc.teamcode.utils.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.WebcamProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import java.util.function.DoubleSupplier;


@TeleOp(name = "MyTeleOp", group = "TeleOp")
public class MyTeleOp extends OpMode {
    private Follower follower;
    private MecanumDrive mecanumDrive;
    private WebcamName webcam1;
    private WebcamProcessor webcamProcessor;
    private AutoAim autoAim;
    private PinpointLocalizer pinpointLocalizer;
    private Flywheel flywheel;




    @Override
    public void init() {

        telemetry.addData("Status", "Initializing");


        pinpointLocalizer = new PinpointLocalizer(hardwareMap, Constants.pinpointConstants);
        pinpointLocalizer.resetIMU();
        flywheel = new Flywheel(hardwareMap);
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcamProcessor = new WebcamProcessor(webcam1, telemetry, Constants.webcamProcessorInputs);
        webcamProcessor.initialize();

        DoubleSupplier yawInRadProvider = () -> pinpointLocalizer.getPose().getHeading();
        mecanumDrive = new MecanumDrive(hardwareMap, Constants.mecanumConstants, telemetry, 1.0, yawInRadProvider);

//        follower = Constants.createFollower(hardwareMap);

        DoubleSupplier yawInDegProvider = () -> radToDeg(pinpointLocalizer.getPose().getHeading());
        autoAim = new AutoAim(yawInDegProvider);
    }

    private double radToDeg(double radians) {
        return AngleUnit.DEGREES.fromRadians(radians);
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

        pinpointLocalizer.update(); // update pose

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
