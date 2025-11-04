package org.firstinspires.ftc.teamcode.tunings;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.FuturePose;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.G;
import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.teamcode.components.AutoAim;
import org.firstinspires.ftc.teamcode.components.MecanumDrive;
import org.firstinspires.ftc.teamcode.components.WebcamProcessor;
import org.firstinspires.ftc.teamcode.helpers.PIDController;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.function.DoubleSupplier;

@TeleOp(group = "Tunings")
public class AutoAimPidTuner extends OpMode {
    private MecanumDrive mecanumDrive;
    private WebcamProcessor webcamProcessor;
    private AutoAim autoAim;
    private PinpointLocalizer pinpointLocalizer;

    private Pose lastTagPose;

    private int stage = 1; // 1 = kp, 2 = ki, 3 = kd
    private Pose3D lastRobotPose;

    @Override
    public void init() {
        pinpointLocalizer = new PinpointLocalizer(hardwareMap, Constants.pinpointConstants);
        pinpointLocalizer.resetIMU();

        webcamProcessor = new WebcamProcessor(hardwareMap, telemetry, Constants.webcamProcessorInputs);
        webcamProcessor.initialize();

        DoubleSupplier pinpointHeading = () -> pinpointLocalizer.getPose().getHeading();
        mecanumDrive = new MecanumDrive(hardwareMap, Constants.mecanumConstants, telemetry, 1.0, pinpointHeading);

        FuturePose futurePose = () -> pinpointLocalizer.getPose();
        autoAim = new AutoAim(futurePose, telemetry);
        autoAim.setTarget(G.pois.goal);
    }

    @Override
    public void loop() {
        webcamProcessor.update();

        AprilTagDetection tag = webcamProcessor.getDetection();
        if (tag != null) {
            lastRobotPose = tag.robotPose;
            lastTagPose = Utils.toPedro(tag.robotPose);
            pinpointLocalizer.setPose(lastTagPose);
        }
        pinpointLocalizer.update();

        if (lastTagPose != null) {
            telemetry.addData("lastTagPose", String.format("x=%.1f y=%.1f h=%.1f %s",
                    lastTagPose.getX(), lastTagPose.getY(),
                    AngleUnit.DEGREES.fromRadians(lastTagPose.getHeading()),
                    lastTagPose.getCoordinateSystem().getClass().getSimpleName()));
            telemetry.addData("lastRobotPose", String.format("x=%.1f y=%.1f h=%.1f",
                    lastRobotPose.getPosition().x, lastRobotPose.getPosition().y,
                    lastRobotPose.getOrientation().getYaw()));
        }

        if (gamepad1.left_trigger > 0.3 && !autoAim.isActive()) {
            autoAim.setActive(true);
        }
        if (gamepad1.left_trigger < 0.3 && autoAim.isActive()) {
            autoAim.setActive(false);
        }

        PIDController pid = autoAim.getHeadingPidController();
        switch (stage) {
            case 1:
                if (gamepad1.aWasPressed()) {
                    pid.setKp(pid.getKp() - 0.01);
                } else if (gamepad1.bWasPressed()) {
                    pid.setKp(pid.getKp() + 0.01);
                }
                break;
            case 2:
                if (gamepad1.aWasPressed()) {
                    pid.setKi(pid.getKi() - 0.01);
                } else if (gamepad1.bWasPressed()) {
                    pid.setKi(pid.getKi() + 0.01);
                }
                break;
            case 3:
                if (gamepad1.aWasPressed()) {
                    pid.setKd(pid.getKd() - 0.01);
                } else if (gamepad1.bWasPressed()) {
                    pid.setKd(pid.getKd() + 0.01);
                }
                break;
        }
        if (gamepad1.xWasPressed()) {
            stage = (stage + 1) % 4;
            if (stage == 0) stage = 1;
        }
        telemetry.addData("Stage", stage);
        telemetry.addData("Kp", pid.getKp());
        telemetry.addData("Ki", pid.getKi());
        telemetry.addData("Kd", pid.getKd());

        autoAim.update();
        double headingPid = autoAim.getHeadingPid();
        telemetry.addData("headingPid", headingPid);
        mecanumDrive.update(gamepad1, headingPid);
    }
}
