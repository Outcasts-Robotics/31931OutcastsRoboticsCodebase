package org.firstinspires.ftc.teamcode.tunings;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.components.AutoAim;
import org.firstinspires.ftc.teamcode.components.MecanumDrive;
import org.firstinspires.ftc.teamcode.helpers.PIDController;
import org.firstinspires.ftc.teamcode.components.WebcamProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.function.DoubleSupplier;

@TeleOp(group = "Tunings")
public class AutoAimPidTuner extends OpMode {
    private MecanumDrive mecanumDrive;
    private WebcamName webcam1;
    private WebcamProcessor webcamProcessor;
    private AutoAim autoAim;
    private PinpointLocalizer pinpointLocalizer;

    private int stage = 1; // 1 = kp, 2 = ki, 3 = kd

    @Override
    public void init() {
        pinpointLocalizer = new PinpointLocalizer(hardwareMap, Constants.pinpointConstants);
        pinpointLocalizer.resetIMU();

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcamProcessor = new WebcamProcessor(webcam1, telemetry, Constants.webcamProcessorInputs);
        webcamProcessor.initialize();

        DoubleSupplier yawInRadProvider = () -> pinpointLocalizer.getPose().getHeading();
        mecanumDrive = new MecanumDrive(hardwareMap, Constants.mecanumConstants, telemetry, 1.0, yawInRadProvider);

        DoubleSupplier yawInDegProvider = () -> radToDeg(pinpointLocalizer.getPose().getHeading());
        autoAim = new AutoAim(yawInDegProvider);
    }

    private double radToDeg(double radians) {
        return AngleUnit.DEGREES.fromRadians(radians);
    }

    @Override
    public void loop() {
        webcamProcessor.loopUpdate();
        pinpointLocalizer.update();
        AprilTagDetection detection = webcamProcessor.getLastDetection();

        PIDController pid = autoAim.getYawPidController();
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

        autoAim.loopUpdate(detection == null ? null : detection.ftcPose);
        double yawPid = autoAim.getYawPid();
        telemetry.addData("Yaw PID", yawPid);
        mecanumDrive.loopUpdate(gamepad1, yawPid);
    }
}
