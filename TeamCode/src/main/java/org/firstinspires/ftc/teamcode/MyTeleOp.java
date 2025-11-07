package org.firstinspires.ftc.teamcode;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.FuturePose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.AutoAim;
import org.firstinspires.ftc.teamcode.components.Launcher;
import org.firstinspires.ftc.teamcode.components.MecanumDrive;
import org.firstinspires.ftc.teamcode.components.WebcamProcessor;
import org.firstinspires.ftc.teamcode.helpers.LoopTimer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.function.DoubleSupplier;


@TeleOp(name = "MyTeleOp", group = "TeleOp")
public class MyTeleOp extends OpMode {
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    private MecanumDrive mecanumDrive;
    private WebcamProcessor webcamProcessor;
    private AutoAim autoAim;
    private PinpointLocalizer pinpointLocalizer;
    private Launcher launcher;
    private LoopTimer loopTimer;


    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        loopTimer = new LoopTimer(panelsTelemetry, 10);

        pinpointLocalizer = new PinpointLocalizer(hardwareMap, Constants.pinpointConstants);
        pinpointLocalizer.resetIMU();

        webcamProcessor = new WebcamProcessor(hardwareMap, telemetry, Constants.webcamProcessorInputs);
        webcamProcessor.initialize();

        DoubleSupplier pinpointHeading = () -> pinpointLocalizer.getPose().getHeading();
        mecanumDrive = new MecanumDrive(hardwareMap, Constants.mecanumConstants, telemetry, 1.0, pinpointHeading);

//        follower = Constants.createFollower(hardwareMap);

        FuturePose futurePose = () -> pinpointLocalizer.getPose();
        autoAim = new AutoAim(futurePose, telemetry);
        autoAim.setTarget(G.pois.goal);

        launcher = new Launcher(hardwareMap, gamepad1);
        launcher.init();
    }

    @Override
    public void init_loop() {
        webcamProcessor.initLoopUpdate(gamepad1);
        if (gamepad1.leftBumperWasPressed() || gamepad1.rightBumperWasPressed()) {
            G.initGlobals(G.alliance == Alliance.RED ? Alliance.BLUE : Alliance.RED);
        }
        telemetry.addData("Alliance", G.alliance);
        autoAim.setTarget(G.pois.goal);
    }

    @Override
    public void start() {
        telemetry.addData("Status", "Starting");
        loopTimer.reset();
    }

    @Override
    public void loop() {
        loopTimer.update();

        webcamProcessor.update();
        AprilTagDetection tag = webcamProcessor.getDetection();

        launcher.update();

        if (tag != null) {
            pinpointLocalizer.setPose(Utils.toPedro(tag.robotPose));
        }
        pinpointLocalizer.update();
        telemetry.addData("pinpoint", pinpointLocalizer.getPose());

        if (!autoAim.isActive() && gamepad1.left_trigger > 0.2) {
            autoAim.setActive(true);
        } else if (autoAim.isActive() && gamepad1.left_trigger < 0.2) {
            autoAim.setActive(false);
        }

        if (autoAim.isActive() && gamepad1.left_trigger > 0.2) {
            autoAim.update();
            mecanumDrive.update(gamepad1, autoAim.getHeadingPid());
        } else {
            mecanumDrive.update(gamepad1, 0);
        }

        if (gamepad1.rightBumperWasPressed()) {
            pinpointLocalizer.resetIMU();
        }

        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        try {
            launcher.onStop();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        telemetry.addData("Status", "Stopping");
    }
}
