package org.firstinspires.ftc.teamcode;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Launcher;
import org.firstinspires.ftc.teamcode.components.MecanumDrive;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp(name = "MyTeleOp", group = "TeleOp")
public class MyTeleOp extends OpMode {
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    private MecanumDrive mecanumDrive;
    private PinpointLocalizer pinpointLocalizer;
    private Launcher launcher;


    @Override
    public void init() {
        pinpointLocalizer = new PinpointLocalizer(hardwareMap, Constants.pinpointConstants);
        pinpointLocalizer.resetIMU();
        mecanumDrive = new MecanumDrive(hardwareMap, Constants.mecanumConstants, () -> pinpointLocalizer.getPose().getHeading());
        launcher = new Launcher(hardwareMap, gamepad1);
        launcher.init();
    }

    @Override
    public void loop() {
        launcher.update();
        pinpointLocalizer.update();
        mecanumDrive.update(gamepad1);
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
    }
}
