package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Launcher;

@TeleOp(name = "MyTeleOp", group = "TeleOp")
public class MyTeleOp extends OpMode {

    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private PinpointLocalizer pinpointLocalizer;
    private Launcher launcher;

    @Override
    public void init() {
        PinpointConstants constants = new PinpointConstants()
                .hardwareMapName("pinpoint")
                .forwardPodY(-2)
                .strafePodX(-6.5)
                .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
                .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpointLocalizer = new PinpointLocalizer(hardwareMap, constants);
        pinpointLocalizer.resetIMU();

        launcher = new Launcher(hardwareMap, gamepad1);
        launcher.init();
        launcher.onStop();
    }

    @Override
    public void loop() {
        pinpointLocalizer.update();
        launcher.update();

        if (gamepad1.optionsWasPressed()) {
            pinpointLocalizer.resetIMU();
        }

        panelsTelemetry.addData("Target RPM", launcher.getTargetRpm());
        panelsTelemetry.addData("Current RPM", launcher.getFlywheelRPM());
        panelsTelemetry.addData("Pose", pinpointLocalizer.getPose());
        panelsTelemetry.update(telemetry);
    }

    @SuppressLint("NewApi")
    @Override
    public void stop() {
        launcher.onStop();
    }
}
