package org.firstinspires.ftc.teamcode;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.MecanumDrive;


@TeleOp(name = "MyTeleOp", group = "TeleOp")
public class MyTeleOp extends OpMode {
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private MecanumDrive mecanumDrive;
    private PinpointLocalizer pinpointLocalizer;
    private DcMotorEx flywheel;
    private Servo gate;

    @Override
    public void init() {
        pinpointLocalizer = new PinpointLocalizer(hardwareMap, new PinpointConstants()
                .hardwareMapName("pinpoint")
                .forwardPodY(-2)
                .strafePodX(-6.5)
                .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
                .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD));
        pinpointLocalizer.resetIMU();
        mecanumDrive = new MecanumDrive(hardwareMap, () -> pinpointLocalizer.getPose().getHeading());
        this.flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        this.gate = hardwareMap.get(Servo.class, "gateServo");
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        gate.setDirection(Servo.Direction.REVERSE);
        closeGate();
    }

    @Override
    public void loop() {
        pinpointLocalizer.update();
        mecanumDrive.update(gamepad1);
        if (gamepad1.rightBumperWasPressed()) {
            pinpointLocalizer.resetIMU();
        }
        panelsTelemetry.update(telemetry);

        if (gamepad1.xWasPressed()){
            setFlywheelRPM(3000);
            openGate();
            try {
                Thread.sleep(340);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            closeGate();
        }

    }

    @Override
    public void stop() {
        flywheel.setVelocity(0);
        closeGate();
    }

    private void openGate() {
        gate.setPosition(0.2);
    }

    private void closeGate() {
        gate.setPosition(0.4);
    }
    private void setFlywheelRPM(double rpm) {
        flywheel.setVelocity((rpm * 28.0) / 60.0);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (Math.abs(getFlywheelRPM() - rpm) > 100) {
            if (timer.milliseconds() > 2500) break;
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    private double getFlywheelRPM() {
        return (flywheel.getVelocity() * 60.0) / 28.0;
    }

}
