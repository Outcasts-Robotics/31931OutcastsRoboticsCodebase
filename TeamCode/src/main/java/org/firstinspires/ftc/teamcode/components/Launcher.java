package org.firstinspires.ftc.teamcode.components;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleConsumer;

public class Launcher {
    private final DcMotorEx flywheel;
    private final DcMotorEx flywheel2;

    private final Gamepad gamepad;
    private final Servo gate;
    private final MecanumDrive mecanumDrive;
    private final PIDController pidController;
    private final DoubleConsumer powerSetter;
    private final TelemetryManager panelsTelemetry;
    private volatile double targetRpm = 0;
    private double shootRpm = 5000;


    //no all at once shooting
    public Launcher(HardwareMap hardwareMap, Gamepad gamepad, MecanumDrive mecanumDrive, TelemetryManager panelsTelemetry) {
        this.flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        this.flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        this.gamepad = gamepad;
        this.gate = hardwareMap.get(Servo.class, "gateServo");
        this.mecanumDrive = mecanumDrive;
        this.panelsTelemetry = panelsTelemetry;
        this.powerSetter = v -> {
            flywheel.setPower(max(-1, min(v, 1)));
            flywheel2.setPower(max(-1, min(v, 1)));
        };

        this.pidController = new PIDController(this::getFlywheelRPM, this.powerSetter, panelsTelemetry);
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    private void setTargetRpm(double rpm) {
        targetRpm = rpm;
        pidController.setTarget(targetRpm);
    }

    public void init() {
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        gate.setDirection(Servo.Direction.REVERSE);
        closeGate();
        pidController.start();
    }

    public double getFlywheelRPM() {
        return (flywheel.getVelocity() * 60.0) / 28.0;
    }

    private void waitForFlywheelRPM(double targetRPM) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (Math.abs(getFlywheelRPM() - targetRPM) > 50) {
            if (timer.milliseconds() > 2500) break;
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    private void openGate() {
        gate.setPosition(0.15);
    }

    private void closeGate() {
        gate.setPosition(-.05);
    }

    public void update() {
        if (gamepad.xWasPressed()) {
            mecanumDrive.freeze();
            launch();
        }
    }

    public void onStop() {
        pidController.stop();
        flywheel2.setVelocity(0);
        flywheel.setVelocity(0);
        closeGate();
    }

    public void launch() {
        setTargetRpm(shootRpm);
        waitForFlywheelRPM(targetRpm);
        openGate();
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        closeGate();
        setTargetRpm(0);
    }

    public void changeShootRPM(double rpm) {
        this.shootRpm = rpm;
    }
}