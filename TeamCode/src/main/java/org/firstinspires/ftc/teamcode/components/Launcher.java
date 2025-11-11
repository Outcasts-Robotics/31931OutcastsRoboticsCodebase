package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher {
    private final DcMotorEx flywheel;
    private final Gamepad gamepad;
    private final Servo gate;
    private volatile double targetRpm = 3000;

    public Launcher(HardwareMap hardwareMap, Gamepad gamepad) {
        this.flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        this.gamepad = gamepad;
        this.gate = hardwareMap.get(Servo.class, "gateServo");
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public void setTargetRpm(double targetRpm) {
        this.targetRpm = targetRpm;
    }

    public void init() {
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        gate.setDirection(Servo.Direction.REVERSE);
        closeGate();
    }

    private double getFlywheelRPM() {
        return (flywheel.getVelocity() * 60.0) / 28.0;
    }

    private void setFlywheelRPM(double rpm) {
        flywheel.setVelocity((rpm * 28.0) / 60.0);
    }

    private void waitForFlywheelRPM(double targetRPM) throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (Math.abs(getFlywheelRPM() - targetRPM) > 100) {
            if (timer.milliseconds() > 2500) break;
            Thread.sleep(10);
        }
    }

    private void openGate() {
        gate.setPosition(0.2);
    }

    private void closeGate() {
        gate.setPosition(0.4);
    }

    public void update() {
        if (gamepad.xWasPressed()) {
            launch();
        }
    }

    public void onStop() {
        flywheel.setVelocity(0);
        closeGate();
    }

    public void launch() {
        setFlywheelRPM(targetRpm);
        try {
            waitForFlywheelRPM(targetRpm);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        openGate();
        try {
            Thread.sleep(340);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        closeGate();
        try {
            Thread.sleep(340);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        setFlywheelRPM(0);
    }
}