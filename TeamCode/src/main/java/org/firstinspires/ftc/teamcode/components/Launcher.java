package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher {
    private final DcMotorEx flywheel;
    private final DcMotorEx flywheel2;

    private final Gamepad gamepad;
    private final Servo gate;
    private final MecanumDrive mecanumDrive;
    private volatile double targetRpm = 4500;
    //no all at once shooting
    public Launcher(HardwareMap hardwareMap, Gamepad gamepad, MecanumDrive mecanumDrive) {
        this.flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        this.flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        this.gamepad = gamepad;
        this.gate = hardwareMap.get(Servo.class, "gateServo");
        this.mecanumDrive = mecanumDrive;

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
        flywheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        gate.setDirection(Servo.Direction.REVERSE);
        // this.flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0,0,0,0));
        closeGate();
    }

    private double getFlywheelRPM() {
        return (flywheel.getVelocity() * 60.0) / 28.0;
    }

    private void setFlywheelRPM(double rpm) {
        flywheel.setVelocity((rpm * 28.0) / 60.0);
        flywheel2.setVelocity((rpm * 28.0) / 60.0);
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
        flywheel2.setVelocity(0);
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
            Thread.sleep(1000); //340 too high,170 too low 255 good?
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