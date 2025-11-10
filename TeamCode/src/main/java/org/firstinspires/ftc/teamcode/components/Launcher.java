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
    private volatile int launchCount = 0;
    private volatile boolean isLaunching = false;
    private long lastLaunchTime = 0;
    private Thread workerThread;
    private volatile boolean isRunning = true;

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
        isRunning = true;
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        if (workerThread == null || !workerThread.isAlive()) {
            workerThread = new Thread(this::internalLaunchLoop);
            workerThread.start();
        }
        gate.setDirection(Servo.Direction.REVERSE);
        closeGate();
    }


    private void internalLaunchLoop() {
        while (isRunning) {
            try {
                if (launchCount > 0) {
                    isLaunching = true;
                    setFlywheelRPM(targetRpm);
                    waitForFlywheelRPM(targetRpm);
                    openGate();
                    Thread.sleep(340);
                    closeGate();
                    synchronized (this) {
                        launchCount = Math.max(0, launchCount - 1);
                        if (launchCount == 0) {
                            isLaunching = false;
                        }
                    }
                    lastLaunchTime = System.currentTimeMillis();
                    Thread.sleep(700);
                } else {
                    if (System.currentTimeMillis() - lastLaunchTime > 2000)
                        setFlywheelRPM(0);
                    Thread.sleep(100);
                }
            } catch (InterruptedException e) {
                isRunning = false; // interrupted
            }
        }
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
            incrementLaunchCount();
        }
    }

    public void incrementLaunchCount() {
        synchronized (this) {
            launchCount++;
        }
    }

    public int getLaunchCount() {
        synchronized (this) {
            return launchCount;
        }
    }

    public boolean isLaunching() {
        return isLaunching;
    }

    public void onStop() throws InterruptedException {
        isRunning = false;
        flywheel.setVelocity(0);
        closeGate();
        workerThread.join();
    }
}