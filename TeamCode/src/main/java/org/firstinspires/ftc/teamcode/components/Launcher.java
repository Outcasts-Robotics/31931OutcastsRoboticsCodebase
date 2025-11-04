package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher {
    private static final double IDLE_RPM = 0;
    private static final long IDLE_DELAY_MS = 2000; // Delay before switching to idle speed after last launch
    private static final double RPM_TOLERANCE = 80; // RPM tolerance for considering target speed reached
    private static final double GATE_OPEN_POSITION = 0.2; // Adjust based on your servo's range
    private static final double GATE_CLOSE_POSITION = 0.4; // Adjust based on your servo's range
    private final DcMotorEx flywheel;
    private final Gamepad gamepad;
    private final Servo gate;
    private volatile double targetRpm = 3000;
    private int gateOperationDelayMs = 340; // Time to keep gate open in milliseconds
    private volatile int launchCount = 0;
    private volatile boolean isLaunching = false;
    private long lastLaunchTime = 0;
    private Thread workerThread;

    String flywheelName = "flywheel";
    String gateName = "gateServo";


    public Launcher(HardwareMap hardwareMap, Gamepad gamepad) {
        this.flywheel = hardwareMap.get( DcMotorEx.class, flywheelName);
        this.gamepad = gamepad;
        this.gate = hardwareMap.get(Servo.class, gateName);
        closeGate(); // Ensure gate starts closed
    }

    public void init() {
        // Configure flywheel motor
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        // Start the launch loop in a separate thread
        workerThread = new Thread(this::internalLaunchLoop);
        workerThread.start();

        gate.setDirection(Servo.Direction.REVERSE);

        closeGate(); // Ensure gate starts closed
    }

    private void internalLaunchLoop() {
        while (!Thread.currentThread().isInterrupted()) {
            try {
                if (launchCount > 0 && !isLaunching) {
                    isLaunching = true;
                    lastLaunchTime = System.currentTimeMillis();

                    try {
                        setFlywheelRPM(targetRpm);

                        waitForFlywheelRPM(targetRpm, 2000); // 2 second timeout

                        openGate();

                        Thread.sleep(gateOperationDelayMs);

                        closeGate();

                        synchronized (this) {
                            launchCount = Math.max(0, launchCount - 1);
                        }

                        lastLaunchTime = System.currentTimeMillis();

                        Thread.sleep(100);
                    } finally {
                        isLaunching = false;
                    }
                } else {
                    // When no launches are queued and we're not currently launching,
                    // wait a bit after the last launch before going to idle speed
                    if (!isLaunching && launchCount == 0 &&
                            System.currentTimeMillis() - lastLaunchTime > IDLE_DELAY_MS) {
                        setFlywheelRPM(IDLE_RPM);
                    }

                    Thread.sleep(10);
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }
    }

    public void setFlywheelRPM(double rpm) {
        // Convert RPM to ticks per second
        double ticksPerSecond = (rpm * 28.0) / 60.0; // 28:1 motor with 1:1 gearing to flywheel
        flywheel.setVelocity(ticksPerSecond);
    }

    public double getFlywheelRPM() {
        // Convert ticks per second to RPM
        return (flywheel.getVelocity() * 60.0) / 28.0; // 28:1 motor with 1:1 gearing
    }

    private void setFlywheelPower(double power){
        flywheel.setPower(power);
    }

    private double getFLywheelPower(){
        return flywheel.getPower();
    }

    private void waitForFlywheelRPM(double targetRPM, long timeoutMs) throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (Math.abs(getFlywheelRPM() - targetRPM) > RPM_TOLERANCE) {
            if (timer.milliseconds() > timeoutMs) {
                break; // Timeout reached
            }
            Thread.sleep(10); // Small delay to prevent busy waiting
        }
    }

    private void openGate() {
        gate.setPosition(GATE_OPEN_POSITION);
    }

    private void closeGate() {
        gate.setPosition(GATE_CLOSE_POSITION);
    }

    public void update() {
        if (gamepad.xWasPressed()) {
            synchronized (this) {
                launchCount++;
            }
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

    public void onStop() {
        flywheel.setPower(0);
        closeGate();
        workerThread.interrupt();
    }
}