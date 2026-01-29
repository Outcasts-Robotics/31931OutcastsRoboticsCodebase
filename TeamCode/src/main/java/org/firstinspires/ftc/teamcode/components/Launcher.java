package org.firstinspires.ftc.teamcode.components;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {

    private final DcMotorEx flywheel;
    private final DcMotorEx flywheel2;
    private final Gamepad gamepad;
    private final Servo gate;

    private double targetRpm = 0;

    private static final double SHOOT_RPM = 5000.0;
    private static final double RPM_TOLERANCE = 50.0;
    private static final long SPINUP_TIMEOUT_MS = 2500;
    private static final long GATE_OPEN_MS = 1000;

    private static final double kP = 0.00035;
    private static final double kI = 0.0000008;
    private static final double kD = 0.00002;

    private double integral = 0;
    private double lastError = 0;
    private long lastPidTime = 0;

    public void launch() {
        // Set target RPM and reset PID
        setTargetRpm(SHOOT_RPM);
        
        // Wait for flywheel to reach target speed
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < SPINUP_TIMEOUT_MS) {
            runPid(System.currentTimeMillis());
            if (Math.abs(getFlywheelRPM() - SHOOT_RPM) <= RPM_TOLERANCE) {
                break;
            }
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }
        
        // Open gate to feed disc
        openGate();
        
        // Keep gate open for 1 second
        try {
            Thread.sleep(GATE_OPEN_MS);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return;
        }
        
        // Close gate and stop flywheel
        closeGate();
        setTargetRpm(0);
        flywheel.setPower(0);
        flywheel2.setPower(0);
    }

    private enum State {
        IDLE,
        SPINUP,
        FEED
    }

    private State state = State.IDLE;
    private long stateStartTime = 0;

    public Launcher(HardwareMap hardwareMap, Gamepad gamepad) {
        this.flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        this.flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        this.gamepad = gamepad;
        this.gate = hardwareMap.get(Servo.class, "gateServo");
    }

    public void init() {
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        gate.setDirection(Servo.Direction.REVERSE);
        closeGate();

        lastPidTime = System.currentTimeMillis();
    }

    public void update() {
        long now = System.currentTimeMillis();

        if (gamepad.xWasPressed() && state == State.IDLE) {
            setTargetRpm(SHOOT_RPM);
            transition(State.SPINUP);
        }

        runPid(now);

        switch (state) {
            case IDLE:
                break;

            case SPINUP:
                boolean atSpeed = Math.abs(getFlywheelRPM() - SHOOT_RPM) <= RPM_TOLERANCE;
                boolean timeout = now - stateStartTime >= SPINUP_TIMEOUT_MS;

                if (atSpeed || timeout) {
                    openGate();
                    transition(State.FEED);
                }
                break;

            case FEED:
                if (now - stateStartTime >= GATE_OPEN_MS) {
                    closeGate();
                    setTargetRpm(0);
                    transition(State.IDLE);
                }
                break;
        }
    }

    private void runPid(long now) {
        double dt = (now - lastPidTime) / 1000.0;
        lastPidTime = now;

        if (dt <= 0) return;

        double error = targetRpm - getFlywheelRPM();
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = kP * error + kI * integral + kD * derivative;
        output = max(-1, min(output, 1));

        flywheel.setPower(output);
        flywheel2.setPower(output);
    }

    private void transition(State newState) {
        state = newState;
        stateStartTime = System.currentTimeMillis();
    }

    private void setTargetRpm(double rpm) {
        targetRpm = rpm;
        integral = 0;
        lastError = 0;
    }

    public double getFlywheelRPM() {
        double rpm1 = (flywheel.getVelocity() * 60.0) / 28.0;
        double rpm2 = (flywheel2.getVelocity() * 60.0) / 28.0;
        return (rpm1 + rpm2) / 2.0;
    }

    private void openGate() {
        gate.setPosition(0.15);
    }

    private void closeGate() {
        gate.setPosition(-0.05);
    }

    public void onStop() {
        state = State.IDLE;
        setTargetRpm(0);
        flywheel.setPower(0);
        flywheel2.setPower(0);
        closeGate();
    }

    public double getTargetRpm() {
        return targetRpm;
    }
}
