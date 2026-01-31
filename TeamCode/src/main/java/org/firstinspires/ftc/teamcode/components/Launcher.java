package org.firstinspires.ftc.teamcode.components;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.bylazar.telemetry.TelemetryManager;

public class Launcher {

    private final DcMotorEx flywheel;
    private final DcMotorEx flywheel2;
    private final Gamepad gamepad;
    private final Servo gate;
    private final PIDController pidController;

    private double targetRpm = 0;

    private static final double SHOOT_RPM = 5500.0;
    private static final double RPM_TOLERANCE = 50.0;
    private static final long SPINUP_TIMEOUT_MS = 2500;
    private static final long GATE_OPEN_MS = 1000;

    public void launch() {
        // Set target RPM
        setTargetRpm(SHOOT_RPM);
        
        // Wait for flywheel to reach target speed
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < SPINUP_TIMEOUT_MS) {
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
    }

    private enum State {
        IDLE,
        SPINUP,
        FEED
    }

    private State state = State.IDLE;
    private long stateStartTime = 0;

    public Launcher(HardwareMap hardwareMap, Gamepad gamepad, TelemetryManager telemetryManager) {
        this.flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        this.flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        this.gamepad = gamepad;
        this.gate = hardwareMap.get(Servo.class, "gateServo");
        
        // Initialize PIDController with current state supplier and power setter
        this.pidController = new PIDController(
            this::getFlywheelRPM,
            power -> {
                double clampedPower = Math.max(-1, Math.min(1, power));
                flywheel.setPower(clampedPower);
                flywheel2.setPower(clampedPower);
            },
            telemetryManager
        );
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

        // Start the PID controller thread
        pidController.start();
    }

    public void update() {
        long now = System.currentTimeMillis();



        if (gamepad.xWasPressed() && state == State.IDLE) {
            setTargetRpm(SHOOT_RPM);
            transition(State.SPINUP);
        }

        if (gamepad.triangleWasPressed() && state == State.IDLE) {
            setTargetRpm(-220);
            transition(State.SPINUP);
        }





        switch (state) {
            case IDLE:
                break;

            case SPINUP:
                boolean atSpeed = Math.abs(getFlywheelRPM() - targetRpm) <= RPM_TOLERANCE;
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

    private void setTargetRpm(double rpm) {
        targetRpm = rpm;
        pidController.setTarget(rpm);
    }

    private void transition(State newState) {
        state = newState;
        stateStartTime = System.currentTimeMillis();
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
        pidController.stop();
        flywheel.setPower(0);
        flywheel2.setPower(0);
        closeGate();
    }

    public double getTargetRpm() {
        return targetRpm;
    }
}
