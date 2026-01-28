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
    private final PIDController pidController;
    private final DoubleConsumer powerSetter;

    private volatile double targetRpm = 0;
    private final double shootRpm = 5000;

    // 🔹 NEW: launcher worker thread
    private Thread launchThread;

    public Launcher(HardwareMap hardwareMap,
                    Gamepad gamepad,
                    TelemetryManager panelsTelemetry) {

        this.flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        this.flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        this.gamepad = gamepad;
        this.gate = hardwareMap.get(Servo.class, "gateServo");

        this.powerSetter = v -> {
            flywheel.setPower(max(-1, min(v, 1)));
            flywheel2.setPower(max(-1, min(v, 1)));
        };

        this.pidController =
                new PIDController(this::getFlywheelRPM, powerSetter, panelsTelemetry);
    }

    public void init() {
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        flywheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        gate.setDirection(Servo.Direction.REVERSE);
        closeGate();

        pidController.start();
    }

    public void update() {
        if (gamepad.xWasPressed()) {

            launch();
        }
    }


    public void launch() {
        // Prevent double-launching
        if (launchThread != null && launchThread.isAlive()) return;

        launchThread = new Thread(() -> {
            try {
                setTargetRpm(shootRpm);


                ElapsedTime timer = new ElapsedTime();
                timer.reset();

                while (!Thread.currentThread().isInterrupted()
                        && Math.abs(getFlywheelRPM() - shootRpm) > 50) {

                    if (timer.milliseconds() > 2500) break;
                    Thread.sleep(10);
                }

                if (Thread.currentThread().isInterrupted()) return;

                // Fire
                openGate();
                Thread.sleep(1000);
                closeGate();

                setTargetRpm(0);

            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        });

        launchThread.start();
    }



    private void setTargetRpm(double rpm) {
        targetRpm = rpm;
        pidController.setTarget(targetRpm);
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
        if (launchThread != null && launchThread.isAlive()) {
            launchThread.interrupt();
        }

        pidController.stop();
        powerSetter.accept(0);
        closeGate();
    }

    public double getTargetRpm() {
        return targetRpm;
    }
}
