package org.firstinspires.ftc.teamcode.tunings;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class FlywheelGraph extends OpMode {
    // Constants
    private static final double SECS_PER_MIN = 60.0;
    private static final double TICKS_PER_REV = 28.0;

    // Telemetry
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    // Hardware
    private DcMotorEx flywheel;

    // State
    private double targetRPM = 0;
    private boolean isFlywheelRunning = false;
    private boolean rightTriggerPressed = false;
    private boolean leftTriggerPressed = false;

    // PID tuning state
    private PIDFCoefficients pidfCoefficients;
    private enum PIDCoefficient { P, I, D }
    private PIDCoefficient selectedCoefficient = PIDCoefficient.P;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Get the current PIDF coefficients
        pidfCoefficients = flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void init_loop() {
        // Allow tuning of target RPM before starting the opmode
        handleGamepadInput();
        telemetry.addData("Target RPM", targetRPM);

        // Display PID coefficients in init_loop as well
        telemetry.addData("Tuning", selectedCoefficient.name());
        telemetry.addData("P", pidfCoefficients.p);
        telemetry.addData("I", pidfCoefficients.i);
        telemetry.addData("D", pidfCoefficients.d);
        telemetry.update();
    }

    @Override
    public void loop() {
        handleGamepadInput();
        updateFlywheel();
        updateTelemetry();
    }

    /**
     * Handles gamepad inputs for controlling the flywheel.
     */
    private void handleGamepadInput() {
        // Gamepad 1 controls
        if (gamepad1.rightBumperWasPressed()) {
            targetRPM += 100;
        } else if (gamepad1.leftBumperWasPressed()) {
            targetRPM -= 100;
        }

        boolean rightTrigger = gamepad1.right_trigger > 0.5;
        if (rightTrigger && !rightTriggerPressed) {
            targetRPM = (((int) (targetRPM / 500)) + 1) * 500;
        }
        rightTriggerPressed = rightTrigger;

        boolean leftTrigger = gamepad1.left_trigger > 0.5;
        if (leftTrigger && !leftTriggerPressed) {
            targetRPM = ((int) ((targetRPM - 1) / 500)) * 500;
        }
        leftTriggerPressed = leftTrigger;

        if (targetRPM < 0) {
            targetRPM = 0;
        }

        if (gamepad1.xWasPressed()) {
            isFlywheelRunning = !isFlywheelRunning;
        }

        // Gamepad 2 for PID tuning
        if (gamepad2.yWasPressed()) {
            selectedCoefficient = PIDCoefficient.P;
        }

        if (gamepad2.bWasPressed()) {
            selectedCoefficient = PIDCoefficient.I;
        }

        if (gamepad2.aWasPressed()) {
            selectedCoefficient = PIDCoefficient.D;
        }

        double p_step = 0.5;
        double i_step = 0.05;
        double d_step = 0.005;

        boolean pidChanged = false;
        if (gamepad2.dpadUpWasPressed()) {
            switch (selectedCoefficient) {
                case P: pidfCoefficients.p += p_step; break;
                case I: pidfCoefficients.i += i_step; break;
                case D: pidfCoefficients.d += d_step; break;
            }
            pidChanged = true;
        }

        if (gamepad2.dpadDownWasPressed()) {
            switch (selectedCoefficient) {
                case P: pidfCoefficients.p -= p_step; break;
                case I: pidfCoefficients.i -= i_step; break;
                case D: pidfCoefficients.d -= d_step; break;
            }
            pidChanged = true;
        }

        if (pidChanged) {
            pidfCoefficients.p = Math.max(0, pidfCoefficients.p);
            pidfCoefficients.i = Math.max(0, pidfCoefficients.i);
            pidfCoefficients.d = Math.max(0, pidfCoefficients.d);
            flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        }
    }

    /**
     * Updates the flywheel's velocity based on the current state.
     */
    private void updateFlywheel() {
        double targetVelocity = targetRPM * TICKS_PER_REV / SECS_PER_MIN;
        flywheel.setVelocity(isFlywheelRunning ? targetVelocity : 0);
    }

    /**
     * Updates the telemetry with flywheel information.
     */
    private void updateTelemetry() {
        double flywheelRPM = flywheel.getVelocity() * SECS_PER_MIN / TICKS_PER_REV;
        panelsTelemetry.addData("rpm_target", targetRPM);
        panelsTelemetry.addData("rpm_actual", flywheelRPM);
        panelsTelemetry.addData("power", flywheel.getPower());
        panelsTelemetry.addData("running", isFlywheelRunning);

        // PID Telemetry
        panelsTelemetry.addData("Tuning", selectedCoefficient.name());
        panelsTelemetry.addData("P", pidfCoefficients.p);
        panelsTelemetry.addData("I", pidfCoefficients.i);
        panelsTelemetry.addData("D", pidfCoefficients.d);

        panelsTelemetry.update(telemetry);
    }
}
