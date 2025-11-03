package org.firstinspires.ftc.teamcode.tunings;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "servo+test")
public class FlywheelGraphwithServo extends OpMode {
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
    public Servo gateServo;
    double closedPosition = .4;
    double openPosition =  .2;

    ElapsedTime timer = new ElapsedTime();

    gateStates currState = gateStates.OPEN;
    public enum gateStates{
        OPEN,
        CLOSED
    }

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        gateServo.setDirection(Servo.Direction.REVERSE);
        gateServo.setPosition(openPosition);

    }

    @Override
    public void init_loop() {
        // Allow tuning of target RPM before starting the opmode
        handleGamepadInput();
        telemetry.addData("Target RPM", targetRPM);
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
        // Adjust target RPM
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

        // Prevent target RPM from going below zero
        if (targetRPM < 0) {
            targetRPM = 0;
        }

        // Toggle flywheel state
        if (gamepad1.xWasPressed()) {
            isFlywheelRunning = !isFlywheelRunning;
        }




        if(gamepad1.circle) {
            if (timer.milliseconds() > 500) {
                timer.reset();
                if (currState == gateStates.CLOSED) {
                    gateServo.setPosition(openPosition);
                    currState = gateStates.OPEN;
                } else {
                    gateServo.setPosition(closedPosition);
                    currState = gateStates.CLOSED;
                }
            }
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
        panelsTelemetry.addData("Gate State", currState);
        panelsTelemetry.addData("Servo Position", gateServo.getController().getServoPosition(gateServo.getPortNumber()));
        panelsTelemetry.update(telemetry);

    }
}
