package org.firstinspires.ftc.teamcode.tunings;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTester extends OpMode {

    private Servo gate;
    private double openPos = 0.2;
    private double closePos = 0.4;

    @Override
    public void init() {
        gate = hardwareMap.get(Servo.class, "gateServo");
        gate.setDirection(Servo.Direction.REVERSE);
        gate.setPosition(closePos);
    }

    @Override
    public void loop() {
        if (gamepad1.leftBumperWasPressed()) {
            gate.setPosition(openPos);
        }
        if (gamepad1.rightBumperWasPressed()){
            gate.setPosition(closePos);
        }

        double pos = gate.getPosition();
        telemetry.addData("servoPos", pos);

    }
}
