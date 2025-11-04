package org.firstinspires.ftc.teamcode.tunings;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoGateTuner extends OpMode {
    double pos = 0.2;
    private Servo gate;

    @Override
    public void init() {
        gate = hardwareMap.get(Servo.class, "gateServo");
        gate.setDirection(Servo.Direction.REVERSE);
        gate.setPosition(0.2);
    }

    @Override
    public void loop() {
        telemetry.addData("pos", pos);
        if (gamepad1.leftBumperWasPressed()) {
            pos -= 0.05;
            gate.setPosition(pos);
        }
        if (gamepad1.rightBumperWasPressed()) {
            pos += 0.05;
            gate.setPosition(pos);
        }
    }
}
