package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class motorTest extends OpMode {

    DcMotor motor;

    @Override
    public void init() {
         motor = (DcMotor) hardwareMap.get("motorTest");

    }

    @Override
    public void loop() {
        double power = gamepad1.left_stick_y;
        motor.setPower(power);
        telemetry.addData("Motor Power", power);
    }
}
