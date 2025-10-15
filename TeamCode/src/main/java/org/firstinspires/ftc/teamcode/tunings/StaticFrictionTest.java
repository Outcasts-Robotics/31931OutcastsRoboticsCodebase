package org.firstinspires.ftc.teamcode.tunings;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.components.MecanumDrive;

@TeleOp(group = "Tunings")
public class StaticFrictionTest extends OpMode {
    MecanumDrive mecanumDrive;
    int stage = 0; // 0 - forward, 1 - strafe, 2 - rotate
    double power = 0.0;

    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(hardwareMap, Constants.mecanumConstants, telemetry, 1.0, null);
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            power -= 0.005;
        }
        if (gamepad1.bWasPressed()) {
            power += 0.005;
        }

        if (gamepad1.xWasPressed()) {
            power = 0;
            stage = (stage + 1) % 3;
        }
        telemetry.addData("Stage", stage == 0 ? "Forward" : stage == 1 ? "Strafe" : "Rotate");
        telemetry.addData("Power", power);
        switch (stage) {
            case 0:
                mecanumDrive.setDrive(power, 0, 0, true);
                break;
            case 1:
                mecanumDrive.setDrive(0, power, 0, true);
                break;
            case 2:
                mecanumDrive.setDrive(0, 0, power, true);
                break;
        }
    }
}
