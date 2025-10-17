package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

@TeleOp(name = "flywheeltest")
public class flywheelTest extends OpMode {


    public Flywheel flywheel;
    @Override
    public void init() {
        flywheel = new Flywheel(hardwareMap);

    }

    @Override
    public void loop() {
        if (gamepad1.circle){
            flywheel.startMotor();
            telemetry.addData("starting", "true");
        }

        if (gamepad1.triangle){
            flywheel.stopMotor();
        }

        if(gamepad1.cross){
            flywheel.setTargetPower(.5);
        }

        flywheel.updatePID();

        telemetry.addData("Flywheel RPM", flywheel.getCurrentRPM());
        telemetry.addData("Flywheel Motor Power", flywheel.getCurrentPower());
        telemetry.update();
    }
}
