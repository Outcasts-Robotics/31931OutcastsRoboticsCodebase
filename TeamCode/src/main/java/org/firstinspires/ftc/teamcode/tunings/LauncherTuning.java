package org.firstinspires.ftc.teamcode.tunings;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.components.Launcher;

@TeleOp
public class LauncherTuning extends OpMode {
    private Launcher launcher;
    @Override
    public void init() {
        launcher = new Launcher(
                hardwareMap,
                gamepad1, null);
        launcher.init();
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()){
            launcher.setTargetRpm(launcher.getTargetRpm() - 250);
        }
        if (gamepad1.bWasPressed()){
            launcher.setTargetRpm(launcher.getTargetRpm() + 250);
        }

        telemetry.addData("rpm", launcher.getTargetRpm());
        launcher.update();
    }
}
