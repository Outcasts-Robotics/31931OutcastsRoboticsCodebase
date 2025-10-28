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
                hardwareMap.get(DcMotorEx.class, "flywheel"),
                hardwareMap.get(Servo.class, "gateServo"),
                gamepad1);
        launcher.init();
    }

    @Override
    public void loop() {
        if (gamepad1.leftBumperWasPressed()) {
            launcher.setGateOperationDelayMs(launcher.getGateOperationDelayMs() - 10);
        } else if (gamepad1.rightBumperWasPressed()) {
            launcher.setGateOperationDelayMs(launcher.getGateOperationDelayMs() + 10);
        }
        telemetry.addData("gateOpenMs", launcher.getGateOperationDelayMs());
        telemetry.addData("toLaunch", launcher.getLaunchCount());
        launcher.update();
    }
}
