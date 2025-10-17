package org.firstinspires.ftc.teamcode.tunings;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.Launcher;
@TeleOp(name = "launchertest")
public class LauncherTest extends OpMode {
    public Launcher launcher;
    public double speedTarget = 3000;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init(){
        launcher = new Launcher(hardwareMap, gamepad1);
        launcher.setFlywheelRPM(speedTarget);

        timer.reset();
    }
    @Override
    public void loop(){
        telemetry.addData("time", timer.milliseconds());
        telemetry.addData("flywheel RPM", launcher.getFlywheelRPM());
        telemetry.update();
    }


}
