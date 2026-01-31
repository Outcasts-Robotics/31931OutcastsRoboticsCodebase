package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.components.Launcher;

@Autonomous
public class Hopeful extends LinearOpMode {
    @SuppressLint("NewApi")
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        TelemetryManager telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        final DcMotor frontLeft = hardwareMap.get(DcMotor.class, "fl");
        final DcMotor frontRight = hardwareMap.get(DcMotor.class, "fr");
        final DcMotor rearRight = hardwareMap.get(DcMotor.class, "rr");
        final DcMotor rearLeft = hardwareMap.get(DcMotor.class, "rl");
        final Launcher launcher = new Launcher(hardwareMap, gamepad1, telemetry);

        launcher.init();

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setPower(.5);
        frontRight.setPower(.5);
        rearLeft.setPower(.5);
        rearRight.setPower(.5);

        Thread.sleep(2000);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);

        telemetry.addLine("Turning...");
        telemetry.update();

        frontRight.setPower(.2);
        frontLeft.setPower(-.2);
        rearRight.setPower(-.2);
        rearLeft.setPower(.2);

        Thread.sleep(1000);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
        telemetry.update();

        Thread.sleep(500);
        telemetry.addLine("Launching...");
        telemetry.update();


        launcher.launch();
        Thread.sleep(300);


        telemetry.addLine("Stopping launcher");
        telemetry.update();
        launcher.onStop();
        telemetry.addLine("Stopped launcher");
        telemetry.update();



    }
}
//blahh