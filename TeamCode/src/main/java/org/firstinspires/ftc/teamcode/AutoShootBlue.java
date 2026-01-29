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
public class AutoShootBlue extends LinearOpMode {
    @SuppressLint("NewApi")
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        final DcMotor frontLeft = hardwareMap.get(DcMotor.class, "fl");
        final DcMotor frontRight = hardwareMap.get(DcMotor.class, "fr");
        final DcMotor rearRight = hardwareMap.get(DcMotor.class, "rr");
        final DcMotor rearLeft = hardwareMap.get(DcMotor.class, "rl");
        final Launcher launcher = new Launcher(hardwareMap, gamepad1);

        launcher.init();

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setPower(-.3);
        frontRight.setPower(-.3);
        rearLeft.setPower(-.3);
        rearRight.setPower(-.3);

        Thread.sleep(1000);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);

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