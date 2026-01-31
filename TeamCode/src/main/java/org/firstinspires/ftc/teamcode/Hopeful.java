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
    public void runOpMode() {

        TelemetryManager telemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        final DcMotor frontLeft = hardwareMap.get(DcMotor.class, "fl");
        final DcMotor frontRight = hardwareMap.get(DcMotor.class, "fr");
        final DcMotor rearRight = hardwareMap.get(DcMotor.class, "rr");
        final DcMotor rearLeft = hardwareMap.get(DcMotor.class, "rl");

        final Launcher launcher = new Launcher(hardwareMap, gamepad1, telemetry);
        launcher.init();

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // 🔒 BRAKE when power = 0
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        // Drive forward
        frontLeft.setPower(0.5);
        frontRight.setPower(0.5);
        rearLeft.setPower(0.5);
        rearRight.setPower(0.5);

        sleep((long) (2000*1.5));

        // Stop
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);

        sleep(500);
        telemetry.addLine("Turning...");
        telemetry.update();

        // 🔄 Clean tank turn (in place)
        frontLeft.setPower(-0.5);
        rearLeft.setPower(-0.5);
        frontRight.setPower(0.5);
        rearRight.setPower(0.5);

        sleep(500);

        // Final stop
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
        telemetry.update();
        frontLeft.setPower(0.5);
        frontRight.setPower(0.5);
        rearLeft.setPower(0.5);
        rearRight.setPower(0.5);

        sleep(500);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
        try {
            launcher.launch();
            Thread.sleep(300);


            telemetry.addLine("Stopping launcher");
            telemetry.update();
            launcher.onStop();
            telemetry.addLine("Stopped launcher");
            telemetry.update();


        } catch(InterruptedException e){
            launcher.onStop();
        }
        launcher.onStop();

    }
}
