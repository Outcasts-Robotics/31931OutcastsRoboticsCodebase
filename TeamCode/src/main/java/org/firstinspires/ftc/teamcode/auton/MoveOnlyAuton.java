package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class MoveOnlyAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        final DcMotor frontLeft = hardwareMap.get(DcMotor.class, "fl");
        final DcMotor frontRight = hardwareMap.get(DcMotor.class, "fr");
        final DcMotor rearRight = hardwareMap.get(DcMotor.class, "rr");
        final DcMotor rearLeft = hardwareMap.get(DcMotor.class, "rl");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setPower(.3);
        frontRight.setPower(-.3);
        rearLeft.setPower(-.3);
        rearRight.setPower(.3);

        Thread.sleep(2000);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }
}
