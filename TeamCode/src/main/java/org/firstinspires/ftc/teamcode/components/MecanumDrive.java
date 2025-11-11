package org.firstinspires.ftc.teamcode.components;

import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.function.DoubleSupplier;

public class MecanumDrive {
    private final double maxPower;
    private final DcMotorEx frontLeftMotor;
    private final DcMotorEx frontRightMotor;
    private final DcMotorEx backLeftMotor;
    private final DcMotorEx backRightMotor;
    private final DoubleSupplier yawInRadProvider;

    public MecanumDrive(HardwareMap hw, DoubleSupplier yawInRadProvider) {
        frontLeftMotor = hw.get(DcMotorEx.class, "fl");
        frontRightMotor = hw.get(DcMotorEx.class, "fr");
        backLeftMotor = hw.get(DcMotorEx.class, "rl");
        backRightMotor = hw.get(DcMotorEx.class, "rr");
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        maxPower = 0.7;
        this.yawInRadProvider = yawInRadProvider;

    }

    public void update(Gamepad gamepad) {
        boolean relativeToBot = yawInRadProvider == null || gamepad.left_bumper;
        setDrive(-gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x, relativeToBot);
    }

    private void setDrive(double forward, double strafe, double rotate, boolean relativeToBot) {
        if (!relativeToBot) {
            double theta = Math.atan2(forward, strafe);
            double r = Math.hypot(strafe, forward);
            theta = AngleUnit.normalizeRadians(theta - yawInRadProvider.getAsDouble());
            double newForward = r * Math.sin(theta);
            double newRight = r * Math.cos(theta);
            setDrive(newForward, newRight, rotate, true);
        } else {
            double frontLeftPower = forward + strafe + rotate;
            double frontRightPower = forward - strafe - rotate;
            double backRightPower = forward + strafe - rotate;
            double backLeftPower = forward - strafe + rotate;
            double denominator = 1.0;
            denominator = Math.max(denominator, Math.abs(frontLeftPower));
            denominator = Math.max(denominator, Math.abs(frontRightPower));
            denominator = Math.max(denominator, Math.abs(backRightPower));
            denominator = Math.max(denominator, Math.abs(backLeftPower));
            frontLeftMotor.setPower(maxPower * (frontLeftPower / denominator));
            frontRightMotor.setPower(maxPower * (frontRightPower / denominator));
            backLeftMotor.setPower(maxPower * (backLeftPower / denominator));
            backRightMotor.setPower(maxPower * (backRightPower / denominator));
        }
    }
}
