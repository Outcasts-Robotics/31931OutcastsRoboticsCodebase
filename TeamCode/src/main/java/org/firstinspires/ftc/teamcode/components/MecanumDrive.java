package org.firstinspires.ftc.teamcode.components;

import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    public MecanumDrive(HardwareMap hw, MecanumConstants mecanumConstants, DoubleSupplier yawInRadProvider) {
        frontLeftMotor = hw.get(DcMotorEx.class, mecanumConstants.leftFrontMotorName);
        frontRightMotor = hw.get(DcMotorEx.class, mecanumConstants.rightFrontMotorName);
        backLeftMotor = hw.get(DcMotorEx.class, mecanumConstants.leftRearMotorName);
        backRightMotor = hw.get(DcMotorEx.class, mecanumConstants.rightRearMotorName);
        frontLeftMotor.setDirection(mecanumConstants.leftFrontMotorDirection);
        frontRightMotor.setDirection(mecanumConstants.rightFrontMotorDirection);
        backLeftMotor.setDirection(mecanumConstants.leftRearMotorDirection);
        backRightMotor.setDirection(mecanumConstants.rightRearMotorDirection);
        maxPower = mecanumConstants.maxPower;
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
