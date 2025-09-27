package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrive {
    private final double maxPower;
    private final DcMotorEx frontLeftMotor;
    private final DcMotorEx frontRightMotor;
    private final DcMotorEx backLeftMotor;
    private final DcMotorEx backRightMotor;
    private final IMU imu;

    public MecanumDrive(HardwareMap hw, MecanumConstants mecanumConstants, String imuName) {
        frontLeftMotor = hw.get(DcMotorEx.class, mecanumConstants.leftFrontMotorName);
        frontRightMotor = hw.get(DcMotorEx.class, mecanumConstants.rightFrontMotorName);
        backLeftMotor = hw.get(DcMotorEx.class, mecanumConstants.leftRearMotorName);
        backRightMotor = hw.get(DcMotorEx.class, mecanumConstants.rightRearMotorName);

        frontLeftMotor.setDirection(mecanumConstants.leftFrontMotorDirection);
        frontRightMotor.setDirection(mecanumConstants.rightFrontMotorDirection);
        backLeftMotor.setDirection(mecanumConstants.leftRearMotorDirection);
        backRightMotor.setDirection(mecanumConstants.rightRearMotorDirection);

        maxPower = mecanumConstants.maxPower;

        imu = imuName == null ? null : hw.get(IMU.class, imuName);
    }

    public void loopUpdate(Gamepad gamepad) {
        if (imu != null && gamepad.aWasPressed()) {
            imu.resetYaw();
        }

        double speedScale = 0.2 + (1 - gamepad.right_trigger) * 0.8;
        boolean relativeToBot = imu == null || gamepad.left_bumper;
        setDrive(-gamepad.left_stick_y * speedScale,
                gamepad.left_stick_x * speedScale,
                gamepad.right_stick_x * speedScale,
                relativeToBot);
    }

    private void setDrive(double forward, double strafe, double rotate, boolean relativeToBot) {
        if (!relativeToBot) {
            double theta = Math.atan2(forward, strafe);
            double r = Math.hypot(strafe, forward);

            // Second, rotate angle by the angle the robot is pointing
            theta = AngleUnit.normalizeRadians(theta -
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

            // Third, convert back to cartesian
            double newForward = r * Math.sin(theta);
            double newRight = r * Math.cos(theta);

            // Finally, call the drive method with robot relative forward and right amounts
            setDrive(newForward, newRight, rotate, true);
        } else {
            double frontLeftPower = forward + strafe + rotate;
            double frontRightPower = forward - strafe - rotate;
            double backRightPower = forward + strafe - rotate;
            double backLeftPower = forward - strafe + rotate;

            double max = maxPower;
            max = Math.max(max, Math.abs(frontLeftPower));
            max = Math.max(max, Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backRightPower));
            max = Math.max(max, Math.abs(backLeftPower));

            frontLeftMotor.setPower(maxPower * (frontLeftPower / max));
            frontRightMotor.setPower(maxPower * (frontRightPower / max));
            backLeftMotor.setPower(maxPower * (backLeftPower / max));
            backRightMotor.setPower(maxPower * (backRightPower / max));
        }
    }
}
