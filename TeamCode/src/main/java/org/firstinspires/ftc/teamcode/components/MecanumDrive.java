package org.firstinspires.ftc.teamcode.components;

import static androidx.core.math.MathUtils.clamp;

import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.helpers.UpDownSlewRateLimiter;

import java.util.function.DoubleSupplier;

public class MecanumDrive {
    private final double staticFrictionPower;  // min power to have any movement
    private final double maxPower;
    private final DcMotorEx frontLeftMotor;
    private final DcMotorEx frontRightMotor;
    private final DcMotorEx backLeftMotor;
    private final DcMotorEx backRightMotor;
    private final UpDownSlewRateLimiter flSlew, frSlew, blSlew, brSlew;
    private final DoubleSupplier yawInRadProvider;
    private final Telemetry telemetry;

    public MecanumDrive(HardwareMap hw, MecanumConstants mecanumConstants, Telemetry telemetry, double slewRate, DoubleSupplier yawInRadProvider) {
        frontLeftMotor = hw.get(DcMotorEx.class, mecanumConstants.leftFrontMotorName);
        frontRightMotor = hw.get(DcMotorEx.class, mecanumConstants.rightFrontMotorName);
        backLeftMotor = hw.get(DcMotorEx.class, mecanumConstants.leftRearMotorName);
        backRightMotor = hw.get(DcMotorEx.class, mecanumConstants.rightRearMotorName);

        frontLeftMotor.setDirection(mecanumConstants.leftFrontMotorDirection);
        frontRightMotor.setDirection(mecanumConstants.rightFrontMotorDirection);
        backLeftMotor.setDirection(mecanumConstants.leftRearMotorDirection);
        backRightMotor.setDirection(mecanumConstants.rightRearMotorDirection);

        maxPower = mecanumConstants.maxPower;

        this.telemetry = telemetry;
        this.yawInRadProvider = yawInRadProvider;

        // slew rate limiter - brake rate doubled for faster slowdown
        flSlew = new UpDownSlewRateLimiter(slewRate, slewRate * 2, true);
        frSlew = new UpDownSlewRateLimiter(slewRate, slewRate * 2, true);
        blSlew = new UpDownSlewRateLimiter(slewRate, slewRate * 2, true);
        brSlew = new UpDownSlewRateLimiter(slewRate, slewRate * 2, true);

        staticFrictionPower = 0.1;
    }

    public void loopUpdate(Gamepad gamepad, double yawPid) {
        double speedScale = 0.3 + (1 - gamepad.right_trigger) * 0.7; // slow down to 30%

        boolean relativeToBot = yawInRadProvider == null || gamepad.left_bumper;

        // pid deadzone
        if (Math.abs(yawPid) < 0.06) {
            yawPid = 0;
        }

        yawPid = clamp(yawPid, -0.3, 0.3); // max rotation speed is clamped to avoid over spinning
        if (Math.abs(yawPid) < staticFrictionPower) {
            yawPid = staticFrictionPower * Math.signum(yawPid);
        }

        setDrive(-gamepad.left_stick_y * speedScale,
                gamepad.left_stick_x * speedScale,
                gamepad.right_stick_x * speedScale + yawPid,
                relativeToBot);
    }

    public void setDrive(double forward, double strafe, double rotate, boolean relativeToBot) {
        if (!relativeToBot) {
            double theta = Math.atan2(forward, strafe);
            double r = Math.hypot(strafe, forward);

            // Second, rotate angle by the angle the robot is pointing
            theta = AngleUnit.normalizeRadians(theta - yawInRadProvider.getAsDouble());

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

            double denominator = 1.0;
            denominator = Math.max(denominator, Math.abs(frontLeftPower));
            denominator = Math.max(denominator, Math.abs(frontRightPower));
            denominator = Math.max(denominator, Math.abs(backRightPower));
            denominator = Math.max(denominator, Math.abs(backLeftPower));

            frontLeftMotor.setPower(flSlew.apply(maxPower * (frontLeftPower / denominator)));
            frontRightMotor.setPower(frSlew.apply(maxPower * (frontRightPower / denominator)));
            backLeftMotor.setPower(blSlew.apply(maxPower * (backLeftPower / denominator)));
            backRightMotor.setPower(brSlew.apply(maxPower * (backRightPower / denominator)));
        }
    }
}
