package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.PIDController;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.function.DoubleSupplier;

public class AutoAim {

    public static final double TARGET_STALE_TIMEOUT_SEC = 5.0;
    private final PIDController yawPidController;
    private final DoubleSupplier yawInDegProvider;

    private double lastTargetRobotYaw;
    private final ElapsedTime timer;
    private double yawPid;

    public AutoAim(DoubleSupplier yawInDegProvider) {
        this.yawInDegProvider = yawInDegProvider;
        this.yawPidController = new PIDController(0.01, 0.0, 0.0, new ElapsedTime());
        this.timer = new ElapsedTime();
    }

    public void loopUpdate(AprilTagPoseFtc aprilTagPoseFtc) {
        double robotYaw = yawInDegProvider.getAsDouble();

        if (aprilTagPoseFtc != null) {
            double tagYaw = aprilTagPoseFtc.yaw;
            lastTargetRobotYaw = normalize(robotYaw + tagYaw);
            timer.reset();
        }

        if (timer.seconds() < TARGET_STALE_TIMEOUT_SEC) {
            yawPid = yawPidController.calculate(yawDiff(robotYaw, lastTargetRobotYaw), 0);
        } else {
            yawPidController.reset();
            yawPid = 0;  // no change
        }
    }

    public PIDController getYawPidController() {
        return yawPidController;
    }

    private double yawDiff(double fromYaw, double toYaw) {
        return normalize(toYaw - fromYaw);
    }

    private double normalize(double v) {
        if (v > 180) {
            return v - 360;
        } else if (v < -180) {
            return v + 360;
        } else {
            return v;
        }
    }

    public double getYawPid() {
        return yawPid;
    }
}
