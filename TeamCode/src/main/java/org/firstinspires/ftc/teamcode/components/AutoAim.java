package org.firstinspires.ftc.teamcode.components;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.FuturePose;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.teamcode.helpers.PIDController;

public class AutoAim {

    private final PIDController headingPidController;
    private final FuturePose futurePose;

    private final Telemetry telemetry;
    private double headingPid;
    private boolean active;
    private Pose2D target;

    public AutoAim(FuturePose futurePose, Telemetry telemetry) {
        this.futurePose = futurePose;
        this.headingPidController = new PIDController(9, 0.1, 0.6, new ElapsedTime());
        this.telemetry = telemetry;
    }

    public void update() {
        telemetry.addData("autoaim", isActive());
        Pose pose = futurePose.getPose();

        telemetry.addData("robotPose", String.format("x=%.1f y=%.1f h=%.1f", pose.getX(), pose.getY(),
                AngleUnit.DEGREES.fromRadians(pose.getHeading())));

        Pose targetPoseFtc = new Pose(target.getX(DistanceUnit.INCH), target.getY(DistanceUnit.INCH), 0, FTCCoordinates.INSTANCE);
        Pose targetPose = Utils.toPedro(targetPoseFtc);
        telemetry.addData("aimAtPose", String.format("x=%.1f y=%.1f", targetPose.getX(), targetPose.getY()));

        Pose robotToTarget = targetPose.minus(pose);
        double targetHeading = Math.atan2(robotToTarget.getY(), robotToTarget.getX());
        telemetry.addData("targetHeading", AngleUnit.DEGREES.fromRadians(targetHeading));
        double headingError = AngleUnit.normalizeRadians(targetHeading - pose.getHeading());
        if (!isActive()) {
            return;
        }
        headingPid = headingPidController.calculate(headingError, 0);
        telemetry.addData("headingPid", headingPid);
    }

    public PIDController getHeadingPidController() {
        return headingPidController;
    }

    public double getHeadingPid() {
        if (!isActive()) {
            return 0;
        }
        return headingPid;
    }

    public boolean isActive() {
        return active;
    }

    public void setActive(boolean active) {
        this.active = active;
    }

    public void setTarget(Pose2D target) {
        this.target = target;
    }
}
