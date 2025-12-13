package org.firstinspires.ftc.teamcode;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Launcher;
import org.firstinspires.ftc.teamcode.components.MecanumDrive;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp(name = "MyTeleOp", group = "TeleOp")
public class MyTeleOp extends OpMode {
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private MecanumDrive mecanumDrive;
    private PinpointLocalizer pinpointLocalizer;
    private Launcher launcher;
    private Follower follower;
    private Pose correctPose = new Pose(72,8, 0);

    @Override
    public void init() {
        pinpointLocalizer = new PinpointLocalizer(hardwareMap, Constants.localizerConstants);
        pinpointLocalizer.resetIMU();
        follower = new FollowerBuilder(Constants.followerConstants, hardwareMap).build();
        follower.setStartingPose(new Pose(72, 8, 0));
        mecanumDrive = new MecanumDrive(hardwareMap, () -> pinpointLocalizer.getPose().getHeading());
        launcher = new Launcher(hardwareMap, gamepad1, mecanumDrive);
        launcher.init();
    }

    @Override
    public void loop() {
        pinpointLocalizer.update();
        follower.update();
        if(gamepad1.right_trigger < .2){
            mecanumDrive.update(gamepad1);
            correctPose = follower.getPose();
        } else {
            follower.followPath(follower.pathBuilder().addPath(new BezierLine(pinpointLocalizer.getPose(), correctPose)).build());
        }

        if (gamepad1.rightBumperWasPressed()) {
            pinpointLocalizer.resetIMU();
        }


        panelsTelemetry.update(telemetry);
        launcher.update();
    }

    @Override
    public void stop() {
        launcher.onStop();
    }
}
