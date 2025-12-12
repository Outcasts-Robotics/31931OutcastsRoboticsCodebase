package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.bylazar.field.Line;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.components.ArtifactColor;
import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.components.LauncherV2;
import org.firstinspires.ftc.teamcode.components.MecanumDrive;
import org.firstinspires.ftc.teamcode.components.Spindex;
import org.firstinspires.ftc.teamcode.components.Vision;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "TeleOpV2", group = "TeleOp")
public class TeleOpV2 extends OpMode {
    Spindex spindex;
    LauncherV2 launcher;
    MecanumDrive drive;
    Intake intake;
    PinpointLocalizer pinpointLocalizer;
    TelemetryManager telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    Vision vision = new Vision(hardwareMap);
    Follower follower;
    Pose correctPose;

    Timer poseUpdateTimer = new Timer();


    int shootCommands;
    @Override
    public void init() {
        pinpointLocalizer = new PinpointLocalizer(hardwareMap, Constants.localizerConstants);
        spindex = new Spindex(hardwareMap, "spindexMotor", "colorSensor");
        drive = new MecanumDrive(hardwareMap, () -> pinpointLocalizer.getPose().getHeading());
        launcher = new LauncherV2(hardwareMap, drive);
        intake = new Intake(hardwareMap, "intakeMotor");
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        shootCommands = 0;

        poseUpdateTimer.resetTimer();


    }

    public enum RobotState {
        INTAKE,
        PREP_SHOOT,
        SHOOT,
        PARK
    }

    public RobotState currentState = RobotState.INTAKE;

    @Override
    public void loop() {

        vision.update();
        pinpointLocalizer.update();
        follower.update();
        double angularVelocityLimit = Math.PI / 180 * 5;  // not rotating
        double velocityLimit = 1;  // not moving

        Pose velocity = pinpointLocalizer.getVelocity();
        double linearSpeed = Math.hypot(velocity.getX(), velocity.getY());
        double angularSpeed = Math.abs(velocity.getHeading());
        if (vision.getPositioningTag() != null &&
                linearSpeed < velocityLimit &&
                angularSpeed < angularVelocityLimit &&
                poseUpdateTimer.getElapsedTimeSeconds() > 3) {
            poseUpdateTimer.resetTimer();
            Pose3D pose3dFtc = vision.getRobotPoseFtc();
            Position posFtc = pose3dFtc.getPosition();
            YawPitchRollAngles yprFtc = pose3dFtc.getOrientation();
            Pose pose = new Pose(posFtc.x, posFtc.y, yprFtc.getYaw(AngleUnit.RADIANS), FTCCoordinates.INSTANCE);
            pinpointLocalizer.setPose(pose);
            follower.setPose(pose);
            telemetry.addLine("Pose updated from vision");
        }

        if(gamepad1.triangle){
            pinpointLocalizer.resetIMU();
        }



        if(gamepad1.right_bumper){

            follower.followPath(follower.pathBuilder().addPath(new BezierLine(pinpointLocalizer.getPose(), correctPose)).build());
        }else{
            correctPose = follower.getPose();
        }

        if(currentState != RobotState.SHOOT && !gamepad1.right_bumper){
            drive.update(gamepad1);
        }


        switch (currentState){
            case INTAKE:
                if(spindex.currentMode == Spindex.SpinDexMode.SHOOT){
                    spindex.setCurrentMode(Spindex.SpinDexMode.INTAKE);
                }
                if(gamepad1.dpad_down){
                    currentState = RobotState.PARK;
                }
                if(gamepad2.dpad_up && !spindex.isMotorRunning()){
                    currentState = RobotState.PREP_SHOOT;
                    break;
                }
                if(gamepad2.triangle){
                    spindex.goToSlotIntake(0);
                    spindex.startMotor();
                }
                if(gamepad2.square){
                    spindex.goToSlotIntake(1);
                    spindex.startMotor();
                }
                if(gamepad2.cross){
                    spindex.goToSlotIntake(2);
                    spindex.startMotor();
                }
                if(gamepad2.circle){
                    spindex.goToSlotIntake((spindex.getCurrentIntakeSlot() +1 ) % 3);
                    spindex.stopMotor();
                }
                if(!spindex.isMotorRunning()){
                    spindex.stopMotor();
                }
                if(abs(gamepad2.left_stick_y) > .1){
                    intake.spinUptoPower(gamepad2.left_stick_y * intake.MOTOR_POWER);
                }
                if(spindex.getColorInIntake() == ArtifactColor.BLANK){
                    spindex.intakeColorDetect();
                }
                break;
            case PREP_SHOOT:
                if(spindex.currentMode == Spindex.SpinDexMode.INTAKE){
                    spindex.setCurrentMode(Spindex.SpinDexMode.SHOOT);
                }

                if(gamepad2.right_bumper && !spindex.isMotorRunning()){
                    currentState = RobotState.SHOOT;
                    shootCommands = 1;
                    break;
                }
                if(gamepad2.dpad_left && !spindex.isMotorRunning()){
                    currentState = RobotState.SHOOT;
                    shootCommands = 2;
                    break;
                }
                if(gamepad2.dpad_right && !spindex.isMotorRunning()){
                    currentState = RobotState.SHOOT;
                    shootCommands = 3;
                    break;
                }

                if(gamepad2.triangle){
                    spindex.goToSlotOuttake(0);
                    spindex.startMotor();
                }
                if(gamepad2.square){
                    spindex.goToSlotOuttake(1);
                    spindex.startMotor();
                }
                if(gamepad2.cross){
                    spindex.goToSlotOuttake(2);
                    spindex.startMotor();
                }
                if(gamepad2.circle){
                    spindex.goToSlotOuttake((spindex.getCurrentOuttakeSlot() +1 ) % 3);
                    spindex.startMotor();
                }
                if(!spindex.isMotorRunning()){
                    spindex.stopMotor();
                }
                if(gamepad2.dpad_up){
                    currentState = RobotState.INTAKE;
                }

                break;
            case SHOOT:
                switch (shootCommands){
                    case 1:
                        try {
                            launcher.shootOne(spindex);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                        break;
                    case 2:
                        try {
                            launcher.shootAllfromCurrent(spindex);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                        break;
                    case 3:
                        try {
                            launcher.shootAllfromZero(spindex);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                        break;
                    default:
                        throw new RuntimeException("Wrong Launching Command: " + shootCommands);
                }
                currentState = RobotState.PREP_SHOOT;
                shootCommands = 0;
                break;
            case PARK:

                break;
        }

        telemetry.addData("Status", "Running");
        telemetry.addData("Intake Slot", spindex.getCurrentIntakeSlot() +" : " +spindex.getColorInIntake());
        telemetry.addData("Outtake Slot", spindex.getCurrentOuttakeSlot() +" : " +spindex.getColorInOuttake());
        telemetry.addData("Intake Motor", spindex.isMotorRunning());
        telemetry.addData("Pose", pinpointLocalizer.getPose());
        telemetry.addData("State", currentState);
        telemetry.update();
    }
}
