package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.components.LauncherV2;
import org.firstinspires.ftc.teamcode.components.MecanumDrive;
import org.firstinspires.ftc.teamcode.components.Spindex;

@TeleOp
public class TeleOpV2 extends OpMode {
    Spindex spindex;
    LauncherV2 launcher;
    MecanumDrive drive;
    Intake intake;
    PinpointLocalizer pinpointLocalizer;
    TelemetryManager telemetry = PanelsTelemetry.INSTANCE.getTelemetry();


    @Override
    public void init() {
        pinpointLocalizer = new PinpointLocalizer(hardwareMap, new PinpointConstants().hardwareMapName("pinpoint").forwardPodY(-2).strafePodX(-6.5).forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED).strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD));
        spindex = new Spindex(hardwareMap, "spindexMotor", "colorSensor");
        drive = new MecanumDrive(hardwareMap, () -> pinpointLocalizer.getPose().getHeading());
        launcher = new LauncherV2(hardwareMap, drive);
        intake = new Intake(hardwareMap, "intakeMotor");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
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
        pinpointLocalizer.update();
        if(gamepad1.triangle){
            pinpointLocalizer.resetIMU();
        }


        if(gamepad1.right_bumper){
            drive.freeze();
        }
        else if(currentState != RobotState.SHOOT){
            drive.update(gamepad1);
        }

        int shootCommands = 0;
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
                if(spindex.getColorInIntake() == Spindex.ArtifactColor.BLANK){
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
                    spindex.goToSlotOuttake((spindex.getCurrentIntakeSlot() +1 ) % 3);
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
