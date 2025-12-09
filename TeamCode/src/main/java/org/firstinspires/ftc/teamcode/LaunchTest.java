package org.firstinspires.ftc.teamcode;

import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.components.LauncherV2;
import org.firstinspires.ftc.teamcode.components.MecanumDrive;
import org.firstinspires.ftc.teamcode.components.Spindex;

public class LaunchTest extends OpMode {
    Spindex spindex;
    LauncherV2 launcher;
    MecanumDrive drive;



    @Override
    public void init() {
        PinpointLocalizer pinpointLocalizer = new PinpointLocalizer(hardwareMap,  new PinpointConstants().hardwareMapName("pinpoint").forwardPodY(-2).strafePodX(-6.5).forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED).strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD));
        drive = new MecanumDrive(hardwareMap, () -> pinpointLocalizer.getPose().getHeading());
        spindex =  new Spindex(hardwareMap, "spindexMotor", "colorSensor");
        launcher = new LauncherV2(hardwareMap, drive);

    }

    @Override
    public void loop() {
        if(gamepad1.circle){
            try {
                spindex.goToSlotIntakeBlocking((spindex.getCurrentIntakeSlot() + 1)% 3);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        if(gamepad1.square){
            try {
                spindex.goToSlotOuttakeBlocking((spindex.getCurrentOuttakeSlot() + 1)% 3);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        if(gamepad1.cross){
            if(spindex.currentMode == Spindex.SpinDexMode.SHOOT){
                try {
                    launcher.shootOne(spindex);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        }


        telemetry.addData("Mode", spindex.currentMode);
        telemetry.addData("Intake Slot", spindex.getCurrentIntakeSlot());
        telemetry.addData("Outtake Slot", spindex.getCurrentOuttakeSlot());
        telemetry.addData("Motor Speed", launcher.getFlywheelRPM());
        telemetry.addData("Motor Target Speed", launcher.getTargetRpm());
        telemetry.update();

    }
}
