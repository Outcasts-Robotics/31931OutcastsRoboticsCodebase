package org.firstinspires.ftc.teamcode.components;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LauncherV2 {
    private final DcMotorEx flywheel;

    private final Servo hood;
    private final Servo kicker;
    private final MecanumDrive mecanumDrive;
    private volatile double targetRpm = 3000;


    private double hoodAngle = 0;

    public double KICKER_UP = 0.1;
    public double KICKER_DOWN = .5;

    public LauncherV2(HardwareMap hardwareMap, MecanumDrive mecanumDrive) {
        this.flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        this.kicker = hardwareMap.get(Servo.class, "kicker");
        this.mecanumDrive = mecanumDrive;
        this.hood = hardwareMap.get(Servo.class, "hoodServo");
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        kicker.setDirection(Servo.Direction.REVERSE);


    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public void setTargetRpm(double targetRpm) {
        this.targetRpm = targetRpm;
    }

    public double getFlywheelRPM() {
        return (flywheel.getVelocity() * 60.0) / 28.0;
    }

    private void setFlywheelRPM(double rpm) {
        flywheel.setVelocity((rpm * 28.0) / 60.0);
    }

    public void shootAllfromZero(Spindex spindex) throws InterruptedException {
        mecanumDrive.freeze();

        for(int i = 0; i < 3 ; i++){
            if(spindex.getColorfromSlot(i) != Spindex.ArtifactColor.BLANK){
                spindex.goToSlotOuttakeBlocking(i);
                shootOne(spindex);
            }
        }
    }


    public void shootAllfromCurrent(Spindex spindex) throws InterruptedException {
        mecanumDrive.freeze();
        int slot = spindex.currentSlotOuttake;
        for(int i = 0; i < 3 ; i++){
            if(spindex.getColorfromSlot(slot) != Spindex.ArtifactColor.BLANK){
                spindex.goToSlotOuttakeBlocking(slot);
                shootOne(spindex);
            }
            slot = (slot + 1) % 3;
        }
    }


    public void changeHoodAngle(double hoodPos){
        hood.setPosition(hoodPos);
        this.hoodAngle = hoodPos;
    }



    public void shootOne(Spindex spindex) throws InterruptedException {
        mecanumDrive.freeze();
        setFlywheelRPM(targetRpm);
        while(abs(getFlywheelRPM() - targetRpm) >= .05){
            Thread.sleep(70);
        }
        kicker.setPosition(KICKER_UP);
        spindex.clearOuttakeSlot();


        Thread.sleep(50);
        kicker.setPosition(KICKER_DOWN);

        setFlywheelRPM(0);

    }


    public void onStop() {
        flywheel.setVelocity(0);
    }


}