package org.firstinspires.ftc.teamcode.components;


import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Spindex {
    public static final double MOTOR_SPEED = .5;
    private static final double ENCODER_RESOLUTION = 384.5;

    DcMotorEx spindexMotor;
    ColorSensor colorSensor;
    DistanceSensor distanceSensor;

    public SpinDexMode currentMode = SpinDexMode.INTAKE;
    ArtifactColor[] index = {ArtifactColor.BLANK, ArtifactColor.BLANK, ArtifactColor.BLANK};

    int currentSlotIntake = 0;
    int currentSlotOuttake = 1;

    int[] slotPositionsIntake = {0, (int) (ENCODER_RESOLUTION / 3), (int) (ENCODER_RESOLUTION * 2 / 3)};
    int[] slotPositionsOuttake = {(int) (ENCODER_RESOLUTION / 2), (int) (ENCODER_RESOLUTION / 6), (int) (ENCODER_RESOLUTION * 5 / 6)};

    public Spindex(HardwareMap hardwareMap, String motorName, String colorSensorName) {
        this.spindexMotor = hardwareMap.get(DcMotorEx.class, motorName);
        this.colorSensor = hardwareMap.get(ColorSensor.class, colorSensorName);
        this.distanceSensor = hardwareMap.get(DistanceSensor.class, colorSensorName);
        spindexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexMotor.setTargetPositionTolerance(2);
        colorSensor.enableLed(true);
    }

    public void preLoad(ArtifactColor[] newIndex){
        if(newIndex.length == 3) {
            this.index = newIndex;
        } else{
            throw new Error("Index is wrong size");
        }
    }

    public void goToColorIntake(ArtifactColor color){
        for(int i = 0; i < index.length; i++){
            if(index[i] == color){
                goToSlotIntake(i);
                break;
            }
        }
    }

    public void goToColorOuttake(ArtifactColor color){
        for(int i = 0; i < index.length; i++){
            if(index[i] == color){
                goToSlotOuttake(i);
                break;
            }
        }
    }

    public void goToSlotIntake(int slot) {
        this.currentSlotIntake = slot;
        this.currentSlotOuttake = (slot + 1) % 3;
        spindexMotor.setTargetPosition(slotPositionsIntake[slot]);
        this.currentMode = SpinDexMode.INTAKE;
        // Control motor speed in TeleOp
    }

    public void goToSlotIntakeBlocking(int slot) throws InterruptedException {
        this.currentSlotIntake = slot;
        this.currentSlotOuttake = (slot + 1) % 3;
        spindexMotor.setTargetPosition(slotPositionsIntake[slot]);
        spindexMotor.setPower(MOTOR_SPEED);
        this.currentMode = SpinDexMode.INTAKE;

        while(spindexMotor.isBusy()){
            Thread.sleep(70);
        }

        spindexMotor.setPower(0);
    }

    public void goToSlotOuttake(int slot) {
        this.currentSlotOuttake = slot;
        this.currentSlotIntake = (slot + 2) % 3;
        spindexMotor.setTargetPosition(slotPositionsOuttake[slot]);
        this.currentMode = SpinDexMode.SHOOT;
    }

    public void goToSlotOuttakeBlocking(int slot) throws InterruptedException {
        this.currentSlotOuttake = slot;
        this.currentSlotIntake = (slot + 2) % 3;
        spindexMotor.setTargetPosition(slotPositionsOuttake[slot]);
        spindexMotor.setPower(MOTOR_SPEED);
        this.currentMode = SpinDexMode.SHOOT;

        while(spindexMotor.isBusy()){
            Thread.sleep(70);
        }

        spindexMotor.setPower(0);
    }

    public void setCurrentMode(SpinDexMode mode) {
        if (mode == SpinDexMode.INTAKE) {
            goToSlotIntake(currentSlotIntake);
        } else {
            goToSlotOuttake(currentSlotOuttake);
        }
    }

    public int getCurrentIntakeSlot() {
        return this.currentSlotIntake;
    }

    public int getCurrentOuttakeSlot() {
        return this.currentSlotOuttake;
    }

    public ArtifactColor getColorInIntake() {
        return this.index[currentSlotIntake];
    }

    public ArtifactColor getColorInOuttake() {
        return this.index[currentSlotOuttake];
    }

    public ArtifactColor getColorfromSlot(int slot) {
        return this.index[slot];
    }

    public boolean isMotorRunning(){
        return spindexMotor.isBusy();
    }

    public void stopMotor(){
        spindexMotor.setPower(0);
    }

    public void startMotor(){
        spindexMotor.setPower(MOTOR_SPEED);
    }

    public void clearOuttakeSlot() {
        if (this.currentMode == SpinDexMode.INTAKE) {
            return;
        }
        this.index[currentSlotOuttake] = ArtifactColor.BLANK;
    }

    public void intakeColorDetect() {
        if (currentMode == SpinDexMode.SHOOT) {
            return;
        }
        int detectedRed = colorSensor.red();
        int detectedGreen = colorSensor.green();
        int detectedBlue = colorSensor.blue();

        boolean proximity = distanceSensor.getDistance(DistanceUnit.INCH) <= 3;
        if (detectedGreen > detectedRed && detectedGreen > detectedBlue && proximity) {
            index[currentSlotIntake] = ArtifactColor.GREEN;
        } else if (detectedRed > detectedBlue && proximity) {
            index[currentSlotIntake] = ArtifactColor.PURPLE;
        } else {
            index[currentSlotIntake] = ArtifactColor.BLANK;
        }
    }

    public enum SpinDexMode {
        INTAKE,
        SHOOT
    }

}
