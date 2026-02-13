package org.firstinspires.ftc.teamcode.components;

import static java.lang.Math.max;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final DcMotor intakeMotor;
    private volatile double PowerTarget;
    private double time;
    private final double DEFAULT_TIME = 3;

    Intake(HardwareMap hardwareMap) {
        this.intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        this.intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        this.time = 0;
        this.PowerTarget = 0;
    }

    public void speedUp(int time){
        PowerTarget = 1;
        if(time > 0) {
            this.time = time;
        }else{
            this.time = Double.POSITIVE_INFINITY;
        }
    }

    public void speedUp(int time, double power){
        PowerTarget = 1 * power;
        if(time > 0) {
            this.time = time;
        }else{
            this.time = Double.POSITIVE_INFINITY;
        }
    }

    public void speedUp(){
        PowerTarget = 1;
        this.time = DEFAULT_TIME;
    }


    public void speedUp(double power){
        PowerTarget = 1 * power;
        this.time = DEFAULT_TIME;
    }

    public void stop(){
        PowerTarget = 0;
        this.time = 0;
    }

    public void update(){
        long lastUpdate = System.currentTimeMillis();
        if(this.time > 0 && PowerTarget > 0) {
            this.intakeMotor.setPower(PowerTarget);
            this.time = max(this.time - (System.currentTimeMillis() - lastUpdate) * 1000, 0);
        }else if( this.intakeMotor.getPower() != 0){
            this.intakeMotor.setPower(0);
        }
    }
}
