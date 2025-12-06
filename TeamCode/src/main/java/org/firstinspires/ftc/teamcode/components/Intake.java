package org.firstinspires.ftc.teamcode.components;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public DcMotorEx intakeMotor;
    public final double MOTOR_POWER = 1.0;
    public final int MOTOR_RPM = 1150;
    public final double ENCODER_RESOLUTION = 145.1;


    public Intake(HardwareMap hardwareMap, String intakeName){
           this.intakeMotor = hardwareMap.get(DcMotorEx.class, intakeName);
           this.intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           this.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
           this.intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void spinUpMax(){
        intakeMotor.setPower(MOTOR_POWER);
    }

    public void stop(){
        intakeMotor.setPower(0);

    }

    public void spinUptoPower(double power){
        intakeMotor.setPower(Math.min(Math.max(power, -1), 1));
    }

    public double getCommandedPower(){
        return intakeMotor.getPower();
    }

    public double getRPM(){
        return (intakeMotor.getVelocity() / ENCODER_RESOLUTION) * 60;
    }

    public boolean isRevvedUp() {
        double power = getCommandedPower();
        double expected = MOTOR_RPM * power;
        double actual = getRPM();

        if (Math.abs(power) < 0.1) return true;

        return Math.abs(actual - expected) <= Math.abs(expected) * 0.05;
    }

}