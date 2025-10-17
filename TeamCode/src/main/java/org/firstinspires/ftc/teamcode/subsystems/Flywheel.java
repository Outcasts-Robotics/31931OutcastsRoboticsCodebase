package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import androidx.core.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.PIDController;

public class Flywheel {
    private final DcMotor flywheelMotor;
//    private final PIDController flywheelVelPID;
      private final ElapsedTime timer = new ElapsedTime();
//
//    // --- Motor constants for goBILDA 5203 6000 RPM ---
      private static final double MAX_RPM = 6000.0;
//    private static final double TICKS_PER_REV = 28.0; // 5203 encoder CPR
//    private static final double SECONDS_PER_MIN = 60.0;
//
//    // --- PID constants (tune later) ---
//    private static final double kP = 0.1;
//    private static final double kI = 0.00005;
//    private static final double kD = 0.0001;

    // --- State ---
    private double targetRPM = 0;
    private double targetPower = 0.3;
    private double currentRPM = 0;
    private double currentPower = 0;
    private double lastPosition = 0;
    private double lastTime = 0;
    private boolean isRunning = false;

    public Flywheel(HardwareMap hardwareMap){
        this.flywheelMotor = hardwareMap.get(DcMotor.class, "flywheelMotor");
        this.flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //his.flywheelVelPID = new PIDController(kP, kI, kD, timer);
        //timer.reset();
    }

    /** Start the flywheel using the current target settings */
    public void startMotor(){
        this.isRunning = true;
        timer.reset();
        lastPosition = flywheelMotor.getCurrentPosition();
        lastTime = 0;
        flywheelMotor.setPower(targetPower);

    }

    /** Stop the flywheel */
    public void stopMotor(){
        isRunning = false;
        flywheelMotor.setPower(0);
        currentPower = 0;
        targetPower = 0;
        targetRPM = 0;
    }

    /** Set desired target RPM directly */
    public void setTargetRPM(double rpm){
        this.targetRPM = MathUtils.clamp(rpm, 0, MAX_RPM);
        this.targetPower = targetRPM / MAX_RPM;
    }

    /** Set desired power (0–1) and translate to equivalent RPM */
    public void setTargetPower(double power){
        this.targetPower = MathUtils.clamp(power, -1, 1);
        this.targetRPM = this.targetPower * MAX_RPM;
    }




    /** Call once per control loop */
//    public void updatePID(){
//        if(!isRunning) return;
//
//        // --- Measure current velocity ---
//        double currentPosition = flywheelMotor.getCurrentPosition();
//        double currentTime = timer.seconds();
//
//        double deltaPos = currentPosition - lastPosition;
//        double deltaTime = currentTime - lastTime;
//
//        if (deltaTime > 0) {
//            // Convert ticks/sec → RPM
//            currentRPM = (deltaPos / TICKS_PER_REV) / deltaTime * SECONDS_PER_MIN;
//        }
//
//        // --- PID Control ---
//        // Normalize speeds (0–1 range)
//        double pidOutput = flywheelVelPID.calculate(currentRPM / MAX_RPM, targetRPM / MAX_RPM);
//
//        // Adjust motor power
//        currentPower += pidOutput;
//        currentPower = MathUtils.clamp(currentPower, 0.0, 1.0);
//        flywheelMotor.setPower(currentPower);
//
//
//        // Update for next iteration
//        lastPosition = currentPosition;
//        lastTime = currentTime;
//
//    }

    // --- Telemetry Getters ---
    public double getCurrentRPM(){ return currentRPM; }
    public double getTargetRPM(){ return targetRPM; }
    public double getTargetPower(){ return targetPower; }
    public double getCurrentPower(){ return currentPower; }
    public boolean isRunning(){ return isRunning; }
}
