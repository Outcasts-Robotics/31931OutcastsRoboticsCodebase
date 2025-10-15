package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A manual PID controller for controlling anything that requires a input and provide feedback.
 */
public class PIDController {
    private double kp;
    private double ki;
    private double kd;
    private double integralError;
    private double lastError;
    private ElapsedTime timer;

    public PIDController(double kp, double ki, double kd, ElapsedTime timer) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.timer = timer;
    }

    public void reset() {
        integralError = 0;
        lastError = 0;
        timer.reset();
    }

    public void setKp(double kp) {
        this.kp = kp;
    }

    public double getKp() {
        return kp;
    }

    public void setKi(double ki) {
        this.ki = ki;
    }

    public double getKi() {
        return ki;
    }

    public void setKd(double kd) {
        this.kd = kd;
    }

    public double getKd() {
        return kd;
    }

    /**
     * @param value current value
     * @param target target value
     * @return calculated PID output
     */
    public double calculate(double value, double target) {
        if (timer.seconds() > 5) {  // if lagged too long, integral term will be too large
            reset();
        }
        double dt = timer.seconds();

        double et = target - value;
        double p = kp * et;
        integralError += et * dt;
        double i = ki * integralError;
        double d = Math.abs(dt) < 1e-6 ? 0 : kd * (et - lastError) / dt;
        lastError = et;
        timer.reset();
        return p + i + d;
    }

    public void log(Telemetry telemetry, String caption) {
        telemetry.addData(caption, "Kp=%.2f; Ki=%.2f; Kd=%.2f", kp, ki, kd);
    }
}
