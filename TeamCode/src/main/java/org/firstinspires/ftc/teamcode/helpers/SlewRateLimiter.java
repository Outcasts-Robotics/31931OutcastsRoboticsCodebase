package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *
 */
public class SlewRateLimiter {
    private final double rate;
    private double previousValue;
    private final ElapsedTime timer;

    public SlewRateLimiter(double rate) {
        this(rate, new ElapsedTime());
    }

    public SlewRateLimiter(double rate, ElapsedTime timer) {
        this.rate = rate;
        this.previousValue = 0.0;
        this.timer = timer;
        this.timer.reset();
    }

    public void setPreviousValue(double previousValue) {
        this.previousValue = previousValue;
    }

    public double apply(double targetValue) {
        if (rate <= 0.0) return targetValue;
        double deltaTime = timer.seconds();
        timer.reset();
        if (deltaTime < 0.0001) deltaTime = 0.0001;
        double currentValue = previousValue;

        double change = targetValue - currentValue;
        double absChange = Math.abs(change);
        if (absChange > rate * deltaTime) {
            currentValue += rate * deltaTime * Math.signum(change);
        } else {
            currentValue = targetValue;
        }
        previousValue = currentValue;
        return currentValue;
    }

}
