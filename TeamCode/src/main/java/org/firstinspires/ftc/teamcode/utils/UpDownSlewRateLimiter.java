package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *  Up rate = away from zero rate
 *  Down rate = towards zero rate
 */
public class UpDownSlewRateLimiter {
    private final double awayFromZeroRate;
    private final double towardsZeroRate;
    private boolean allowJumpToZero;
    private final ElapsedTime timer;
    private double previousValue;

    public UpDownSlewRateLimiter(double upRate, double downRate, boolean allowJumpToZero) {
        timer = new ElapsedTime();
        this.awayFromZeroRate = upRate;
        this.towardsZeroRate = downRate;
        this.allowJumpToZero = allowJumpToZero;
    }

    public double apply(double targetValue) {
        // handle allow direct jump to zero scenario
        if (targetValue == 0.0 && allowJumpToZero) {
            previousValue = 0.0;
            timer.reset();
            return 0.0;
        }

        double currentValue = previousValue;
        double deltaTime = timer.seconds();
        timer.reset(); // reset for next interval

        // prevent division by zero, or excessively small deltaTime issues
        if (deltaTime < 0.00001) deltaTime = 0.00001;

        double change = targetValue - currentValue;
        double absChange = Math.abs(change);

        // If no change is needed, return the current value
        if (absChange < 1e-9) {
            previousValue = targetValue;
            return targetValue;
        }

        double allowedChange; // max abs change allowed

        if (currentValue == 0.0) {
            // Starting from zero, any movement is increasing magnitude
            allowedChange = awayFromZeroRate * deltaTime;
        } else if (targetValue == 0.0) {
            // Moving towards zero (decreasing magnitude)
            allowedChange = towardsZeroRate * deltaTime;
        } else if (Math.signum(currentValue) == Math.signum(targetValue)) {
            // Moving in the same sign (not crossing zero)
            if (Math.abs(targetValue) > Math.abs(currentValue)) {
                // Increasing magnitude (moving away from zero)
                allowedChange = awayFromZeroRate * deltaTime;
            } else {
                // Decreasing magnitude (moving towards zero)
                allowedChange = towardsZeroRate * deltaTime;
            }
        } else {
            // crossing zero
            double timeToZero = Math.abs(currentValue) / towardsZeroRate;
            if (deltaTime <= timeToZero) {
                // not enough delta time to reach zero
                allowedChange = towardsZeroRate * deltaTime;
            } else {
                allowedChange = Math.abs(currentValue) + (deltaTime - timeToZero) * awayFromZeroRate;
            }
        }

        // Apply the sign of the desired change to the allowed magnitude of change
        double actualChange = Math.copySign(Math.min(absChange, allowedChange), change);
        previousValue = currentValue + actualChange;
        return previousValue;
    }

    public void setAllowJumpToZero(boolean allowJumpToZero) {
        this.allowJumpToZero = allowJumpToZero;
    }
}
