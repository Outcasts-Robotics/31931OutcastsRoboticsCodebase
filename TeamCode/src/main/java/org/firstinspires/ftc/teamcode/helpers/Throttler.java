package org.firstinspires.ftc.teamcode.helpers;

/**
 * A utility class that helps in rate-limiting or throttling the execution of code.
 * It can be used to ensure that certain operations don't happen more frequently than a specified rate.
 */
public class Throttler {
    private volatile Long lastRunNanos;
    private volatile long delayMillis;

    /**
     * Creates a new Throttler with the specified delay between allowed operations.
     *
     * @param delayMillis The minimum delay between allowed operations in milliseconds
     * @throws IllegalArgumentException if delayMillis is negative
     */
    public Throttler(long delayMillis) {
        if (delayMillis < 0) {
            throw new IllegalArgumentException("Delay must be non-negative");
        }
        this.delayMillis = delayMillis;
    }

    /**
     * Gets the current delay between allowed operations.
     *
     * @return The delay in milliseconds
     */
    public long getDelayMillis() {
        return delayMillis;
    }

    /**
     * Sets the delay between allowed operations.
     *
     * @param delayMillis The new delay in milliseconds
     * @throws IllegalArgumentException if delayMillis is negative
     */
    public void setDelayMillis(long delayMillis) {
        if (delayMillis < 0) {
            throw new IllegalArgumentException("Delay must be non-negative");
        }
        this.delayMillis = delayMillis;
    }

    /**
     * Checks if an operation is allowed to proceed based on the throttling delay.
     *
     * @return true if the operation is allowed, false if it should be throttled
     */
    public boolean allow() {
        long now = System.nanoTime();
        if (lastRunNanos == null) {
            lastRunNanos = now;
            return true;
        }
        
        long elapsedNanos = now - lastRunNanos;
        if (elapsedNanos >= delayMillis * 1_000_000L) {
            lastRunNanos = now;
            return true;
        }
        return false;
    }
    
    /**
     * Gets the remaining time in milliseconds until the next operation would be allowed.
     *
     * @return The remaining time in milliseconds, or 0 if the operation would be allowed immediately
     */
    public long getRemainingMillis() {
        if (lastRunNanos == null) {
            return 0;
        }
        long elapsedNanos = System.nanoTime() - lastRunNanos;
        long remainingNanos = (delayMillis * 1_000_000L) - elapsedNanos;
        return Math.max(0, remainingNanos / 1_000_000L);
    }
    
    /**
     * Waits (blocks) until the next operation would be allowed based on the throttling delay.
     * If no operation has been performed yet, returns immediately.
     * 
     * @throws InterruptedException if the current thread is interrupted while waiting
     */
    public void waitUntilNext() throws InterruptedException {
        if (lastRunNanos == null) {
            lastRunNanos = System.nanoTime();
            return;
        }
        
        long remainingNanos = (delayMillis * 1_000_000L) - (System.nanoTime() - lastRunNanos);
        if (remainingNanos > 0) {
            // Convert to millis and add 1 to ensure we don't wait too short due to integer division
            long millis = (remainingNanos / 1_000_000) + 1;
            Thread.sleep(millis);
        }
        lastRunNanos = System.nanoTime();
    }
}
