package org.firstinspires.ftc.teamcode.utils;


public class Throttler {
    private Long lastRunNanos;
    private long delayMillis;

    public Throttler(long delayMillis) {
        this.delayMillis = delayMillis;
    }

    public long getDelayMillis() {
        return delayMillis;
    }

    public void setDelayMillis(long delayMillis) {
        this.delayMillis = delayMillis;
    }

    public boolean allow() {
        if (lastRunNanos == null) {
            lastRunNanos = System.nanoTime();
            return true;
        }
        long currentTimeNanos = System.nanoTime();
        if (currentTimeNanos - lastRunNanos >= delayMillis * 1000000) {
            lastRunNanos = currentTimeNanos;
            return true;
        } else {
            return false;
        }
    }
}
