package org.firstinspires.ftc.teamcode.helpers;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

public class LoopTimer {
    private final ElapsedTime timer = new ElapsedTime();
    private final TelemetryManager telemetry;
    private final int trackLastNAverage;
    private final double[] loopTimes;
    private int loopTimesIndex = 0;
    private double loopTimesSum;


    public LoopTimer(TelemetryManager telemetry, int trackLastNAverage) {
        this.telemetry = telemetry;
        this.trackLastNAverage = trackLastNAverage;
        this.loopTimes = new double[trackLastNAverage];
    }

    public void reset() {
        timer.reset();
        loopTimesSum = 0;
        loopTimesIndex = 0;
        Arrays.fill(loopTimes, 0);
    }

    public void update() {
        double ms = timer.milliseconds();
        timer.reset();
        loopTimesSum = loopTimesSum - loopTimes[loopTimesIndex] + ms;
        loopTimes[loopTimesIndex] = ms;
        loopTimesIndex = (loopTimesIndex + 1) % trackLastNAverage;
        telemetry.addData("Loop Time", ms);
        telemetry.addData("Loop Time Avg", loopTimesSum / trackLastNAverage);
    }
}
