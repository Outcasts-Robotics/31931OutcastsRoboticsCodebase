package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.LauncherConstants;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
@Configurable
public class PIDController implements Runnable {

    private final DoubleSupplier currentStateSupplier;
    private final DoubleConsumer powerSetter;
    private final TelemetryManager panelsTelemetry;
    double errorSum = 0;

    private volatile boolean running;
    private double target;
    private volatile boolean stopped;

    private final ElapsedTime timer = new ElapsedTime();

    public PIDController(DoubleSupplier currentStateSupplier,
                         DoubleConsumer powerSetter, TelemetryManager panelsTelemetry) {
        this.currentStateSupplier = currentStateSupplier;
        this.powerSetter = powerSetter;
        stopped = true;
        this.panelsTelemetry = panelsTelemetry;
    }

    public void start() {
        new Thread(this).start();
    }

    @Override
    public void run() {
        timer.reset();
        running = true;
        stopped = false;

        double lastError = 0;
        while (running) {
            if(target == 0) {
                powerSetter.accept(0);
                try {
                    Thread.sleep(LauncherConstants.zeroWaitTimeMs);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                continue;
            }
            double value = currentStateSupplier.getAsDouble();
            double error = target - value;
            double power = LauncherConstants.kP * error + LauncherConstants.kI * errorSum + LauncherConstants.kD * (error - lastError) + LauncherConstants.kF;
            lastError = error;
            errorSum += error;

            if (timer.milliseconds() > 50) {
                panelsTelemetry.addData("Current RPM", value);
                panelsTelemetry.addData("Target RPM", this.target);
                timer.reset();
            }
            try {
                Thread.sleep(LauncherConstants.waitTimeMs);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            powerSetter.accept(power);
        }
        stopped = true;
    }

    public void stop() {
        running = false;
        while (!stopped) {
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    public void setTarget(double target) {
        this.target = target;
        errorSum = 0;
    }
}
