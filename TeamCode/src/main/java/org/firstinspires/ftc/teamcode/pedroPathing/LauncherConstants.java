package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class LauncherConstants {
    public static volatile double kP = 0.006;
    public static volatile double kI = .00030000;
    public static volatile double kD = 0.001;
    public static volatile double kF = 0;

    public static volatile long waitTimeMs = 1;
    public static volatile long zeroWaitTimeMs = 10;
}
