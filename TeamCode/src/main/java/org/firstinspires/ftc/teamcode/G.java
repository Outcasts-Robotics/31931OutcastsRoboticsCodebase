package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Global variables.
 */
public abstract class G {
    public static Alliance alliance;

    public static POIs pois;

    public static class POIs {
        public Pose2D goal;
        public Pose2D obelisk;
        public Pose2D gate;
        public Pose2D baseZone;
        public Pose2D loadingZone;
        public Pose2D launchNear;
        public Pose2D launchMid;
        public Pose2D launchFar;
    }

    private static final POIs _red = new POIs();
    {{
        _red.goal = new Pose2D(DistanceUnit.CM, -182.875, 182.875, AngleUnit.DEGREES, 0);
    }}
    private static final POIs _blue = new POIs();
    {{
        _blue.goal = new Pose2D(DistanceUnit.CM, -182.875, -182.875, AngleUnit.DEGREES, 0);
    }}

    public static void setup(Alliance alliance) {
        G.alliance = alliance;
        if (alliance == Alliance.RED) {
            pois = _red;
        } else {
            pois = _blue;
        }
    }
}
