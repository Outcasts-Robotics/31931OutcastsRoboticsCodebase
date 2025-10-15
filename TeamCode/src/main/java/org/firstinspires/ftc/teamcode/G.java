package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Global variables.
 */
public abstract class G {
    private G() {}

    public static Alliance alliance;

    public static POIs pois;

    private static final POIs _redPOIs;
    private static final POIs _bluePOIs;

    static {
        // Initialize red alliance POIs with default values (you may want to adjust these)
        _redPOIs = new POIs(
            new Pose2D(DistanceUnit.CM, -182.875, 182.875, AngleUnit.DEGREES, 0), // goal
            null, // obelisk
            null, // gate
            null, // baseZone
            null, // loadingZone
            null, // launchNear
            null, // launchMid
            null  // launchFar
        );
        
        // Initialize blue alliance POIs with default values (you may want to adjust these)
        _bluePOIs = new POIs(
            new Pose2D(DistanceUnit.CM, -182.875, -182.875, AngleUnit.DEGREES, 0), // goal
            null, // obelisk
            null, // gate
            null, // baseZone
            null, // loadingZone
            null, // launchNear
            null, // launchMid
            null  // launchFar
        );
    }

    public static void initGlobals(Alliance alliance) {
        G.alliance = alliance;
        if (alliance == Alliance.RED) {
            pois = _redPOIs;
        } else {
            pois = _bluePOIs;
        }
    }
}
