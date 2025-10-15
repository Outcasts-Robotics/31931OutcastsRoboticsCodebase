package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Point of interests on the field.
 */
public class POIs {
    public final Pose2D goal;
    public final Pose2D obelisk;
    public final Pose2D gate;
    public final Pose2D baseZone;
    public final Pose2D loadingZone;
    public final Pose2D launchNear;
    public final Pose2D launchMid;
    public final Pose2D launchFar;

    public POIs(Pose2D goal, Pose2D obelisk, Pose2D gate, Pose2D baseZone, Pose2D loadingZone, Pose2D launchNear, Pose2D launchMid, Pose2D launchFar) {
        this.goal = goal;
        this.obelisk = obelisk;
        this.gate = gate;
        this.baseZone = baseZone;
        this.loadingZone = loadingZone;
        this.launchNear = launchNear;
        this.launchMid = launchMid;
        this.launchFar = launchFar;
    }
}
