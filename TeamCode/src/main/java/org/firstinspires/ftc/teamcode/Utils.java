package org.firstinspires.ftc.teamcode;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public abstract class Utils {
    private Utils() {
    }

    public static Pose toPedro(Pose pose) {
        if (pose.getCoordinateSystem() == PedroCoordinates.INSTANCE) {
            return pose;
        }
        Pose rotatedPose = pose.rotate(-Math.PI / 2, false);
        Pose result = rotatedPose.plus(new Pose(72, 72, 0, FTCCoordinates.INSTANCE));
        return new Pose(result.getX(), result.getY(), result.getHeading(), PedroCoordinates.INSTANCE);
    }

    public static Pose toPedro(Pose3D robotPose) {
        return toPedro(new Pose(robotPose.getPosition().x, robotPose.getPosition().y,
                robotPose.getOrientation().getYaw(AngleUnit.RADIANS), FTCCoordinates.INSTANCE));
    }
}
