package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utils;
import org.junit.Test;

public class PoseTest {
    @Test
    public void testToPose() {
        Pose pose = new Pose(0, 0, 0, PedroCoordinates.INSTANCE);
        Pose poseFtc = pose.getAsCoordinateSystem(FTCCoordinates.INSTANCE);
        System.out.println("FTC %f %f %f".formatted(poseFtc.getX(), poseFtc.getY(), AngleUnit.DEGREES.fromRadians(poseFtc.getHeading())));

        poseFtc = new Pose(0,0, 0, FTCCoordinates.INSTANCE);
        pose = poseFtc.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        System.out.println("Pedro %f %f %f".formatted(pose.getX(), pose.getY(), AngleUnit.DEGREES.fromRadians(pose.getHeading())));
    }

    @Test
    public void testUtilsConversion(){
        Pose pedro = Utils.toPedro(new Pose(0, 0, 0, FTCCoordinates.INSTANCE));
        System.out.println(pedro);
    }
}
