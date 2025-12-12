package org.firstinspires.ftc.teamcode.components;


import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class Vision {

    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTagProcessor;
    private final CameraName camera;

    private Tag positioningTag;
    private Pose3D robotPoseFtc;
    private Tag motifTag;

    public Vision(HardwareMap hardwareMap) {
        camera = hardwareMap.get(CameraName.class, "camera");
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setCameraPose(new Position(DistanceUnit.INCH, 5, 5, 5, 0),
                        new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, 0))
                // .setLensIntrinsics(1,1,1,1)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(camera)
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(800, 600))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
    }

    public void update() {
        ArrayList<AprilTagDetection> detections = aprilTagProcessor.getFreshDetections();
        positioningTag = null;
        robotPoseFtc = null;
        for (AprilTagDetection detection : detections) {
            Tag tag = Tag.of(detection.id);
            if (tag != null) {
                if (tag.isGoal) {
                    positioningTag = tag;
                    robotPoseFtc = detection.robotPose; // pose of the robot wrt field
                } else {  // is motif tag
                    if (motifTag == null) {
                        motifTag = tag;
                    }
                }
            }
            if (positioningTag != null && motifTag != null) {
                break;
            }
        }
    }

    public void stop() {
        visionPortal.stopStreaming();
    }

    public Tag getPositioningTag() {
        return positioningTag;
    }

    public Pose3D getRobotPoseFtc() {
        return robotPoseFtc;
    }

    public  Tag getMotifTag(){
        return motifTag;
    }
}
