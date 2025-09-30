package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Optional;
import java.util.concurrent.TimeUnit;

public class WebcamProcessor {

    public static class Inputs {
        public Position cameraPosition;
        public YawPitchRollAngles cameraOrientation;
        public int detectionMaxAgeMs;  // this should be higher than camera refresh time
        public boolean telemetryDetails;
        public boolean detectAllTags;  // non competition use
    }

    private final Position cameraPosition;
    private final YawPitchRollAngles cameraOrientation;
    private final int detectionMaxAgeMs;
    private final boolean telemetryDetails;

    private final WebcamName webcam;
    private final Telemetry telemetry;
    private final boolean detectAllTags;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    int minExposure;
    int maxExposure;
    int stepExposure;
    int minGain;
    int maxGain;
    int stepGain;

    private AprilTagDetection lastDetection;

    public WebcamProcessor(WebcamName webcam, Telemetry telemetry, Inputs inputs) {
        this.webcam = webcam;
        this.telemetry = telemetry;
        this.cameraPosition = inputs.cameraPosition;
        this.cameraOrientation = inputs.cameraOrientation;
        this.detectionMaxAgeMs = inputs.detectionMaxAgeMs;
        this.telemetryDetails = inputs.telemetryDetails;
        this.detectAllTags = inputs.detectAllTags;
    }

    public void initialize() {
        aprilTagProcessor = new AprilTagProcessor.Builder().setCameraPose(cameraPosition, cameraOrientation)
                .build();
        visionPortal = new VisionPortal.Builder().setCamera(webcam).addProcessor(aprilTagProcessor).build();

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting for camera to stream");
            telemetry.update();
            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        ExposureControl exposureControl = getExposureControl();
        // set exposure mode to manual
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            telemetry.addData("Exposure mode", "Manual");
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        minExposure = (int) exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
//            maxExposure = (int) exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);
        maxExposure = 100; // anything larger would be too laggy
        stepExposure = 10;
        exposureControl.setExposure(10, TimeUnit.MILLISECONDS);

        GainControl gainControl = getGainControl();
        minGain = gainControl.getMinGain();
        maxGain = gainControl.getMaxGain();
        gainControl.setGain(2);
        stepGain = 1;
    }

    private ExposureControl getExposureControl() {
        return visionPortal.getCameraControl(ExposureControl.class);
    }

    private GainControl getGainControl() {
        return visionPortal.getCameraControl(GainControl.class);
    }

    public void initLoopUpdate(Gamepad gamepad) {
        GainControl gainControl = getGainControl();
        if (gamepad.yWasPressed()) {
            int gain = Math.min(maxGain, gainControl.getGain() + stepGain);
            telemetry.addData("Gain", gain);
            gainControl.setGain(gain);
        }
        if (gamepad.aWasPressed()) {
            int gain = Math.max(minGain, gainControl.getGain() - stepGain);
            telemetry.addData("Gain", gain);
            gainControl.setGain(gain);
        }
        ExposureControl exposureControl = getExposureControl();
        if (gamepad.xWasPressed()) {
            long exposure = Math.max(minExposure, exposureControl.getExposure(TimeUnit.MILLISECONDS) - stepExposure);
            telemetry.addData("Exposure", exposure);
            exposureControl.setExposure(exposure, TimeUnit.MILLISECONDS);
        }
        if (gamepad.bWasPressed()) {
            long exposure = Math.min(maxExposure, exposureControl.getExposure(TimeUnit.MILLISECONDS) + stepExposure);
            telemetry.addData("Exposure", exposure);
            exposureControl.setExposure(exposure, TimeUnit.MILLISECONDS);
        }
        loopUpdate();
    }

    /**
     * Processing can be paused/resumed. Use if necessary to save CPU or bandwidth.
     */
    public void setProcessing(boolean enabled) {
        visionPortal.setProcessorEnabled(aprilTagProcessor, enabled);
    }

    public void loopUpdate() {
        List<AprilTagDetection> freshDetections = aprilTagProcessor.getFreshDetections();
        if (freshDetections != null) {
            Optional<AprilTagDetection> minRangeTag = freshDetections.stream()
                    .filter(detection ->
                            detectAllTags || Tags.isPositionTagId(detection.id))
                    .min((d1, d2) -> {
                        if (d1.id == d2.id) return 0;
                        if (d1.ftcPose == null) return 1;
                        if (d2.ftcPose == null) return -1;
                        if (d2.ftcPose.range == d1.ftcPose.range) return 0;
                        return d2.ftcPose.range > d1.ftcPose.range ? 1 : -1;
                    });
            if (minRangeTag.isPresent() && minRangeTag.get().ftcPose != null) {
                lastDetection = minRangeTag.get();
            }
        } else {
            long age = detectionAgeInMs();
            if (age != -1 && age > detectionMaxAgeMs)
                lastDetection = null;
        }
        logTelemetry();
    }

    private void logTelemetry() {
        String tag = "none(-1ms)";
        if (lastDetection != null) {
            long age = detectionAgeInMs();
            tag = lastDetection.metadata.name + "(" + age + "ms)";
        }
        telemetry.addData("Tag", tag);
        if (telemetryDetails) {
            if (lastDetection != null) {
                AprilTagPoseFtc pose = lastDetection.ftcPose;
                telemetry.addData("XYZ", "%.1f, %.1f, %.1f", pose.x, pose.y, pose.z);
                telemetry.addData("RPY", "%.1f, %.1f, %.1f", pose.roll, pose.pitch, pose.yaw);
                telemetry.addData("RBE", "%.1f, %.1f, %.1f", pose.range, pose.bearing, pose.elevation);
            }
        }
    }

    private long detectionAgeInMs() {
        if (lastDetection != null) {
            return (System.nanoTime() - lastDetection.frameAcquisitionNanoTime) / 1000000;
        }
        return -1;
    }

    public AprilTagDetection getLastDetection() {
        return lastDetection;
    }
}
