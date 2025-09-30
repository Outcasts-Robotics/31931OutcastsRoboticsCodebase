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

/**
 * Manages a webcam and AprilTag processing for robot vision.
 * Handles camera initialization, exposure/gain control, detection filtering,
 * and provides the last known relevant AprilTag detection.
 */
public class WebcamProcessor {

    public static class Inputs {
        public Position cameraPosition;
        public YawPitchRollAngles cameraOrientation;
        public int detectionMaxAgeMs;  // this should be higher than camera refresh time
        public boolean telemetryDetails;
        public boolean detectAllTags;  // non competition use
    }

    // Camera and robot configuration
    private final Position cameraPosition;
    private final YawPitchRollAngles cameraOrientation;
    private final int detectionMaxAgeMs;
    private final boolean telemetryDetails;

    // Hardware and FTC SDK components
    private final WebcamName webcam;
    private final Telemetry telemetry;
    private final boolean detectAllTags;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    // Camera control parameters (cached after initialization)
    int minExposure;
    int maxExposure;
    int stepExposure;
    int minGain;
    int maxGain;
    int stepGain;

    /**
     * Stores the most recently processed, relevant AprilTag detection.
     */
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
     * Enables or disables the AprilTag processor.
     * Call this to save CPU resources when AprilTag detection is not needed.
     *
     * @param enabled True to enable processing, false to disable.
     */
    public void setProcessing(boolean enabled) {
        visionPortal.setProcessorEnabled(aprilTagProcessor, enabled);
    }

    /**
     * Main processing loop. Fetches fresh AprilTag detections, filters for the most relevant one
     * (closest, valid pose, and matching ID criteria), updates {@link #lastDetection},
     * and logs telemetry.
     */
    public void loopUpdate() {
        List<AprilTagDetection> freshDetections = aprilTagProcessor.getFreshDetections();
        if (freshDetections != null) {
            Optional<AprilTagDetection> minRangeTag = freshDetections.stream()
                    .filter(detection ->
                            detectAllTags || Tags.isPositionTagId(detection.id))
                    .min((d1, d2) -> {
                        if (d1.id == d2.id) return 0;
                        if (d1.ftcPose == null && d2.ftcPose == null) return 0;
                        if (d1.ftcPose == null) return 1;
                        if (d2.ftcPose == null) return -1;
                        return Double.compare(d1.ftcPose.range, d2.ftcPose.range);
                    });
            if (minRangeTag.isPresent() && minRangeTag.get().ftcPose != null) {
                lastDetection = minRangeTag.get();
            } else {
                checkAndInvalidateStaleDetection();
            }
        } else {
            checkAndInvalidateStaleDetection();
        }
        logTelemetry();
    }

    private void checkAndInvalidateStaleDetection() {
        long age = detectionAgeInMs();
        if (age != -1 && age > detectionMaxAgeMs)
            lastDetection = null;
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
