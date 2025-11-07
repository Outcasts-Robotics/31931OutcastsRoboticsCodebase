package org.firstinspires.ftc.teamcode.components;

import android.util.Size;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Manages a webcam and AprilTag processing for robot vision.
 * Handles camera initialization, exposure/gain control, detection filtering,
 * and provides the last known relevant AprilTag detection.
 */
public class WebcamProcessor {

    public static class Inputs {
        public String webcamName;
        public Position cameraPosition;
        public YawPitchRollAngles cameraOrientation;
        public boolean telemetryDetails;
        public boolean detectAllTags;  // non competition use
    }

    // Camera and robot configuration
    private final boolean telemetryDetails;

    // Hardware and FTC SDK components
    private final Telemetry telemetry;
    private final boolean detectAllTags;
    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTagProcessor;

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
    private AprilTagDetection detection;

    public WebcamProcessor(HardwareMap hardwareMap, Telemetry telemetry, Inputs inputs) {
        WebcamName webcam = hardwareMap.get(WebcamName.class, inputs.webcamName);
        this.telemetry = telemetry;
        this.telemetryDetails = inputs.telemetryDetails;
        this.detectAllTags = inputs.detectAllTags;
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setCameraPose(inputs.cameraPosition, inputs.cameraOrientation)
//          #2
//        Focals (pixels) - Fx: 690.141 Fy: 690.141
//        Optical center - Cx: 413.213 Cy: 288.749
//        Radial distortion (Brown's Model)
//        K1: 0.0552367 K2: -0.137876 K3: 0.106257
//        P1: 0.000903713 P2: 0.00158721
//        Skew: 0
//            #1
//        Focals (pixels) - Fx: 684.534 Fy: 684.534
//        Optical center - Cx: 403.737 Cy: 287.339
//        Radial distortion (Brown's Model)
//        K1: 0.0506626 K2: -0.0634906 K3: -0.0114408
//        P1: 0.000150679 P2: -0.000150837
//        Skew: 0
                .setLensIntrinsics(690.141, 690.141, 413.213, 288.749)
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(800, 600))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTagProcessor)
                .build();

    }

    public void initialize() {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("Waiting for camera to stream");
            telemetry.update();
            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
            telemetry.addLine("Camera ready");
            telemetry.update();
        }

        ExposureControl exposureControl = getExposureControl();
        // set exposure mode to manual
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            telemetry.addLine("Set exposure mode to Manual");
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        minExposure = (int) exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
        maxExposure = 100; // anything larger would be too laggy
        stepExposure = 10;
        exposureControl.setExposure(14, TimeUnit.MILLISECONDS);

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
        update();
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
     * (closest, valid pose, and matching ID criteria), updates {@link #detection},
     * and logs telemetry.
     */
    public void update() {
        detection = null;
        List<AprilTagDetection> freshDetections = aprilTagProcessor.getFreshDetections();
        if (freshDetections != null) {
            freshDetections.stream()
                    .filter(detection ->
                            detectAllTags || AprilTag.isPositionTagId(detection.id))
                    .filter(detection -> detection.ftcPose != null)
                    .min((d1, d2) -> {
                        if (d1.id == d2.id) return 0;
                        return Double.compare(d1.ftcPose.range, d2.ftcPose.range);
                    })
                    .ifPresent(d -> detection = d);
        }
        logTelemetry();
    }

    private void logTelemetry() {
        String tag = "none(-1ms)";
        if (detection != null) {
            long age = detectionAgeInMs();
            tag = detection.metadata.name + "(" + age + "ms)";
        }
        telemetry.addData("Tag", tag);
        Pose3D pose = detection.robotPose;
        Position position = pose.getPosition();
        YawPitchRollAngles orientation = pose.getOrientation();
        telemetry.addData("Position", String.format("X:%.1f Y:%.1f Z:%.1f", position.x, position.y, position.z));
        telemetry.addData("Orientation", String.format("X:%.1f Y:%.1f Z:%.1f", orientation.getRoll(), orientation.getPitch(), orientation.getYaw(AngleUnit.DEGREES)));
    }

    private long detectionAgeInMs() {
        if (detection != null) {
            return (System.nanoTime() - detection.frameAcquisitionNanoTime) / 1000000;
        }
        return -1;
    }

    public AprilTagDetection getDetection() {
        return detection;
    }
}
