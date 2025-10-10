package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

public class WebcamAprilTag {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    public WebcamAprilTag(HardwareMap hardwareMap, boolean useWebcam, String webcamName) {
        AprilTagLibrary.Builder tagLibraryBuilder = new AprilTagLibrary.Builder();
        tagLibraryBuilder.addTag(20, "Tag_20", 2, DistanceUnit.INCH);
        tagLibraryBuilder.addTag(21, "Tag_21", 2, DistanceUnit.INCH);
        tagLibraryBuilder.addTag(22, "Tag_22", 2, DistanceUnit.INCH);
        tagLibraryBuilder.addTag(23, "Tag_23", 2, DistanceUnit.INCH);
        tagLibraryBuilder.addTag(24, "Tag_24", 2, DistanceUnit.INCH);
        AprilTagLibrary customAprilTagLibrary = tagLibraryBuilder.build();

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(customAprilTagLibrary)
                .setNumThreads(4)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();
        aprilTagProcessor.setDecimation(1);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (useWebcam) {
            builder.setCamera(hardwareMap.get(WebcamName.class, webcamName));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.setCameraResolution(new Size(640, 480));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.enableLiveView(true);
        builder.setAutoStopLiveView(true);
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    public List<AprilTagDetection> getDetections() {
        return aprilTagProcessor.getDetections();
    }

    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}