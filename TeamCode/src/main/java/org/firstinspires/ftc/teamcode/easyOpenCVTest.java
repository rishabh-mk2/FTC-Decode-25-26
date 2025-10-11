package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

@TeleOp(name = "WebCam OPEN CV Test", group = "TeleOp")
public class easyOpenCVTest extends LinearOpMode {

    @Override
    public void runOpMode() {

         // Don't clear telemetry each loop
        telemetry.setMsTransmissionInterval(100);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        // --- PURPLE PROCESSOR ---
        ColorBlobLocatorProcessor purpleColorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setDrawContours(true)
                .setBoxFitColor(1)
                .setBlurSize(5)
                .setDilateSize(15)
                .setErodeSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        // --- GREEN PROCESSOR ---
        ColorBlobLocatorProcessor greenColorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setDrawContours(true)
                .setBoxFitColor(1)
                .setBlurSize(5)
                .setDilateSize(15)
                .setErodeSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        // --- BUILD VISION PORTAL ---
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(purpleColorLocator)
                .addProcessor(greenColorLocator)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480)) // Lower resolution = faster updates
                .build();

        // Make sure both processors stay enabled
        portal.setProcessorEnabled(purpleColorLocator, true);
        portal.setProcessorEnabled(greenColorLocator, true);

        telemetry.setAutoClear(true);

        waitForStart();

        while (opModeIsActive()) {

            // --- READ BLOBS ---
            List<ColorBlobLocatorProcessor.Blob> purpleBlobs = purpleColorLocator.getBlobs();
            List<ColorBlobLocatorProcessor.Blob> greenBlobs = greenColorLocator.getBlobs();

            // --- FILTER BLOBS ---
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                    50, 20000, purpleBlobs);
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                    0.3, 1, purpleBlobs);

            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                    50, 20000, greenBlobs);
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                    0.3, 1, greenBlobs);

            // --- TELEMETRY OUTPUT ---
            telemetry.addLine("=== PURPLE BLOBS ===");
            for (ColorBlobLocatorProcessor.Blob p : purpleBlobs) {
                Circle circle = p.getCircle();
                telemetry.addData("Circularity", "%.2f", p.getCircularity());
                telemetry.addData("Radius", "%.2f", circle.getRadius());
                telemetry.addData("Center (X,Y)", "(%.1f, %.1f)", circle.getX(), circle.getY());
                telemetry.addLine();
            }

            telemetry.addLine("=== GREEN BLOBS ===");
            for (ColorBlobLocatorProcessor.Blob g : greenBlobs) {
                Circle circle = g.getCircle();
                telemetry.addData("Circularity", "%.2f", g.getCircularity());
                telemetry.addData("Radius", "%.2f", circle.getRadius());
                telemetry.addData("Center (X,Y)", "(%.1f, %.1f)", circle.getX(), circle.getY());
                telemetry.addLine();
            }

            telemetry.update();

            // Small delay to sync with frame rate
            sleep(50);
        }

        // Stop vision portal cleanly when OpMode ends
        portal.close();
    }
}