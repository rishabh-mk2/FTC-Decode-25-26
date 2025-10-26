package org.firstinspires.ftc.teamcode;

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

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "WebCam Circle Detection updted", group = "TeleOp")
public class easyOpenCVTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        telemetry.setMsTransmissionInterval(100);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setAutoClear(true);


        // --- PURPLE PROCESSOR ---
        ColorBlobLocatorProcessor purpleColorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setDrawContours(true)
                .setBoxFitColor(1)
                .setBlurSize(5)
                .setErodeSize(15)
                .setDilateSize(15)
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
                .setErodeSize(15)
                .setDilateSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        // --- VISION PORTAL ---
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(purpleColorLocator)
                .addProcessor(greenColorLocator)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        portal.setProcessorEnabled(purpleColorLocator, true);
        portal.setProcessorEnabled(greenColorLocator, true);

        waitForStart();

        while (opModeIsActive()) {

            // Clear previous detections
            List<Circle> detectedPurpleCircles = new ArrayList<>();
            List<Circle> detectedGreenCircles = new ArrayList<>();

            detectedPurpleCircles = processpurple(purpleColorLocator.getBlobs());
            detectedGreenCircles = processgreen(greenColorLocator.getBlobs());

            //List<ColorBlobLocatorProcessor.Blob> detectedPurpleCircles=purpleColorLocator.getBlobs();
            //List<ColorBlobLocatorProcessor.Blob> detectedGreenCircles=greenColorLocator.getBlobs();
            // --- TELEMETRY OUTPUT ---

            telemetry.addLine("=== PURPLE CIRCLES ===");
            for (Circle c : detectedPurpleCircles) {
                    telemetry.addData("Center (X,Y)", "(%.1f, %.1f)", c.getX(), c.getY());
                    telemetry.addData("Radius", "%.2f", c.getRadius());
                    double distance = -0.000001152*Math.pow(c.getY(),3) + 0.0014257 * Math.pow(c.getY(), 2) - 0.6473 * c.getY() + 127.58;
                    telemetry.addData("distance: ",distance);
            }

            telemetry.addLine("=== GREEN CIRCLES ===");
            for (Circle c : detectedGreenCircles) {
                    telemetry.addData("Center (X,Y)", "(%.1f, %.1f)", c.getX(), c.getY());
                    telemetry.addData("Radius", "%.2f", c.getRadius());
            }

            telemetry.update();
            sleep(50);
        }

        portal.close();
    }

    // --- HELPER FUNCTION TO PROCESS BLOBS ---
    private List<Circle> processpurple(List<ColorBlobLocatorProcessor.Blob> blobs) {
        List<Circle> circles = new ArrayList<>();

        for (ColorBlobLocatorProcessor.Blob b : blobs) {
            float cx = (float)b.getCircle().getX();
            float cy = (float)b.getCircle().getY();
            float r = (float)b.getCircle().getRadius();

            if (r < 1f) continue;
            if(1f <r && r <200f){
                circles.add(new Circle(cx, cy, r));
                continue;}

            if (b.getCircularity() > 0.8) {
                // Single circle
                circles.add(new Circle(cx, cy, r));
            } else {
                // Approximate overlapping circles: split horizontally
                float splitOffset = r * 0.5f; // adjust spacing as needed
                float splitRadius = r * 0.8f;

                circles.add(new Circle(cx - splitOffset, cy +splitOffset, splitRadius));
                circles.add(new Circle(cx + splitOffset, cy - splitOffset, splitRadius));
            }
        }

        return circles;
    }
    private List<Circle> processgreen(List<ColorBlobLocatorProcessor.Blob> blobs) {
        List<Circle> circles = new ArrayList<>();

        for (ColorBlobLocatorProcessor.Blob b : blobs) {
            float cx = (float)b.getCircle().getX();
            float cy = (float)b.getCircle().getY();
            float r = (float)b.getCircle().getRadius();

            if (r < 30f) continue;
            if(30f <r && r <60f){
                circles.add(new Circle(cx, cy, r));
                continue;}

            if (b.getCircularity() > 0.8) {
                // Single circle
                circles.add(new Circle(cx, cy, r));
            } else {
                // Approximate overlapping circles: split horizontally
                float splitOffset = r * 0.5f; // adjust spacing as needed
                float splitRadius = r * 0.8f;

                circles.add(new Circle(cx - splitOffset, cy -splitOffset, splitRadius));
                circles.add(new Circle(cx + splitOffset, cy + splitOffset, splitRadius));
            }
        }

        return circles;
    }

}
