package org.firstinspires.ftc.teamcode.testOpmodes;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@TeleOp(name = "Circle Detection Webcam", group = "TeleOp")
//@Disabled
public class easyOpenCVTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{

        telemetry.setMsTransmissionInterval(100);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setAutoClear(true);

        //ImageRegion region1 = ImageRegion.asImageCoordinates(leftX, topY, rightX, bottomY); // LEFT MOST ROI
        //ImageRegion region2 = ImageRegion.asImageCoordinates(leftX, topY, rightX, bottomY);
        //ImageRegion region3 = ImageRegion.asImageCoordinates(leftX, topY, rightX, bottomY); // CENTER ROI
        //ImageRegion region4 = ImageRegion.asImageCoordinates(leftX, topY, rightX, bottomY);
        ImageRegion region5 = ImageRegion.asImageCoordinates(840, 130, 1260, 240); // RIGHT MOST ROI*/
        // --- PURPLE PROCESSOR ---
        ColorBlobLocatorProcessor purpleColorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
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
                .setRoi(ImageRegion.entireFrame())
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
                .setCameraResolution(new Size(1280, 960))
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
            // --- TELEMETRY OUTPUT ---

            /*telemetry.addLine("=== PURPLE CIRCLES ===");
            for (Circle c : detectedPurpleCircles) {
                    telemetry.addData("Center (X,Y)", "(%.1f, %.1f)", c.getX(), c.getY());
                    //telemetry.addData("Radius", "%.2f", c.getRadius());
                    //double distance = -0.000001152*Math.pow(c.getY(),3) + 0.0014257 * Math.pow(c.getY(), 2) - 0.6473 * c.getY() + 127.58;
                    //telemetry.addData("distance: ",distance);
            }*/

            telemetry.addLine("=== GREEN CIRCLES ===");
            for (Circle c : detectedGreenCircles) {
                    telemetry.addData("Center (X,Y)", "(%.1f, %.1f)", c.getX(), c.getY());
                    telemetry.addData("Radius", "%.2f", c.getRadius());
                    double distance = -0.000001152*Math.pow(c.getY(),3) + 0.0014257 * Math.pow(c.getY(), 2) - 0.6473 * c.getY() + 127.58;
                     telemetry.addData("distance: ",distance);
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
            double aspectRatio = b.getAspectRatio();

            if (r < 30f) continue;
            if(30f < r && r < 60f){
                circles.add(new Circle(cx, cy, r));
                continue;}

            if ((b.getAspectRatio() <= 1.1 ) || (b.getAspectRatio() > 1.1 && r < 76 && (cy > 420 || cy < 60)|| (b.getAspectRatio() > 1.1 && r < 76 && (cx < 40 || cx > 570)))){
                // Single circle
                circles.add(new Circle(cx, cy, r));
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
            double aspectRatio = b.getAspectRatio();

            if (r < 15f) continue;
            if(15f < r && r < 100f){
                circles.add(new Circle(cx, cy, r));
                continue;}

            if ((b.getAspectRatio() <= 2.5 ) || (b.getAspectRatio() > 1.1 && r < 76 && (cy > 420 || cy < 60)|| (b.getAspectRatio() > 1.1 && r < 76 && (cx < 40 || cx > 570)))){
                // Single circle
                circles.add(new Circle(cx, cy, r));
            }
        }

        return circles;
    }
    public double aspectRatio;
    public double circularity;
    private double testAspectRatio(List<ColorBlobLocatorProcessor.Blob> blobs) {
        for (ColorBlobLocatorProcessor.Blob b : blobs) {
            aspectRatio = b.getAspectRatio();
        };
        return aspectRatio;
    }
    private double testCircularity(List<ColorBlobLocatorProcessor.Blob> blobs) {
        for (ColorBlobLocatorProcessor.Blob b : blobs) {
            circularity = b.getCircularity();
        };
        return circularity;
    }

}
