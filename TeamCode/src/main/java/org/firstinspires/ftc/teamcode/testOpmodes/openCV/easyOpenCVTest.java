package org.firstinspires.ftc.teamcode.testOpmodes.openCV;

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

@TeleOp(name = "Oriented Ball Splitter", group = "TeleOp")
public class easyOpenCVTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(10);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        ColorBlobLocatorProcessor purpleColorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(5)
                .setErodeSize(0)
                .setDilateSize(4)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.OPENING)
                .build();

        ColorBlobLocatorProcessor greenColorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(5)
                .setErodeSize(1)
                .setDilateSize(1)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.OPENING)
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(purpleColorLocator)
                .addProcessor(greenColorLocator)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1920, 1200))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        waitForStart();

        while (opModeIsActive()) {
            List<ColorBlobLocatorProcessor.Blob> purpleBlobs = purpleColorLocator.getBlobs();
            List<ColorBlobLocatorProcessor.Blob> greenBlobs = greenColorLocator.getBlobs();

            ColorBlobLocatorProcessor.Util.filterByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 4200, 100000, purpleBlobs);
            ColorBlobLocatorProcessor.Util.filterByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 4200, 100000, greenBlobs);

            List<Circle> purpleCircles = splitByOrientation(purpleBlobs);
            List<Circle> greenCircles = splitByOrientation(greenBlobs);

            telemetry.addLine("=== ORIENTED TRACKER ===");
            telemetry.addData("Purple", purpleCircles.size());
            telemetry.addData("Green",  greenCircles.size());

            for (Circle c : purpleCircles) {
                telemetry.addLine(String.format("P: (%3.0f, %3.0f)", c.getX(), c.getY()));
            }
            for (Circle c : greenCircles) {
                telemetry.addLine(String.format("G: (%3.0f, %3.0f)", c.getX(), c.getY()));
            }

            telemetry.update();
            sleep(15);
        }
        portal.close();
    }

    private List<Circle> splitByOrientation(List<ColorBlobLocatorProcessor.Blob> blobs) {
        List<Circle> resultCircles = new ArrayList<>();

        for (ColorBlobLocatorProcessor.Blob b : blobs) {
            if (b.getCircularity() > 0.5) {
                resultCircles.add(b.getCircle());
                continue;
            }
            double cx = b.getBoxFit().center.x;
            double cy = b.getBoxFit().center.y;
            double width = b.getBoxFit().size.width;
            double height = b.getBoxFit().size.height;
            double angleDeg = b.getBoxFit().angle;

            double longSide = Math.max(width, height);
            double shortSide = Math.min(width, height);
            int n = (int) Math.round(longSide / shortSide);

            if (n <= 1) {
                resultCircles.add(b.getCircle());
            } else {
                float radius = (float) (shortSide / 2.0);

                if (width < height) {
                    angleDeg += 90;
                }
                double angleRad = Math.toRadians(angleDeg);

                double ux = Math.cos(angleRad);
                double uy = Math.sin(angleRad);

                for (int i = 0; i < n; i++) {
                    double shift = (i - (n - 1) / 2.0) * (longSide / n);
                    double finalX = cx + shift * ux;
                    double finalY = cy + shift * uy;
                    resultCircles.add(new Circle((float) finalX, (float) finalY, radius));
                }
            }
        }
        return resultCircles;
    }
}