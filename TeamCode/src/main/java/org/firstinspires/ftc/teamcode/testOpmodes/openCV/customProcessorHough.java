package org.firstinspires.ftc.teamcode.testOpmodes.openCV;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Hough", group = "Test")
public class customProcessorHough extends LinearOpMode {

    private VisionPortal      visionPortal;
    private ROIProcessorHough processor;

    @Override
    public void runOpMode() {

        processor = new ROIProcessorHough();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480)) // no need for 1920x1080 — processor downscales internally
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(processor)
                .enableLiveView(true)
                .build();

        // Wait for camera to actually be streaming before init'ing
        while (!isStarted() && !isStopRequested()) {
            if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                telemetry.addLine("Camera READY — waiting for start");
            } else {
                telemetry.addLine("Camera initializing...");
            }
            telemetry.update();
            sleep(50);
        }

        waitForStart();

        // Per-zone sorted lists (reused each loop)
        List<Point> z1Purple = new ArrayList<>();
        List<Point> z1Green  = new ArrayList<>();
        List<Point> z2Purple = new ArrayList<>();
        List<Point> z2Green  = new ArrayList<>();
        List<Point> z3Purple = new ArrayList<>();
        List<Point> z3Green  = new ArrayList<>();

        while (opModeIsActive()) {

            // Grab snapshots once per loop (avoids repeated synchronized calls)
            List<float[]> purpleCircles = processor.getPurpleCircles();
            List<float[]> greenCircles  = processor.getGreenCircles();

            // Clear zone buckets
            z1Purple.clear(); z1Green.clear();
            z2Purple.clear(); z2Green.clear();
            z3Purple.clear(); z3Green.clear();

            // Sort purple detections into zones
            for (float[] c : purpleCircles) {
                Point p   = new Point(c[0], c[1]);
                int   roi = processor.getROIIndexForPoint(p);
                // ROI zones 1–6 map left→right across the frame.
                // Zones 1–2 = Left, 3–4 = Center, 5–6 = Right
                if      (roi == 1 || roi == 2) z1Purple.add(p);
                else if (roi == 3 || roi == 4) z2Purple.add(p);
                else if (roi == 5 || roi == 6) z3Purple.add(p);
            }

            // Sort green detections into zones
            for (float[] c : greenCircles) {
                Point p   = new Point(c[0], c[1]);
                int   roi = processor.getROIIndexForPoint(p);
                if      (roi == 1 || roi == 2) z1Green.add(p);
                else if (roi == 3 || roi == 4) z2Green.add(p);
                else if (roi == 5 || roi == 6) z3Green.add(p);
            }

            // ── Telemetry ────────────────────────────────────────────────
            telemetry.addLine("=== ZONE 1  (Left) ===");
            reportZone(z1Purple, z1Green);

            telemetry.addLine("=== ZONE 2  (Center) ===");
            reportZone(z2Purple, z2Green);

            telemetry.addLine("=== ZONE 3  (Right) ===");
            reportZone(z3Purple, z3Green);

            // Determine most populated zone by total circle count
            int z1Count = z1Purple.size() + z1Green.size();
            int z2Count = z2Purple.size() + z2Green.size();
            int z3Count = z3Purple.size() + z3Green.size();
            int maxCount = Math.max(z1Count, Math.max(z2Count, z3Count));

            String mostPopulated = "None detected";
            if (maxCount > 0) {
                if      (z1Count == maxCount) mostPopulated = "Zone 1 — Left";
                else if (z2Count == maxCount) mostPopulated = "Zone 2 — Center";
                else                          mostPopulated = "Zone 3 — Right";
            }

            telemetry.addLine("────────────────────────");
            telemetry.addData("Most Populated Zone",  mostPopulated);
            telemetry.addData("Purple circles total", purpleCircles.size());
            telemetry.addData("Green  circles total", greenCircles.size());
            telemetry.addData("Camera state",         visionPortal.getCameraState());
            telemetry.update();

            sleep(30);
        }

        visionPortal.close();
    }

    private void reportZone(List<Point> purple, List<Point> green) {
        if (purple.isEmpty() && green.isEmpty()) {
            telemetry.addLine("  (none)");
            return;
        }
        for (Point p : purple)
            telemetry.addLine(String.format("  PURPLE @ (%.0f, %.0f)", p.x, p.y));
        for (Point p : green)
            telemetry.addLine(String.format("  GREEN  @ (%.0f, %.0f)", p.x, p.y));
    }
}