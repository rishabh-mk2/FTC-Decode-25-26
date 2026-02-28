package org.firstinspires.ftc.teamcode.testOpmodes.randomStuff;

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

    private VisionPortal visionPortal;
    private uselessHoughShit processor;

    @Override
    public void runOpMode() {

        processor = new uselessHoughShit();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(processor)
                .enableLiveView(true)
                .build();

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

        List<Point> z1Purple = new ArrayList<>(); List<Point> z1Green = new ArrayList<>();
        List<Point> z2Purple = new ArrayList<>(); List<Point> z2Green = new ArrayList<>();
        List<Point> z3Purple = new ArrayList<>(); List<Point> z3Green = new ArrayList<>();
        List<Point> z4Purple = new ArrayList<>(); List<Point> z4Green = new ArrayList<>();
        List<Point> z5Purple = new ArrayList<>(); List<Point> z5Green = new ArrayList<>();
        List<Point> z6Purple = new ArrayList<>(); List<Point> z6Green = new ArrayList<>();

        while (opModeIsActive()) {

            List<float[]> purpleCircles = processor.getPurpleCircles();
            List<float[]> greenCircles  = processor.getGreenCircles();

            z1Purple.clear(); z1Green.clear();
            z2Purple.clear(); z2Green.clear();
            z3Purple.clear(); z3Green.clear();
            z4Purple.clear(); z4Green.clear();
            z5Purple.clear(); z5Green.clear();
            z6Purple.clear(); z6Green.clear();

            for (float[] c : purpleCircles) {
                Point p   = new Point(c[0], c[1]);
                int   roi = processor.getROIIndexForPoint(p);
                switch (roi) {
                    case 1: z1Purple.add(p); break;
                    case 2: z2Purple.add(p); break;
                    case 3: z3Purple.add(p); break;
                    case 4: z4Purple.add(p); break;
                    case 5: z5Purple.add(p); break;
                    case 6: z6Purple.add(p); break;
                }
            }

            for (float[] c : greenCircles) {
                Point p   = new Point(c[0], c[1]);
                int   roi = processor.getROIIndexForPoint(p);
                switch (roi) {
                    case 1: z1Green.add(p); break;
                    case 2: z2Green.add(p); break;
                    case 3: z3Green.add(p); break;
                    case 4: z4Green.add(p); break;
                    case 5: z5Green.add(p); break;
                    case 6: z6Green.add(p); break;
                }
            }
            telemetry.addLine("=== ROI 1 ===");
            reportZone(z1Purple, z1Green);

            telemetry.addLine("=== ROI 2 ===");
            reportZone(z2Purple, z2Green);

            telemetry.addLine("=== ROI 3 ===");
            reportZone(z3Purple, z3Green);

            telemetry.addLine("=== ROI 4 ===");
            reportZone(z4Purple, z4Green);

            telemetry.addLine("=== ROI 5 ===");
            reportZone(z5Purple, z5Green);

            telemetry.addLine("=== ROI 6 ===");
            reportZone(z6Purple, z6Green);


            int[] counts = {
                    z1Purple.size() + z1Green.size(),
                    z2Purple.size() + z2Green.size(),
                    z3Purple.size() + z3Green.size(),
                    z4Purple.size() + z4Green.size(),
                    z5Purple.size() + z5Green.size(),
                    z6Purple.size() + z6Green.size(),
            };

            int maxCount  = 0;
            int maxZone   = 0;
            for (int i = 0; i < counts.length; i++) {
                if (counts[i] > maxCount) {
                    maxCount = counts[i];
                    maxZone  = i + 1;
                }
            }

            telemetry.addLine("===========================");
            telemetry.addData("Most Populated ROI", maxCount > 0 ? "ROI " + maxZone : "None");
            telemetry.addData("Purple circles total", purpleCircles.size());
            telemetry.addData("Green  circles total", greenCircles.size());
            telemetry.addData("Camera state", visionPortal.getCameraState());
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