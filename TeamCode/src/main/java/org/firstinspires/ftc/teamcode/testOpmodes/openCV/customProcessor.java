package org.firstinspires.ftc.teamcode.testOpmodes.openCV;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.testOpmodes.openCV.ROIProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "ROI Processor Test", group = "Test")
@Disabled
public class customProcessor extends LinearOpMode {

    private VisionPortal visionPortal;
    private ROIProcessor roiProcessor;

    @Override
    public void runOpMode() {
        roiProcessor = new ROIProcessor();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .enableLiveView(true)
                .setCameraResolution(new Size(1280, 960))
                .addProcessor(roiProcessor)
                .build();

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Camera Initialized at 1280x960");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            List<Point> purpleBalls = roiProcessor.getPurpleCenters();
            List<Point> greenBalls = roiProcessor.getGreenCenters();

            List<Point> roi5Purple = new ArrayList<>();
            List<Point> roi5Green = new ArrayList<>();

            List<Point> roi4Purple = new ArrayList<>();
            List<Point> roi4Green = new ArrayList<>();

            List<Point> roi3Purple = new ArrayList<>();
            List<Point> roi3Green = new ArrayList<>();

            /*List<Point> roi2Purple = new ArrayList<>();
            List<Point> roi2Green = new ArrayList<>();

            List<Point> roi1Purple = new ArrayList<>();
            List<Point> roi1Green = new ArrayList<>();*/

            for (Point p : purpleBalls) {
                int roi = roiProcessor.getROIIndexForPoint(p);
                if (roi == 5) roi5Purple.add(p);
                else if (roi == 4) roi4Purple.add(p);
                else if (roi == 3) roi3Purple.add(p);
                //else if (roi == 2) roi2Purple.add(p);
                //else if (roi == 1) roi1Purple.add(p);
            }

            for (Point p : greenBalls) {
                int roi = roiProcessor.getROIIndexForPoint(p);
                if (roi == 5) roi5Green.add(p);
                else if (roi == 4) roi4Green.add(p);
                else if (roi == 3) roi3Green.add(p);
                //else if (roi == 2) roi2Green.add(p);
                //else if (roi == 1) roi1Green.add(p);

            }

            telemetry.addLine("ROI 5:");
            for (Point p : roi5Purple) telemetry.addLine(String.format("P: (%.1f, %.1f)", p.x, p.y));
            for (Point p : roi5Green) telemetry.addLine(String.format("G: (%.1f, %.1f)", p.x, p.y));

            telemetry.addLine("\nROI 4:");
            for (Point p : roi4Purple) telemetry.addLine(String.format("P: (%.1f, %.1f)", p.x, p.y));
            for (Point p : roi4Green) telemetry.addLine(String.format("G: (%.1f, %.1f)", p.x, p.y));

            telemetry.addLine("\nROI 3:");
            for (Point p : roi3Purple) telemetry.addLine(String.format("P: (%.1f, %.1f)", p.x, p.y));
            for (Point p : roi3Green) telemetry.addLine(String.format("G: (%.1f, %.1f)", p.x, p.y));

            /*telemetry.addLine("\nROI 2:");
            for (Point p : roi2Purple) telemetry.addLine(String.format("P: (%.1f, %.1f)", p.x, p.y));
            for (Point p : roi2Green) telemetry.addLine(String.format("G: (%.1f, %.1f)", p.x, p.y));

            telemetry.addLine("\nROI 1:");
            for (Point p : roi1Purple) telemetry.addLine(String.format("P: (%.1f, %.1f)", p.x, p.y));
            for (Point p : roi1Green) telemetry.addLine(String.format("G: (%.1f, %.1f)", p.x, p.y));*/

            int count5 = roi5Purple.size() + roi5Green.size();
            int count4 = roi4Purple.size() + roi4Green.size();
            int count3 = roi3Purple.size() + roi3Green.size();
            //int count2 = roi2Purple.size() + roi2Green.size();
            //int count1 = roi1Purple.size() + roi1Green.size();

            String mostPopulated = "None";
            int max = Math.max(count5, Math.max(count4, count3));
            //int max = Math.max(count5, Math.max(count4, Math.max(count3, Math.max(count2, count1))));


            if (max > 0) {
                if (max == count5) mostPopulated = "ROI 5";
                else if (max == count4) mostPopulated = "ROI 4";
                else if (max == count3) mostPopulated = "ROI 3";
                    //else if (max == count2) mostPopulated = "ROI 2";
                    //else if (max == count1) mostPopulated = "ROI 1";
                else mostPopulated = "None";
            }

            telemetry.addLine("\n------------------------");
            telemetry.addData("Most Populated Region", mostPopulated);
            telemetry.update();

            sleep(20);
        }

        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}