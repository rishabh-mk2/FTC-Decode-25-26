package org.firstinspires.ftc.teamcode.Robot.Decode.Robot;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;
import org.firstinspires.ftc.teamcode.testOpmodes.openCV.ROIProcessorBlue;
import org.firstinspires.ftc.teamcode.testOpmodes.openCV.ROIProcessorRed;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Point;

import java.util.List;

/**
 * Outputs an integer 1 - 5 based on most populated zone
 * Output 1 ==> 40 inches from corner
 * Output 2 ==> 32 inches from corner
 * Output 3 ==> 24 inches from corner
 * Output 4 ==> 16 inches from corner
 * Output 5 ==> 8 inches from corner
 *
 * Example usage (for myself):
 * // init
 * ZoneDetector detector = new ZoneDetector(opMode);
 *
 * // during runtime
 * detector.getZone();
 *
 * // end
 * detector.close();
 **/
public class ZoneDetector {
    private final VisionPortal     visionPortal;
    private final ROIProcessorRed roiRED;
    private final ROIProcessorBlue roiBLUE;
    private final Alliance         alliance;
    public OpMode      opmode;
    public Telemetry   telemetry;
    public HardwareMap hardwareMap;

    public ZoneDetector(OpMode opMode, Alliance alliance) {
        this.opmode      = opMode;
        this.telemetry   = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;
        this.alliance    = alliance;

        VisionProcessor processor;
        if (alliance == Alliance.RED) {
            roiRED    = new ROIProcessorRed();
            roiBLUE   = null;
            processor = roiRED;
        } else {
            roiBLUE   = new ROIProcessorBlue();
            roiRED    = null;
            processor = roiBLUE;
        }

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(processor)
                .enableLiveView(true)
                .build();
    }

    public int getZone() {
        int[] counts = new int[7];

        List<Point> purpleCenters = (alliance == Alliance.RED) ? roiRED.getPurpleCenters() : roiBLUE.getPurpleCenters();
        List<Point> greenCenters  = (alliance == Alliance.RED) ? roiRED.getGreenCenters()  : roiBLUE.getGreenCenters();

        for (Point p : purpleCenters) {
            int zone = (alliance == Alliance.RED) ? roiRED.getROIIndexForPoint(p) : roiBLUE.getROIIndexForPoint(p);
            if (zone >= 1 && zone <= 6) counts[zone]++;
        }
        for (Point p : greenCenters) {
            int zone = (alliance == Alliance.RED) ? roiRED.getROIIndexForPoint(p) : roiBLUE.getROIIndexForPoint(p);
            if (zone >= 1 && zone <= 6) counts[zone]++;
        }

        int bestPair  = 0;
        int bestCount = 0;
        for (int i = 1; i <= 5; i++) {
            int pairCount = counts[i] + counts[i + 1];
            if (pairCount > bestCount) {
                bestCount = pairCount;
                bestPair  = i;
            }
        }
        return bestPair;
    }

    public void stop() {
        visionPortal.close();
    }
}