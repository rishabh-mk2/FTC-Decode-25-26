package org.firstinspires.ftc.teamcode.testOpmodes.openCV;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;

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
    private final VisionPortal visionPortal;
    private final ROIProcessor roi;
    public OpMode      opmode;
    public Telemetry   telemetry;
    public HardwareMap hardwareMap;
    ExposureControl    exposureControl;

    public ZoneDetector(OpMode opMode) {
        this.opmode      = opMode;
        this.telemetry   = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

        roi = new ROIProcessor();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(roi)
                .enableLiveView(true)
                .build();
    }

    public int getZone() {
        int[] counts = new int[7];

        for (Point p : roi.getPurpleCenters()) {
            int zone = roi.getROIIndexForPoint(p);
            if (zone >= 1 && zone <= 6) counts[zone]++;
        }
        for (Point p : roi.getGreenCenters()) {
            int zone = roi.getROIIndexForPoint(p);
            if (zone >= 1 && zone <= 6) counts[zone]++;
        }

        // checks each pair of zones
        // outputs zone with greatest frequency
        // example: zone1 + zone 2 = 3           example 2: zone 1 + zone 2 = 4
        // example contd: zone 2 + zone 3 = 4    example 2: zone 2 + zone 3 = 2
        // output of above: 2                    example 2: 1

        // essentially outputs the smaller zone number of the pair
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

    public void close() {
        visionPortal.close();
    }
}