package org.firstinspires.ftc.teamcode.testOpmodes;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@TeleOp(name = "ROI Processor Test", group = "Test")
public class customProcessor extends LinearOpMode {

    private VisionPortal visionPortal;
    private ROIProcessor roiProcessor;

    @Override
    public void runOpMode() {
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        roiProcessor = new ROIProcessor();
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .enableLiveView(true)
                .setCameraResolution(new Size(1280, 960))
                .addProcessor(roiProcessor)
                .build();
        visionPortal.setProcessorEnabled(roiProcessor, true);

        telemetry.addLine("Camera initialized");
        telemetry.update();

        waitForStart();

        // Keep OpMode alive while camera runs
        while (opModeIsActive()) {
            telemetry.addLine("Streaming...");
            telemetry.update();
            sleep(50);
        }

        // Cleanup
        visionPortal.close();
    }
}
