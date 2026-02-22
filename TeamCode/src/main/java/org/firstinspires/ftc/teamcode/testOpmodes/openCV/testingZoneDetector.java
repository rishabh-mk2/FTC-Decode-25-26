package org.firstinspires.ftc.teamcode.testOpmodes.openCV;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Zone Detector", group = "Test")
public class testingZoneDetector extends OpMode {
    ZoneDetector detector;
    @Override
    public void init() {
        detector = new ZoneDetector(this);
    }

    @Override
    public void loop() {
        int bestZone = detector.getZone();
        detector.telemetry.addData("Best Zone: ", bestZone);
    }

    @Override
    public void stop() {
        detector.close();
        super.stop();
    }
}
