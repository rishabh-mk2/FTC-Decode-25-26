package org.firstinspires.ftc.teamcode.testOpmodes;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Color", group = "TeleOp")
public class colorSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RevColorSensorV3 color1 = this.hardwareMap.get(RevColorSensorV3.class, "frontSensor1");
        RevColorSensorV3 color2 = this.hardwareMap.get(RevColorSensorV3.class, "frontSensor2");

        waitForStart();
        while (opModeIsActive()) {
            int red = (color1.red() + color2.red())/2;
            int green = (color1.green() + color2.green())/2;
            int blue = (color1.blue() + color2.blue())/2;
            telemetry.addData("Red",red);
            telemetry.addData("Green", green);
            telemetry.addData("BLue", blue);
            telemetry.update();
        }
    }
}
