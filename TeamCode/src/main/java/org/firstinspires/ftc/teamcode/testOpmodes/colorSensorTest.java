package org.firstinspires.ftc.teamcode.testOpmodes;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Color", group = "TeleOp")
public class colorSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RevColorSensorV3 color = this.hardwareMap.get(RevColorSensorV3.class, "front");

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Red",color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("BLue", color.blue());
            telemetry.update();
        }
    }
}
