package org.firstinspires.ftc.teamcode.testOpmodes;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoPosition Finder", group = "TeleOp")

public class servoPositionFinder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo spin1 = this.hardwareMap.get(Servo.class, "spin1");
        Servo spin2 = this.hardwareMap.get(Servo.class, "spin2");
        RevColorSensorV3 frontSensor = hardwareMap.get(RevColorSensorV3.class, "front");

        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.aWasReleased()){
                spin1.setPosition(spin1.getPosition()+0.17);
            }
            if(gamepad1.bWasReleased()){
                spin1.setPosition(spin1.getPosition()-0.17);
            }
            telemetry.addData("position:",spin1.getPosition());
            telemetry.addData("RGB", String.valueOf(frontSensor.red()), frontSensor.green(), frontSensor.blue());
            telemetry.update();
        }
    }
}
