package org.firstinspires.ftc.teamcode.testOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Kicker Test2", group = "TeleOp")
public class kickerTest2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo kicker = this.hardwareMap.get(Servo.class, "kicker");

        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.aWasReleased()){
                kicker.setPosition(kicker.getPosition()+0.05);
            }
            if(gamepad1.bWasReleased()){
                kicker.setPosition(kicker.getPosition()-0.05);
            }
            telemetry.addData("position:",kicker.getPosition());
            telemetry.update();
        }
    }
}
