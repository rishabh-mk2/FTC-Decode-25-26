package org.firstinspires.ftc.teamcode.testOpmodes;

import android.adservices.common.AdServicesOutcomeReceiver;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Turret Test", group = "Test")
public class servoTurret extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo turret1 = this.hardwareMap.get(Servo.class, "turret1");
        Servo turret2 = this.hardwareMap.get(Servo.class, "turret2");
        Servo turret3 = this.hardwareMap.get(Servo.class, "turret3");
        waitForStart();
        turret1.setPosition(0.0);
        turret2.setPosition(0.0);
        turret3.setPosition(0.0);
        while (opModeIsActive()){
            turret1.setPosition(1.0);
            turret2.setPosition(1.0);
            turret3.setPosition(1.0);
        }
    }
}
