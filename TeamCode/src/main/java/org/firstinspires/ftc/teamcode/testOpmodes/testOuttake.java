package org.firstinspires.ftc.teamcode.testOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Outtake Test", group = "TeleOp")
public class testOuttake extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx rightMotor = this.hardwareMap.get(DcMotorEx.class, "rightMotor");
        DcMotorEx leftMotor = this.hardwareMap.get(DcMotorEx.class, "leftMotor");

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while (opModeIsActive()) {
            rightMotor.setPower(-0.9);
            leftMotor.setPower(0.9);
        }
    }
}
