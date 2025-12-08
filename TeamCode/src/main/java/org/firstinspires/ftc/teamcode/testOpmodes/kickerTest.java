package org.firstinspires.ftc.teamcode.testOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Kicker Test", group = "TeleOp")
public class kickerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo top = this.hardwareMap.get(CRServo.class, "topServo");
        CRServo bottom = this.hardwareMap.get(CRServo.class, "bottomServo");
        DcMotorEx intakeMotor = this.hardwareMap.get(DcMotorEx.class, "intakeMotor");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {
            top.setPower(1);
            bottom.setPower(1);
            intakeMotor.setPower(0.25);
        }
    }
}
