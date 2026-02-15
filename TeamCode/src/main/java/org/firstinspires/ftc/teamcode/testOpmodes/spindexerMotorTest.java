package org.firstinspires.ftc.teamcode.testOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Spindexer Test", group = "Test")
@Disabled
public class spindexerMotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx spindexer = this.hardwareMap.get(DcMotorEx.class, "spindexer");
        DcMotorEx leftShooter = this.hardwareMap.get(DcMotorEx.class, "leftShooter");
        DcMotorEx rightShooter = this.hardwareMap.get(DcMotorEx.class, "rightShooter");
        DcMotorEx intake = this.hardwareMap.get(DcMotorEx.class, "intake");

        spindexer.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        waitForStart();
        spindexer.setPower(0.0);

        while (opModeIsActive()){
            leftShooter.setPower(0.5);
            rightShooter.setPower(-0.5);
            intake.setPower(1.0);
            if (gamepad1.xWasReleased()) {
                spindexer.setPower(-1.0);
            } else if (gamepad1.dpadUpWasReleased()) {
                spindexer.setPower(spindexer.getPower() + 0.05);
            } else if (gamepad1.dpadDownWasReleased()) {
                spindexer.setPower(spindexer.getPower() - 0.05);
            }
            telemetry.addLine("==== Motor Speed ====");
            telemetry.addData("Spindxer Speed", spindexer.getPower());
            telemetry.update();
        }
    }
}
