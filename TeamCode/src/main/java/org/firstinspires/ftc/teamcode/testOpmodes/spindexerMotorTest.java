package org.firstinspires.ftc.teamcode.testOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Spindexer Test", group = "Test")
public class spindexerMotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx spindexer = this.hardwareMap.get(DcMotorEx.class, "turret");
        waitForStart();
        spindexer.setPower(0.1);
        while (opModeIsActive()){
            if (gamepad1.dpadUpWasReleased()) {
                spindexer.setPower(spindexer.getPower() + 0.05);
            }
            if (gamepad1.dpadDownWasReleased()) {
                spindexer.setPower(spindexer.getPower() - 0.05);
            }
            telemetry.addLine("==== Motor Speed ====");
            telemetry.addData("Spindxer Speed", spindexer.getPower());
        }
    }
}
