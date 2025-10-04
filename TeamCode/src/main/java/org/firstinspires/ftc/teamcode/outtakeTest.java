package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Outtake Test", group = "TeleOp")
public class outtakeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor intake1 = this.hardwareMap.get(DcMotor.class, "intake1");
        DcMotor intake2 = this.hardwareMap.get(DcMotor.class, "intake2");

        waitForStart();

        while (opModeIsActive()){
            intake1.setPower(-1);
            intake2.setPower(-1);
        }
    }
}

