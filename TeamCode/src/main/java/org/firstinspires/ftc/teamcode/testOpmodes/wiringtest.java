package org.firstinspires.ftc.teamcode.testOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Wiring Test", group = "TeleOp")
@Disabled
public class wiringtest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx rightMotor = this.hardwareMap.get(DcMotorEx.class, "outtake1");
        DcMotorEx leftMotor = this.hardwareMap.get(DcMotorEx.class, "outtake2");
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a){
                rightMotor.setPower(-0.5);
            } else if (gamepad1.b){
                leftMotor.setPower(0.5);
            }
            telemetry.addData("Current Position", rightMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
