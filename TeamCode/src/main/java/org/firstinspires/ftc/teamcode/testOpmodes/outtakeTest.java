package org.firstinspires.ftc.teamcode.testOpmodes;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Velocity PID", group = "Test")
public class outtakeTest extends LinearOpMode {

    PIDController pidController;
    double p = 0.0000, i = 0, d = 0.0000;
    double targetRPM = 6000;
    final double TICKS_PER_REV = 537.7;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx outtake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        DcMotorEx outtake2 = hardwareMap.get(DcMotorEx.class, "intake2");

        outtake1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtake1.setDirection(DcMotorEx.Direction.REVERSE);
        outtake2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtake2.setDirection(DcMotorEx.Direction.REVERSE);

        outtake1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        outtake2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        pidController = new PIDController(p, i, d);

        telemetry.addLine("Ready. Press Play to start!");
        telemetry.update();

        boolean AisPressed = false;
        boolean BisPressed = false;
        boolean XisPressed = false;
        boolean YisPressed = false;
        boolean dPadUpIsPressed = false;
        boolean dPadDownIsPressed = false;

        waitForStart();

        // immediately spin up to full power
        outtake1.setPower(1);
        outtake2.setPower(1);

        // small delay to let the motors start before PID correction
        sleep(10);
        while (opModeIsActive()) {

            if(gamepad1.a && !AisPressed){
                AisPressed = true;
            }
            if (!gamepad1.a && AisPressed) {
                p += 0.0001;
                AisPressed = false;
            } if(gamepad1.b && !BisPressed){
                BisPressed = true;
            } if(!gamepad1.b && BisPressed){
                p -= 0.0001;
                BisPressed = false;
            }

            if(gamepad1.x && !XisPressed){
                XisPressed = true;
            }
            if (!gamepad1.x && XisPressed) {
                d += 0.0001;
                XisPressed = false;
            } if(gamepad1.y && !YisPressed){
                YisPressed = true;
            } if(!gamepad1.y && YisPressed){
                d -= 0.0001;
                YisPressed = false;
            }

            // finds how many ticks the motor moved (basically the raw rotational speed)
            double velTicksPerSec1 = outtake1.getVelocity();
            double velTicksPerSec2 = outtake2.getVelocity();

            // convert velocity to RPM (seconds to minutes and then divide by TICKS_per_REV to convert the tick values to revolutions)
            double rpm1 = velTicksPerSec1 * 60.0 / TICKS_PER_REV;
            double rpm2 = velTicksPerSec2 * 60.0 / TICKS_PER_REV;

            // averaging the motor RPM since they need to move at the same speed -> gives a stable feedback instead of possibly 2 different values
            double currentRPM = (rpm1 + rpm2) / 2.0;

            // PID calculates using the (measured, target))
            double pidOut = pidController.calculate(currentRPM, targetRPM);

            // add a small base power to make sure it never goes to 0
            double basePower = 1.0; // start strong
            double power = basePower + pidOut * 0.001;

            // make sure power is never negative (so it doesnt reverse the direction)
            if (power < 0) power = 0;
            if (power > 1.0) power = 1.0;

            // set both motors to the power
            outtake1.setPower(power);
            outtake2.setPower(power);

            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Current RPM (avg)", "%.1f", currentRPM);
            telemetry.addData("RPM1", "%.1f", rpm1);
            telemetry.addData("RPM2", "%.1f", rpm2);
            telemetry.addData("PID output", pidOut);
            telemetry.addData("Applied Power", "%.3f", power);
            telemetry.addData("kP", p);
            telemetry.addData("kI", i);
            telemetry.addData("kD", d);
            telemetry.update();
        }
    }
}