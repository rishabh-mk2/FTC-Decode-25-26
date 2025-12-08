package org.firstinspires.ftc.teamcode.testOpmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Velocity PID", group = "Test")
@Configurable
public class velocityPID extends LinearOpMode {

    PIDController pidController;
    double p = 0.0100, i = 0, d = 0.0000;
    double targetRPM = 100;
    final double TICKS_PER_REV = 537.7;
    public static double currentRpm;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        DcMotorEx outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");

        outtake1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtake1.setDirection(DcMotorEx.Direction.FORWARD);
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
        while (opModeIsActive()) {


            // turn on/off motors
            if(gamepad1.dpad_up && !dPadUpIsPressed){
                dPadUpIsPressed = true;
            }
            if (!gamepad1.dpad_up && dPadUpIsPressed) {
                outtake1.setVelocity(100);
                outtake2.setVelocity(100);
                dPadUpIsPressed = false;
            }

            if(gamepad1.dpad_down && !dPadDownIsPressed){
                dPadDownIsPressed = true;
            } if(!gamepad1.dpad_down && dPadDownIsPressed){
                outtake1.setVelocity(0);
                outtake2.setVelocity(0);
                dPadDownIsPressed = false;
            }

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
            currentRpm = Math.abs((rpm1 - rpm2) / 2.0);

            // PID calculates using the (measured, target))
            double pidOut = pidController.calculate(currentRpm, targetRPM);

            double power = pidOut * 0.001;

            // make sure power is never negative (so it doesnt reverse the direction)
            if (power < 0) power = 0;
            if (power > 1.0) power = 1.0;

            // set both motors to the power
            //outtake1.setPower(0.6);
            //outtake2.setPower(0.6);


            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Current RPM (avg)", currentRpm);
            telemetry.addData("RPM1", rpm1);
            telemetry.addData("RPM2", rpm2);
            telemetry.addData("PID output", pidOut);
            telemetry.addData("Applied Power", power);
            telemetry.addData("kP", p);
            telemetry.addData("kI", i);
            telemetry.addData("kD", d);
            telemetry.update();
        }
    }
}