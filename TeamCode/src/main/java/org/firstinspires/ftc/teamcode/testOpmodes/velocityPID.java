package org.firstinspires.ftc.teamcode.testOpmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.bylazar.telemetry.PanelsTelemetry;

@TeleOp(name = "Velocity PID", group = "Test")
@Configurable
public class velocityPID extends LinearOpMode {

    PIDController pidController;
    public static double p = 0.0100, i = 0, d = 0.0000;
    public static double targetRPM = 1000;
    public static double currentRpm;
    public static double error = targetRPM - currentRpm;
    final double TICKS_PER_REV = 537.7;

    // Flag to track if motors should be running
    private boolean motorsEnabled = false;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx outtake1 = hardwareMap.get(DcMotorEx.class, "leftShooter");
        DcMotorEx outtake2 = hardwareMap.get(DcMotorEx.class, "rightShooter");

        outtake1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtake1.setDirection(DcMotorEx.Direction.REVERSE);
        outtake2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtake2.setDirection(DcMotorEx.Direction.FORWARD);

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

            // Update PID coefficients
            pidController.setPID(p, i, d);

            // Turn on motors
            if (gamepad1.dpad_up && !dPadUpIsPressed) {
                dPadUpIsPressed = true;
            }
            if (!gamepad1.dpad_up && dPadUpIsPressed) {
                motorsEnabled = true;
                dPadUpIsPressed = false;
            }

            // Turn off motors
            if (gamepad1.dpad_down && !dPadDownIsPressed) {
                dPadDownIsPressed = true;
            }
            if (!gamepad1.dpad_down && dPadDownIsPressed) {
                motorsEnabled = false;
                outtake1.setPower(0);
                outtake2.setPower(0);
                dPadDownIsPressed = false;
            }

            // Adjust P
            if (gamepad1.a && !AisPressed) {
                AisPressed = true;
            }
            if (!gamepad1.a && AisPressed) {
                p += 0.0001;
                AisPressed = false;
            }
            if (gamepad1.b && !BisPressed) {
                BisPressed = true;
            }
            if (!gamepad1.b && BisPressed) {
                p -= 0.0001;
                BisPressed = false;
            }

            // Adjust D
            if (gamepad1.x && !XisPressed) {
                XisPressed = true;
            }
            if (!gamepad1.x && XisPressed) {
                d += 0.0001;
                XisPressed = false;
            }
            if (gamepad1.y && !YisPressed) {
                YisPressed = true;
            }
            if (!gamepad1.y && YisPressed) {
                d -= 0.0001;
                YisPressed = false;
            }


            double velTicksPerSec1 = outtake1.getVelocity();
            double velTicksPerSec2 = outtake2.getVelocity();


            double rpm1 = velTicksPerSec1 * 60.0 / TICKS_PER_REV;
            double rpm2 = velTicksPerSec2 * 60.0 / TICKS_PER_REV;

            currentRpm = Math.abs((rpm1 + rpm2) / 2.0);

            double pidOut = pidController.calculate(currentRpm, targetRPM);

            double power = pidOut;

            if (power < 0) power = 0;
            if (power > 1.0) power = 1.0;

            outtake1.setPower(power);
            outtake2.setPower(power);

            telemetry.addData("Applied Power", power);
            telemetry.addData("Motors Enabled", motorsEnabled);
            telemetry.addData("Target", targetRPM);
            telemetry.addData("Current", currentRpm);
            //telemetry.addData("RPM1", rpm1);
            //telemetry.addData("RPM2", rpm2);
            telemetry.addData("Error", error);
            telemetry.addData("kP", p);
            telemetry.addData("kI", i);
            telemetry.addData("kD", d);
            telemetry.update();
        }
    }
}