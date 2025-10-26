package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@TeleOp(name = "Outtake Test", group = "TeleOp")
public class outtakeTest extends LinearOpMode {

    // Default PIDF
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        DcMotorEx intake2 = hardwareMap.get(DcMotorEx.class, "intake2");

        // Clone motor configurations and unlock full RPM capability
        MotorConfigurationType motorConfig1 = intake1.getMotorType().clone();
        motorConfig1.setAchieveableMaxRPMFraction(1.0);
        intake1.setMotorType(motorConfig1);

        MotorConfigurationType motorConfig2 = intake2.getMotorType().clone();
        motorConfig2.setAchieveableMaxRPMFraction(1.0);
        intake2.setMotorType(motorConfig2);

        // Ensure both motors are using encoders
        intake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Voltage compensation (prevents overspeed when battery full)
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        double compensatedF = MOTOR_VELO_PID.f * 12 / voltage;

        // Apply PIDF coefficients
        PIDFCoefficients compensatedPIDF = new PIDFCoefficients(
                MOTOR_VELO_PID.p,
                MOTOR_VELO_PID.i,
                MOTOR_VELO_PID.d,
                compensatedF
        );

        intake1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, compensatedPIDF);
        intake2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, compensatedPIDF);

        waitForStart();

        // Run both motors at full reverse power for testing
        while (opModeIsActive()) {
            intake1.setPower(-1);
            intake2.setPower(-1);
            sleep(5000);
            telemetry.addData("Motor Power", "-1.0 (Full Reverse)");
            telemetry.addData("Intake1 Velocity", "%.2f", intake1.getVelocity());
            telemetry.addData("Intake2 Velocity", "%.2f", intake2.getVelocity());
            telemetry.addData("Runtime", "%.2f s", getRuntime());
            telemetry.update();
        }
        intake1.setPower(0);
        intake2.setPower(0);
    }
}