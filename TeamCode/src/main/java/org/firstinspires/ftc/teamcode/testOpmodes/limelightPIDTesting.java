package org.firstinspires.ftc.teamcode.testOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.TurretShooter_shreyas;

@TeleOp(name = "Turret PID Tuner", group = "Tuning")
public class limelightPIDTesting extends OpMode {
    TurretShooter_shreyas turretShooter;

    // Tunable values
    public static double kP = 0.02;
    public static double kI = 0.0;
    public static double kD = 0.001;

    @Override
    public void init() {
        turretShooter = new TurretShooter_shreyas(this, Alliance.RED);
    }

    @Override
    public void loop() {
        // Update PID values in real-time
        turretShooter.turretPID.setPID(kP, kI, kD);

        // Run tracking
        turretShooter.trackAprilTag();

        // Manual adjustment with gamepad
        if (gamepad1.dpadUpWasReleased()) kP += 0.001;
        if (gamepad1.dpadDownWasReleased()) kP -= 0.001;
        if (gamepad1.dpadRightWasReleased()) kD += 0.0001;
        if (gamepad1.dpadLeftWasReleased()) kD -= 0.0001;

        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);
        telemetry.update();
    }
}
