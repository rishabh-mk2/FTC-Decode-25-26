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
        turretShooter.trackAprilTag();
        telemetry.update();
    }
}
