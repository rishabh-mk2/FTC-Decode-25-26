package org.firstinspires.ftc.teamcode.testOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.TurretShooter_shreyas;

//@Config
@Disabled
@TeleOp(name = "Turret Test", group = "Tuning")
public class turretTest extends OpMode {

    TurretShooter_shreyas turretShooter;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashBoardTelemetry = dashboard.getTelemetry();
    public static double p = 0, i = 0, d = 0, f = 0;
    public static double targetVel = 0.0;
    public static double hoodPos = 0.7;
    public static double limelightOffset = 0.0;


    @Override
    public void init() {
        turretShooter = new TurretShooter_shreyas(this, Alliance.RED);
    }

    double velocity = 0.0;
    @Override
    public void loop() {
        turretShooter.shoot(targetVel, hoodPos);
        turretShooter.trackAprilTag(limelightOffset);

        dashBoardTelemetry.addData("Target Velocity", targetVel);
        dashBoardTelemetry.addData("Distance", turretShooter.getAprilTagDistance());
        dashBoardTelemetry.update();

        telemetry.update();
    }
}
