package org.firstinspires.ftc.teamcode.testOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.TurretShooter;

//@Config
@Disabled
@TeleOp(name = "Turret Test", group = "Tuning")
public class turretTest extends OpMode {

    TurretShooter turretShooter;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashBoardTelemetry = dashboard.getTelemetry();
    public static double p = 0, i = 0, d = 0, f = 0;
    public static double targetVel = 0.0;
    public static double hoodPos = 0.7;
    public static double limelightOffset = 0.0;


    @Override
    public void init() {
        turretShooter = new TurretShooter(this, Alliance.RED);
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
