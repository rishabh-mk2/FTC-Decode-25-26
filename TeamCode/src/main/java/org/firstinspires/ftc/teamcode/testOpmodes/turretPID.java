package org.firstinspires.ftc.teamcode.testOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.TurretShooter_shreyas;

@Config
@TeleOp(name = "Turret PID Tuner", group = "Tuning")
public class turretPID extends OpMode {

    TurretShooter_shreyas turretShooter;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashBoardTelemetry = dashboard.getTelemetry();

    public static double P = 0.0;
    public static double D = 0.0;
    public static double F = 0.0;

    @Override
    public void init() {
        turretShooter = new TurretShooter_shreyas(this, Alliance.RED);
        PIDFCoefficients shooterPIDFCoefficients = new PIDFCoefficients(P, 0, D, F);
        turretShooter.getMotor(TurretShooter_shreyas.MotorNames.turret).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDFCoefficients);
    }
    @Override
    public void loop() {

    }
}
