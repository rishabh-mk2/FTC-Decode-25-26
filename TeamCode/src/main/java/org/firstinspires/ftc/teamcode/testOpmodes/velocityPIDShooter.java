package org.firstinspires.ftc.teamcode.testOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.TurretShooter_shreyas;

@Config
@TeleOp(name = "Shooter PID Tuner", group = "Tuning")
public class velocityPIDShooter extends OpMode {

    TurretShooter_shreyas turretShooter;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashBoardTelemetry = dashboard.getTelemetry();

    private PIDController controller;
    public static double p = 0, i = 0, d = 0, f = 0;
    public static double targetVel = 0.0;

    @Override
    public void init() {
        turretShooter = new TurretShooter_shreyas(this, Alliance.RED);
        controller = new PIDController(p, i, d);
    }

    double velocity = 0.0;
    @Override
    public void loop() {
        controller.setPID(p, i, d);
        double currentVelocity = turretShooter.getMotor(TurretShooter_shreyas.MotorNames.rightShooter).getVelocity();
        double pid = controller.calculate(currentVelocity, targetVel);

        double velocity = pid + f*targetVel;

        turretShooter.setShooterVelocity(velocity);

        dashBoardTelemetry.addData("Target Velocity", targetVel);
        dashBoardTelemetry.addData("Current Velocity", currentVelocity);
        dashBoardTelemetry.update();

        telemetry.addData("p", p);
    }
}
