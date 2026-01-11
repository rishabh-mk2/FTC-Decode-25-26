package org.firstinspires.ftc.teamcode.testOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.TurretShooter_shreyas;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Config
@Disabled
@TeleOp(name = "Turret PID", group = "Tuning")
public class turretPID extends OpMode {
    TurretShooter_shreyas turretShooter;
    Follower follower;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashBoardTelemetry = dashboard.getTelemetry();

    private PIDController controller;
    public static double p = 0, d = 0, f = 0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        turretShooter = new TurretShooter_shreyas(this, Alliance.RED);
        controller = new PIDController(p, 0, d);
    }

    @Override
    public void start() {
        super.start();
        follower.startTeleopDrive(true);
    }

    public LLResult result;
    double velocity = 0.0;
    @Override
    public void loop() {

        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x * 0.4,
                true
        );

        controller.setPID(p, 0, d);

        result = turretShooter.getLimelight().getLatestResult();

        if (!result.isValid()) {
            telemetry.addData("info", "no april tag being detected");
            turretShooter.getMotor(TurretShooter_shreyas.MotorNames.turret).setPower(0);
            return;
        }
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) {
            turretShooter.getMotor(TurretShooter_shreyas.MotorNames.turret).setPower(0);
            return;
        }

        LLResultTypes.FiducialResult fiducial = fiducials.get(0);

        double currentTargetDeg = fiducial.getTargetXDegrees();
        double turretPos = turretShooter.getMotor(TurretShooter_shreyas.MotorNames.turret).getCurrentPosition();

        double targetTicks = turretPos - (384.5 * currentTargetDeg) / 360.0;

        double pid = controller.calculate(turretPos, targetTicks);

        turretShooter.getMotor(TurretShooter_shreyas.MotorNames.turret).setPower(pid + f*targetTicks);

        dashBoardTelemetry.addData("Target Ticks", targetTicks);
        dashBoardTelemetry.addData("Error", targetTicks - turretPos);
        dashBoardTelemetry.update();
    }
}
