package org.firstinspires.ftc.teamcode.testOpmodes.Mechs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.config.Config;


@Config
@TeleOp(name = "Shooter Velocity PID", group = "TeleOp")
public class ShooterVelocityPId extends OpMode {
    PIDController shooterPID;
    private FtcDashboard dashboard;

    DcMotorEx leftShooter, rightShooter;

    public static double p, i, d, f = 0.0;
    public static double targetVelocity = 0;

    @Override
    public void init() {

        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");

        leftShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterPID = new PIDController(p, i, d);

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        dashboard = FtcDashboard.getInstance();

        telemetry.setAutoClear(true);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        shooterPID.setPID(p, i, d);

        double currentVelocity = leftShooter.getVelocity();

        double pid = shooterPID.calculate(currentVelocity, targetVelocity);
        double ff = targetVelocity * f;

        double power = pid + ff;

        power = Math.max(0, Math.min(power, 1));

        leftShooter.setPower(power);
        rightShooter.setPower(-power);

        dashboard.getTelemetry().addData("target", targetVelocity);
        dashboard.getTelemetry().addData("velocity", currentVelocity);
        dashboard.getTelemetry().update();

        telemetry.addData("Target", targetVelocity);
        telemetry.addData("Velocity", currentVelocity);
        telemetry.addData("Power", power);
        telemetry.update();
    }

    @Override
    public void stop() {

    }
}
