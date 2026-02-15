package org.firstinspires.ftc.teamcode.Robot.Decode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.IntakeSpindexer_shreyas;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.TurretShooter_shreyas;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(name = "Test Teleop V2", group = "TeleOp")
public class v2tests extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    Follower follower;

    DcMotorEx spindexer;
    DcMotorEx leftShooter;
    DcMotorEx rightShooter;
    DcMotorEx intake;
    PIDController spindexerPID;

//    public static double p = 0.03;
//    public static double i = 0.0;
//    public static double d = 0.0001;
    public static double shooterPower = 0.6;
    public static double intakePower = 1.0;
    public static double spindexerPower = 0.0;
    int turretPos = 0;
    int targetTicks = 0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        spindexer = this.hardwareMap.get(DcMotorEx.class, "spindexer");
        leftShooter = this.hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = this.hardwareMap.get(DcMotorEx.class, "rightShooter");
        intake = this.hardwareMap.get(DcMotorEx.class, "intake");

        spindexer.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        spindexer.setPower(0.0);

//        spindexerPID = new PIDController(p, i, d);
//        spindexerPID.setPID(p, i, d);

        turretPos = spindexer.getCurrentPosition();
        targetTicks = turretPos;

        telemetry.setAutoClear(true);
        runtime.reset();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
        follower.startTeleopDrive(true);
        runtime.reset();
    }

    @Override
    public void loop() {
        // --- DRIVE ---
        follower.update();

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x * 0.4,
                true
        );

        leftShooter.setPower(shooterPower);
        rightShooter.setPower(-shooterPower);
        intake.setPower(intakePower);
        if (gamepad1.xWasReleased()) {
            targetTicks = turretPos - 450;
        } else if (gamepad1.dpadUpWasReleased()) {
            spindexer.setPower(spindexer.getPower() + 0.05);
        } else if (gamepad1.dpadDownWasReleased()) {
            spindexer.setPower(spindexer.getPower() - 0.05);
        }
        turretPos = spindexer.getCurrentPosition();

        spindexer.setPower(-spindexerPower);

        telemetry.addLine("==== Motor Speed ====");
        telemetry.addData("Spindxer Speed", spindexer.getPower());
        telemetry.addData("Spindexer Position", spindexer.getCurrentPosition());
        telemetry.update();

    }

    @Override
    public void stop() {

    }
}