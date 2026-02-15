package org.firstinspires.ftc.teamcode.Robot.Decode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(name = "TeleOp", group = "TeleOp")
public class Teleop extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    Follower follower;

    DcMotorEx spindexer;
//    Servo turret1;
    Servo turret2;
    Servo turret3;
    DcMotorEx leftShooter;
    DcMotorEx rightShooter;
    DcMotorEx intake;
    public static double shooterPower = 0.6;
    public static double intakePower = 1.0;
    public static double spindexerPower = 0.45;
    int turretPos = 0;
    int targetTicks = 0;
    public static double turretPosition = 0.0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        spindexer = this.hardwareMap.get(DcMotorEx.class, "spindexer");
        leftShooter = this.hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = this.hardwareMap.get(DcMotorEx.class, "rightShooter");
        intake = this.hardwareMap.get(DcMotorEx.class, "intake");

//        turret1 = this.hardwareMap.get(Servo.class, "turret1");
        turret2 = this.hardwareMap.get(Servo.class, "turret2");
        turret3 = this.hardwareMap.get(Servo.class, "turret3");

        spindexer.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

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

        turret2.setPosition(turretPosition);
        turret3.setPosition(turretPosition);

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