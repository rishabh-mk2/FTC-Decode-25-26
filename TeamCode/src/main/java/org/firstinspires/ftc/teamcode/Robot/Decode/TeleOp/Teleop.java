package org.firstinspires.ftc.teamcode.Robot.Decode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Config
@TeleOp(name = "TeleOp", group = "TeleOp")
public class Teleop extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    Follower follower;
    PIDController spindexerPID;

    DcMotorEx spindexer;
    Servo turret1;
    Servo turret2;
    Servo turret3;
    DcMotorEx leftShooter;
    DcMotorEx rightShooter;
    DcMotorEx intake;
    public static double shooterPower = 0.0;
    public static double intakePower = 0.0;
    int turretPos = 0;
    int targetTicks = 0;
    public static double turretPosition = 0.475;


    RevColorSensorV3 frontSensor1;
    RevColorSensorV3 frontSensor2;
    RevColorSensorV3 backRightSensor1;
    RevColorSensorV3 backRightSensor2;
    RevColorSensorV3 backLeftSensor1;
    RevColorSensorV3 backLeftSensor2;

    Pose startPose = new Pose(0, 0, Math.toRadians(0));

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        spindexer = this.hardwareMap.get(DcMotorEx.class, "spindexer");
        leftShooter = this.hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = this.hardwareMap.get(DcMotorEx.class, "rightShooter");
        intake = this.hardwareMap.get(DcMotorEx.class, "intake");

        turret1 = this.hardwareMap.get(Servo.class, "turret1");
        turret2 = this.hardwareMap.get(Servo.class, "turret2");
        turret3 = this.hardwareMap.get(Servo.class, "turret3");

        spindexer.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontSensor1 = hardwareMap.get(RevColorSensorV3.class, "f1");
        frontSensor2 = hardwareMap.get(RevColorSensorV3.class, "f2");
        backRightSensor1 = hardwareMap.get(RevColorSensorV3.class, "br1");
        backRightSensor2 = hardwareMap.get(RevColorSensorV3.class, "br2");
        backLeftSensor1 = hardwareMap.get(RevColorSensorV3.class, "bl1");
        backLeftSensor2 = hardwareMap.get(RevColorSensorV3.class, "bl2");

        targetTicks = spindexer.getCurrentPosition();

        follower.setStartingPose(startPose);

        telemetry.setAutoClear(true);
        runtime.reset();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
        follower.startTeleopDrive(true);
        runtime.reset();
    }

    public static double spindexerVelocity = 0.0;

    double pid = 0.0;

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
            targetTicks = spindexer.getCurrentPosition() - 450;
        }
        if (gamepad1.dpadUpWasReleased()) {
            spindexer.setPower(spindexer.getPower() + 0.05);
        } else if (gamepad1.dpadDownWasReleased()) {
            spindexer.setPower(spindexer.getPower() - 0.05);
        }

        turret1.setPosition(turretPosition);
        turret2.setPosition(turretPosition);
        turret3.setPosition(turretPosition);

//        pid = spindexerPID.calculate(spindexer.getVelocity(), spindexerVelocity);

//        spindexer.setTargetPosition(targetTicks);
//        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        spindexer.setPower(-0.45);

        telemetry.addData("Spindxer Speed", spindexer.getPower());
        telemetry.addData("Spindexer Vel", spindexer.getVelocity());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());

        telemetry.update();

    }

    @Override
    public void stop() {

    }
}