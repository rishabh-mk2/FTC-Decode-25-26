package org.firstinspires.ftc.teamcode.Robot.Decode.TeleOp;

import static org.firstinspires.ftc.teamcode.Robot.Decode.Robot.TurretShooter.targetVelocity;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.IntakeSpindexer;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.TurretShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(name = "Blue Close TeleOp", group = "TeleOp")
public class BlueCloseTeleop extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    private FtcDashboard dashboard;

    Follower         follower;
    IntakeSpindexer  intakeSpindexer;
    TurretShooter    turretShooter;

    Pose startPose = new Pose(25, 70, Math.toRadians(270));

    // Change to Alliance.BLUE if needed
    Alliance alliance = Alliance.BLUE;

    public static double shooterVel = 0.0;
    public static double hoodPos = 1.0;
    public static double recoil = 0.0;
    public static double turretPos = 0.5;
    DcMotorEx frontLeft, backLeft, backRight, frontRight;

    @Override
    public void init() {
        follower        = Constants.createFollower(hardwareMap);
        intakeSpindexer = new IntakeSpindexer(this, true);
        turretShooter   = new TurretShooter(this, alliance);

        follower.setStartingPose(startPose);

        frontLeft = hardwareMap.get(DcMotorEx.class, "lF");
        backLeft = hardwareMap.get(DcMotorEx.class, "lR");
        backRight = hardwareMap.get(DcMotorEx.class, "rR");
        frontRight = hardwareMap.get(DcMotorEx.class, "rF");


        dashboard = FtcDashboard.getInstance();

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry.setAutoClear(true);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
        runtime.reset();
    }

    boolean expel = false;
    boolean slowMode = false;

    boolean manualBlock = false;
    double dtCurrent = 0.0;
    double totalCurrent = 0.0;

    int loopCounter = 0;
    @Override
    public void loop() {

        // --- DRIVE ---
        follower.update();
        if(!slowMode) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x * 0.45,
                    true
            );
        } else {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * 0.25,
                    -gamepad1.left_stick_x * 0.25,
                    -gamepad1.right_stick_x * 0.45 * 0.4,
                    true
            );
        }

        loopCounter++;

        if(loopCounter % 40 == 0) {
            dtCurrent = frontLeft.getCurrent(CurrentUnit.AMPS) + backLeft.getCurrent(CurrentUnit.AMPS) + backRight.getCurrent(CurrentUnit.AMPS) + frontRight.getCurrent(CurrentUnit.AMPS);
        }

        if(gamepad1.dpadDownWasReleased()) {
            follower.setPose(new Pose(141.5 - 7.0 - 10.0, 8.1 + 10.0, Math.toRadians(180)));
        }
        if(gamepad1.dpadUpWasReleased()) {
            follower.setPose(new Pose(15, 15, Math.toRadians(180)));
        }

        if(dtCurrent > 25) {
            slowMode = true;
        } else {
            slowMode = false;
        }

        if(gamepad1.dpadLeftWasReleased()) {
            slowMode = !slowMode;
        }


        // --- INTAKE / SPINDEXER ---x
        boolean shoot = gamepad1.xWasReleased();
        boolean ready = gamepad1.bWasReleased();
        if(gamepad1.yWasReleased()) {
            manualBlock = !manualBlock;
        }
        if(manualBlock) {
            intakeSpindexer.manualOverrideSpindexerBlock(true);
            intakeSpindexer.setSpindexerBlockPosition();
        } else {
            intakeSpindexer.manualOverrideSpindexerBlock(false);
        }

        intakeSpindexer.update(ready, shoot);

        // --- SHOOTER VELOCITY PID (runs every loop) ---
        turretShooter.update(follower.getPose(), follower.getVelocity(), shoot, true, true, true);

        dashboard.getTelemetry().addData("shooter target", targetVelocity);
        dashboard.getTelemetry().addData("shooter velocity", turretShooter.getMotor(TurretShooter.MotorNames.leftShooter).getVelocity());

        dashboard.getTelemetry().update();

        // --- TELEMETRY ---
        telemetry.addData("Expel?", expel);
        telemetry.addData("Runtime", runtime.seconds());
        telemetry.addData("Current", intakeSpindexer.getMotor(IntakeSpindexer.MotorNames.intake).getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("Ball Count", intakeSpindexer.getConfirmedBallCount());
        telemetry.addData("Spindexer State", intakeSpindexer.getSpindexerState());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading() * 180 / Math.PI);
//        telemetry.addData("INTAKE/SPINDEX CURRENT:", intakeSpindexer.getTotalCurrent());
//        telemetry.addData("TURRET/SHOOTER CURRENT:", turretShooter.getTotalCurrent());
        telemetry.addData("DRIVETRAIN     CURRENT:", dtCurrent);
//        telemetry.addData("TOTAL CURRENT:", intakeSpindexer.getTotalCurrent() + turretShooter.getTotalCurrent() + dtCurrent);
        telemetry.update();
    }

    @Override
    public void stop() {
        turretShooter.stop();
    }
}