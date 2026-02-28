package org.firstinspires.ftc.teamcode.Robot.Decode.TeleOp;

import static org.firstinspires.ftc.teamcode.Robot.Decode.Robot.TurretShooter.targetVelocity;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.IntakeSpindexer;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.TurretShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Config
@TeleOp(name = "TeleOp", group = "TeleOp")
public class Teleop extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    private FtcDashboard dashboard;

    Follower         follower;
    IntakeSpindexer  intakeSpindexer;
    TurretShooter    turretShooter;

    Pose startPose = new Pose(72, 72, Math.toRadians(45));

    // Change to Alliance.BLUE if needed
    Alliance alliance = Alliance.RED;

    public static double shooterVel = 0.0;
    public static double hoodPos = 1.0;
    public static double recoil = 0.0;
    public static double turretPos = 0.5;

    @Override
    public void init() {
        follower        = Constants.createFollower(hardwareMap);
        intakeSpindexer = new IntakeSpindexer(this);
        turretShooter   = new TurretShooter(this, alliance);

        follower.setStartingPose(startPose);

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


        // REHOME POSIITON
        if(gamepad1.dpadUpWasReleased()) {
            follower.setPose(new Pose(143 - 2.1, 12.1, Math.toRadians(0)));
        }
        if(gamepad1.dpadDownWasReleased()) {
            follower.setPose(new Pose(7.05, 8.1, Math.toRadians(0)));
        }

        if(gamepad1.dpadLeftWasReleased()) {
            slowMode = !slowMode;
        }


        // --- INTAKE / SPINDEXER ---x
        boolean shoot = gamepad1.xWasReleased();
        boolean ready = gamepad1.yWasReleased();

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
        telemetry.update();
    }

    @Override
    public void stop() {
        turretShooter.stop();
    }
}