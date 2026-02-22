package org.firstinspires.ftc.teamcode.Robot.Decode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.IntakeSpindexer;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.TurretShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Config
@TeleOp(name = "TeleOp", group = "TeleOp")
public class Teleop extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    Follower         follower;
    IntakeSpindexer  intakeSpindexer;
    TurretShooter    turretShooter;

    Pose startPose = new Pose(0, 0, Math.toRadians(0));

    // Change to Alliance.BLUE if needed
    Alliance alliance = Alliance.RED;

    @Override
    public void init() {
        follower        = Constants.createFollower(hardwareMap);
        intakeSpindexer = new IntakeSpindexer(this);
        turretShooter   = new TurretShooter(this, alliance);

        follower.setStartingPose(startPose);

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

    @Override
    public void loop() {

        // --- DRIVE ---
        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x * 0.58,
                true
        );

        // --- SHOOTER VELOCITY PID (runs every loop) ---
        turretShooter.updateValues(follower.getPose(), follower.getVelocity());
        turretShooter.updateShooterVelocityPID();

        // --- INTAKE / SPINDEXER ---
        boolean shoot = gamepad1.xWasReleased();
        boolean ready = gamepad1.yWasReleased();
        intakeSpindexer.update(shoot, ready);

        // --- TELEMETRY ---
        telemetry.addData("Runtime", runtime.seconds());
        telemetry.addData("Ball Count", intakeSpindexer.getConfirmedBallCount());
        telemetry.addData("Spindexer State", intakeSpindexer.getSpindexerState());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {
        turretShooter.stop();
    }
}