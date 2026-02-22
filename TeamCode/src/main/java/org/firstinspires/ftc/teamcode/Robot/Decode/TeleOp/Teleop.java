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

    public static double turretPosition = 0.0;

    Pose startPose = new Pose(72, 7, Math.toRadians(90));

    // Change to Alliance.BLUE if needed
    Alliance alliance = Alliance.RED;

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
        turretShooter.updateTurret();

        // --- INTAKE / SPINDEXER ---x
        boolean shoot = gamepad1.xWasReleased();
        boolean ready = gamepad1.yWasReleased();
        intakeSpindexer.update(shoot, ready);

        dashboard.getTelemetry().addData("target", targetVelocity);
        dashboard.getTelemetry().addData("velocity", turretShooter.getMotor(TurretShooter.MotorNames.leftShooter).getVelocity());
        dashboard.getTelemetry().update();

//        intakeSpindexer.getServo(IntakeSpindexer.ServoNames.spindexerBlock).setPosition(spindexBlockPos);

//        turretShooter.getServo(TurretShooter.ServoNames.turret1).setPosition(turretPosition);
//        turretShooter.getServo(TurretShooter.ServoNames.turret2).setPosition(turretPosition);

        // --- TELEMETRY ---
        telemetry.addData("Runtime", runtime.seconds());
        telemetry.addData("Current", intakeSpindexer.getMotor(IntakeSpindexer.MotorNames.intake).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Ball Count", intakeSpindexer.getConfirmedBallCount());
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