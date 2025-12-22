package org.firstinspires.ftc.teamcode.Robot.Decode.TeleOp;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.IntakeSpindexer;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.TurretShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOpAll extends OpMode {

    Follower follower;

    TurretShooter turretShooter;
    IntakeSpindexer intakeSpindexer;

    boolean shootPressed = false;
    int ballsToShoot = 0;
    long shootStartTime = 0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Constants.createFollower(hardwareMap).getPose());

        turretShooter = new TurretShooter(this, Alliance.RED); // change alliance if needed
        intakeSpindexer = new IntakeSpindexer(this);
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        // --- DRIVE ---
        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );

        // --- SUBSYSTEM UPDATES ---
        turretShooter.updateTurretTracking();
        intakeSpindexer.update();

        // --- SHOOT INPUT ---
        // Example: press A to shoot 1 ball, B for 2, X for 3
        if (gamepad1.a && !shootPressed) {
            shootPressed = true;
            ballsToShoot = 1;
            shootStartTime = System.currentTimeMillis();
        }
        if (gamepad1.b && !shootPressed) {
            shootPressed = true;
            ballsToShoot = 2;
            shootStartTime = System.currentTimeMillis();
        }
        if (gamepad1.x && !shootPressed) {
            shootPressed = true;
            ballsToShoot = 3;
            shootStartTime = System.currentTimeMillis();
        }

        if (!gamepad1.a && !gamepad1.b && !gamepad1.x) {
            shootPressed = false;
        }

        // --- SHOOT SEQUENCE ---
        if (ballsToShoot > 0 && intakeSpindexer.hasBalls()) {
            // spin up shooter
            turretShooter.setShooterVelocity(-2250);

            // timing for kicker
            long elapsed = System.currentTimeMillis() - shootStartTime;
            for (int i = 0; i < ballsToShoot; i++) {
                if (elapsed > i * 700 && elapsed < i * 700 + 100) {
                    intakeSpindexer.getServo(IntakeSpindexer.ServoNames.kicker).setPosition(0.4);
                } else if (elapsed >= i * 700 + 100 && elapsed < i * 700 + 200) {
                    intakeSpindexer.getServo(IntakeSpindexer.ServoNames.kicker).setPosition(0.225);
                    intakeSpindexer.consumeBall();
                }
            }

            // stop shooter when done
            if (elapsed > ballsToShoot * 700) {
                turretShooter.stopShooter();
                ballsToShoot = 0;
            }
        }

        // Optional telemetry
        telemetry.addData("Balls Loaded", intakeSpindexer.getBallCount());
        telemetry.update();
    }

    @Override
    public void stop() {
        turretShooter.stop();
    }
}
