package org.firstinspires.ftc.teamcode.Robot.Decode.TeleOp;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.IntakeSpindexer_shreyas;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.TurretShooter_shreyas;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@TeleOp(name = "TeleOp_shreyas", group = "TeleOp")
public class Teleop_shreyas extends OpMode {
    Follower follower;
    TurretShooter_shreyas turretShooter;
    IntakeSpindexer_shreyas intakeSpindexer;

    private boolean XisPressed = false;
    private boolean YisPressed = false;
    private boolean BisPressed = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        // Important: Initialize Intake FIRST, then pass it to Shooter
        intakeSpindexer = new IntakeSpindexer_shreyas(this);
        turretShooter = new TurretShooter_shreyas(this, Alliance.RED, intakeSpindexer);
        telemetry.addData("Status", "Initialized");
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
                -gamepad1.right_stick_x * 0.4,
                true
        );

        // --- SUBSYSTEM UPDATES ---
        turretShooter.updateTurretTracking();
        intakeSpindexer.update();

        // --- INTAKE TOGGLE (Button X) ---
        if (gamepad1.x && !XisPressed) {
            XisPressed = true;
            if (intakeSpindexer.state == IntakeSpindexer_shreyas.IntakeState.INTAKING) { // if its running and clicked again, it stops
                intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.IDLE);
            } else {
                if (intakeSpindexer.getBallCount() < 3) { // keeps running if more balls can be loaded / till 3 balls are loaded
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.INTAKING);
                }
            }
        } else if (!gamepad1.x) {
            XisPressed = false;
        }

        // --- INDEXED SHOOT BUTTON (Y) ---
        // Trigger the PPG / PGP / GPP sequence (endgame Teleop)
        if (gamepad1.y && !YisPressed) {
            YisPressed = true;
            turretShooter.shootIndexed(TurretShooter_shreyas.ShootCase.PPG);
            /* TODO: when we find the order (PPG/PGP/GPP) in auto, get the value into teleup and change
            *   the parameter above based on the parameter from AUTO*/
        } else if (!gamepad1.y) {
            YisPressed = false;
        }

        // --- MANUAL SHOOT BUTTON (B) ---
        // Regular shoot for teleop
        if (gamepad1.b && !BisPressed) {
            BisPressed = true;
            turretShooter.shoot(2500, 0.95);
            /*TODO: add the kicker/spindexer stuff for 3 ball shoot (not indexed);
               after the 3 ball shoot routine, stop shooter --> reset the array list*/
            //intakeSpindexer.ballColors.clear();
        } else if (!gamepad1.b) {
            BisPressed = false;
        }

        // --- TELEMETRY ---
        telemetry.addData("Balls Loaded", intakeSpindexer.getBallCount());
        telemetry.addData("Intake State", intakeSpindexer.hasBalls());
        telemetry.update();
    }

    @Override
    public void stop() {
        turretShooter.stop();
    }
}