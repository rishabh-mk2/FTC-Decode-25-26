package org.firstinspires.ftc.teamcode.Robot.Decode.TeleOp;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.shreyas_IntakeSpindexer;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.shreyas_TurretShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Shreyas TeleOp", group = "TeleOp")
public class shreyasTeleOp extends OpMode {

    Follower follower;
    shreyas_TurretShooter turretShooter;
    shreyas_IntakeSpindexer intakeSpindexer;
    boolean intakeHeld = gamepad1.right_bumper;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(follower.getPose());

        turretShooter = new shreyas_TurretShooter(this, Alliance.RED);
        intakeSpindexer = new shreyas_IntakeSpindexer(this);
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

        // --- INTAKE ---
        intakeSpindexer.runIntake(intakeHeld);

        telemetry.addLine("TeleOp Running");
        telemetry.update();
    }

    @Override
    public void stop() {
        turretShooter.stop();
    }
}