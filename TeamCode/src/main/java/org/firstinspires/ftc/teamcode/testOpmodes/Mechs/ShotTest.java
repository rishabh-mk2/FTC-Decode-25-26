package org.firstinspires.ftc.teamcode.testOpmodes.Mechs;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.TurretShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Vector;


@Config
@TeleOp(name = "Shooting w/ Vel Compensation", group = "TeleOp")
public class ShotTest extends OpMode {

    Follower follower;
    TurretShooter turretShooter;

    Pose startPose = new Pose(0, 0, Math.toRadians(0));

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(startPose);

        turretShooter = new TurretShooter(this, Alliance.RED);

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
    }

    @Override
    public void loop() {
        follower.update();
        turretShooter.updateValues(follower.getPose(), follower.getVelocity());
        turretShooter.updateTurret();

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x * 0.58,
                true
        );
    }

    @Override
    public void stop() {

    }
}
