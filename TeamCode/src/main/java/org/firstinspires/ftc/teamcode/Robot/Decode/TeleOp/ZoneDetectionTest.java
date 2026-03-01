package org.firstinspires.ftc.teamcode.Robot.Decode.TeleOp;

import static org.firstinspires.ftc.teamcode.Robot.Decode.Robot.TurretShooter.targetVelocity;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.IntakeSpindexer;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.TurretShooter;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.ZoneDetector;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Config
@TeleOp(name = "Zone Detection Test", group = "TeleOp")
public class ZoneDetectionTest extends OpMode {

    ZoneDetector zoneDetector;

    @Override
    public void init() {

        zoneDetector = new ZoneDetector(this, Alliance.RED);

        telemetry.setAutoClear(true);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {

    }


    @Override
    public void loop() {

        telemetry.addData("Zone", zoneDetector.getZone());
        telemetry.update();
    }

    @Override
    public void stop() {
        zoneDetector.stop();
    }
}