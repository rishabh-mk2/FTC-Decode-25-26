package org.firstinspires.ftc.teamcode.Robot.Decode.Auto;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Base Auto")
public class BaseAuto extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    Follower follower;
    IntakeSpindexer intakeSpindexer;
    TurretShooter turretShooter;

    @Override
    public void waitForStart() {

        super.waitForStart();
    }


    public void runOpMode() {
        // START OF AUTO INIT ROUTINE

        // Setup telemetry settings
        telemetry.setAutoClear(true);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initRobot(this, Alliance.RED);

        telemetry.setAutoClear(true);
        waitForStart();
        runtime.reset();

        new Thread(() -> {
            while (opModeIsActive()) {
                follower.update();
                turretShooter.updateTurretTracking();
                intakeSpindexer.update();
            }
        }).start();

        // END OF AUTO INIT ROUTINE
        if (opModeIsActive()) {

            // TODO: AUTO CODE HERE

            while (opModeIsActive()) {

            }
            requestOpModeStop();
            // END OF AUTO COMPLETION ROUTINE
        }
    }

    public void initRobot (OpMode opMode, Alliance alliance) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Constants.createFollower(hardwareMap).getPose());

        turretShooter = new TurretShooter(opMode, alliance); // change alliance if needed
        intakeSpindexer = new IntakeSpindexer(this);
    }

    public void pause(double milliSec) {
        long currentTime = System.currentTimeMillis();
        while (System.currentTimeMillis() < currentTime + milliSec && opModeIsActive()) {
            // waiting
        }
    }
}