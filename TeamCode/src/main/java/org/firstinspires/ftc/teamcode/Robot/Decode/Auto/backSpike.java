package org.firstinspires.ftc.teamcode.Robot.Decode.Auto; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.IntakeSpindexer_shreyas;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.TurretShooter_shreyas;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Back Spike", group = "Autonomous")
public class backSpike extends OpMode {

    private Follower follower;
    IntakeSpindexer_shreyas intakeSpindexer;
    TurretShooter_shreyas turretShooter;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    // START POSE
    private final Pose startPose = new Pose(95, 12, Math.toRadians(90));



    private PathChain intake1;

    public void buildPaths() {
        intake1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(95.000, 12.000),
                                new Pose(95, 60)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

    }
    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
//                turretShooter.shoot(500, 1.0);
                intakeSpindexer.kickstartShootChain = true;
                intakeSpindexer.shootDone = false;
                intakeSpindexer.shootCallTime = pathTimer.getElapsedTime();
                setPathState(1);
                break;
            case 1:
                if(pathTimer.getElapsedTime() > 1000) {
                    setPathState(2);
                }
                break;
            case 2:
                intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.SHOOTING);
                if(pathTimer.getElapsedTime() > 100 + 100 + 400 + 100 + 100 + 400 + 100 + 100) {
                    setPathState(3);
                }
                break;
            case 3:
//                turretShooter.shoot(500, 1.0);
                follower.followPath(intake1, 1.0, true);
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        intakeSpindexer.update(opmodeTimer.getElapsedTime());
        turretShooter.trackAprilTag();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        intakeSpindexer = new IntakeSpindexer_shreyas(this, true);
        turretShooter = new TurretShooter_shreyas(this, Alliance.RED);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        intakeSpindexer.ballsLoaded = 3;
        intakeSpindexer.rotateSpindexer60(1);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {

    }

}