package org.firstinspires.ftc.teamcode.Robot.Decode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.IntakeSpindexer;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.TurretShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Close")
public class BlueClose extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    IntakeSpindexer intakeSpindexer;
    TurretShooter turretShooter;

    Alliance alliance = Alliance.BLUE;
    boolean shoot = false;
    boolean ready = false;
    private int pathState;

    private final Pose startPose = new Pose(29.1, 132.2, Math.toRadians(180));

    private PathChain shoot1;

    public void buildPaths() {
        shoot1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(29.1 , 132.2),
                                new Pose(40.1, 95)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                    setPathState(1);
                    follower.followPath(shoot1, 0.9, true);
                }
                break;
            case 1:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    setPathState(3);
                    shoot = true;
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    shoot = false;
                    if(pathTimer.getElapsedTimeSeconds() > 0.9) {
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    intakeSpindexer.manualOverrideSpindexerBlock(true);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        intakeSpindexer.update(ready, shoot);
        turretShooter.update(follower.getPose(), follower.getVelocity(), shoot, false, false, true);

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Turret Pos", turretShooter.getServo(TurretShooter.ServoNames.turret1).getPosition());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        intakeSpindexer = new IntakeSpindexer(this, false);
        turretShooter   = new TurretShooter(this, alliance);

        intakeSpindexer.manualOverrideSpindexerBlock(true);
        intakeSpindexer.setSpindexerBlockPosition();

        turretShooter.hoodPos = 0.8;
        turretShooter.shooterVel = 1650;
    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }

}