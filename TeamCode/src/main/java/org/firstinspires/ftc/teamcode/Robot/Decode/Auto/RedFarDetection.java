package org.firstinspires.ftc.teamcode.Robot.Decode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.IntakeSpindexer;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.TurretShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Far Detectiom")
public class RedFarDetection extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    IntakeSpindexer intakeSpindexer;
    TurretShooter turretShooter;

    Alliance alliance = Alliance.RED;
    boolean shoot = false;
    boolean ready = false;
    boolean enableTurret = false;
    boolean enableShooter = false;
    private int pathState;

    private final Pose startPose = new Pose(101.875, 8.1, Math.toRadians(0));

    private PathChain intake1_1, intake1_2, intake1_3, shoot1;

    public void buildPaths() {
        intake1_1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(101.875, 8.1),
                                new Pose(130, 8.1)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .setGlobalDeceleration()
                .build();
        intake1_2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(130, 8.1),
                                new Pose(126, 8.1)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .setGlobalDeceleration()
                .build();
        intake1_3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(126, 8.1),
                                new Pose(130, 8.1)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .setGlobalDeceleration()
                .build();

        shoot1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(130, 8.1),
                                new Pose(101.875, 8.1)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .setGlobalDeceleration()
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                    turretShooter.setTurretPosition(0.66);
                    turretShooter.setShooterVelocity(2070);
                    turretShooter.setRecoil(0.28);
                    turretShooter.setHoodPosition(1.0);
                    if (turretShooter.getMotor(TurretShooter.MotorNames.leftShooter).getVelocity() > 1950) {
                        setPathState(1);
                        shoot = true;
                    }
                }
                break;
            case 1:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 0.55) {
                        setPathState(2);
                        shoot = false;
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(intake1_1, 0.5, true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 0.55) {
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(intake1_2, 0.5, true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(intake1_3, 0.5, true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(shoot1, 0.6, true);
                    ready = true;
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    shoot = true;
                    setPathState(-1);
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

        intakeSpindexer.update(shoot, ready);
        turretShooter.update(follower.getPose(), follower.getVelocity(), shoot, enableTurret, enableShooter);

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

        intakeSpindexer = new IntakeSpindexer(this);
        turretShooter   = new TurretShooter(this, alliance);

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