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

@Autonomous(name = "Red Close")
public class RedClose extends OpMode {

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

    private final Pose startPose = new Pose(112.4, 132.2, Math.toRadians(90));

    private PathChain shoot1, intake1, shoot2, intake2_1, intake2_2, shoot3;

    public void buildPaths() {
        shoot1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(112.4, 132.2),
                                new Pose(105, 95)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-38))
                .build();
        intake1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(105.000, 95),
                                new Pose(102.400, 65.624),
                                new Pose(127, 71.5)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        shoot2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(127, 71.5),
                                new Pose(96, 86)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(20))
                .build();
        intake2_1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(96, 86),
                                new Pose (103.4967, 68),
                                new Pose(128, 67.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(31))
                .setBrakingStart(25)
                .setBrakingStrength(4.0)
                .build();
        intake2_2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(128, 67.5),
                                new Pose(136, 67.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(32))
                .setBrakingStart(25)
                .setBrakingStrength(4.0)
                .build();
        shoot3 = shoot2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(135, 67.5),
                                new Pose(96, 86)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(32))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                    setPathState(1);
                    follower.followPath(shoot1, 1.0, true);
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
                    if(pathTimer.getElapsedTimeSeconds() > 0.75) {
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    turretShooter.hoodPos = 0.84;
                    turretShooter.shooterVel = 1650;
                    setPathState(5);
                    intakeSpindexer.manualOverrideSpindexerBlock(false);
                    follower.followPath(intake1, 0.75, true);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                        setPathState(6);
                    }
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(shoot2, 1.0, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    intakeSpindexer.manualOverrideSpindexerBlock(true);
                    if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                        setPathState(8);
                    }
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    shoot = true;
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    shoot = false;
                    if(pathTimer.getElapsedTimeSeconds() > 0.75) {
                        setPathState(10);
                        intakeSpindexer.manualOverrideSpindexerBlock(false);
                    }
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    follower.followPath(intake2_1, 0.8, true);
                    setPathState(25);
                }
                break;
            case 25:
                if(!follower.isBusy()) {
                    follower.followPath(intake2_2, 0.4, true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    if(intakeSpindexer.getBallCount() == 3 || pathTimer.getElapsedTimeSeconds() > 4.5) {
                        follower.followPath(shoot3, 0.8, true);
                        setPathState(12);
                    }
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    intakeSpindexer.manualOverrideSpindexerBlock(true);
                    if(pathTimer.getElapsedTimeSeconds() > 0.75) {
                        setPathState(13);
                    }
                }
                break;
            case 13:
                if(!follower.isBusy()) {
                    shoot = true;
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy()) {
                    shoot = false;
                    if(pathTimer.getElapsedTimeSeconds() > 0.75) {
                        setPathState(15);
                        intakeSpindexer.manualOverrideSpindexerBlock(false);
                    }
                }
                break;
            case 15:
                if(!follower.isBusy()) {
                    follower.followPath(intake2_1, 0.95, true);
                    setPathState(26);
                }
                break;
            case 26:
                if(!follower.isBusy()) {
                    follower.followPath(intake2_2, 0.4, true);
                    setPathState(16);
                }
                break;
            case 16:
                if(!follower.isBusy()) {
                    if(intakeSpindexer.getBallCount() == 3 || pathTimer.getElapsedTimeSeconds() > 4.5) {
                        follower.followPath(shoot3, 0.95, true);
                        setPathState(17);
                    }
                }
                break;
            case 17:
                if(!follower.isBusy()) {
                    intakeSpindexer.manualOverrideSpindexerBlock(true);
                    if(pathTimer.getElapsedTimeSeconds() > 0.75) {
                        setPathState(18);
                    }
                }
                break;
            case 18:
                if(!follower.isBusy()) {
                    shoot = true;
                    setPathState(19);
                }
                break;
            case 19:
                if(!follower.isBusy()) {
                    shoot = false;
                    if(pathTimer.getElapsedTimeSeconds() > 0.75) {
                        setPathState(20);
                        intakeSpindexer.manualOverrideSpindexerBlock(false);
                    }
                }
                break;
            case 20:
                if(!follower.isBusy()) {
                    follower.followPath(intake2_1, 0.95, true);
                    setPathState(27);
                }
                break;
            case 27:
                if(!follower.isBusy()) {
                    follower.followPath(intake2_2, 0.4, true);
                    setPathState(21);
                }
                break;
            case 21:
                if(!follower.isBusy()) {
                    if(intakeSpindexer.getBallCount() == 3 || pathTimer.getElapsedTimeSeconds() > 4.5) {
                        follower.followPath(shoot3, 0.95, true);
                        setPathState(22);
                    }
                }
                break;
            case 22:
                if(!follower.isBusy()) {
                    intakeSpindexer.manualOverrideSpindexerBlock(true);
                    if(pathTimer.getElapsedTimeSeconds() > 0.75) {
                        setPathState(23);
                    }
                }
                break;
            case 23:
                if(!follower.isBusy()) {
                    shoot = true;
                    setPathState(24);
                }
                break;
            case 24:
                if(!follower.isBusy()) {
                    shoot = false;
                    if(pathTimer.getElapsedTimeSeconds() > 0.75) {
                        setPathState(-1);
                        intakeSpindexer.manualOverrideSpindexerBlock(false);
                    }
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