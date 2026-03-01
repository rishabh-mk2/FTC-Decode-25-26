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
    private Timer pathTimer, actionTimer, opmodeTimer;
    IntakeSpindexer intakeSpindexer;
    TurretShooter turretShooter;

    Alliance alliance = Alliance.BLUE;
    boolean shoot = false;
    boolean ready = false;
    boolean enableTurret = false;
    boolean enableShooter = false;
    private int pathState;

    private final Pose startPose = new Pose(29.1, 132.2, Math.toRadians(180));

    private PathChain shoot1, intake1, shoot2, intake2_1, intake2_2, shoot3, intake3, shoot4, park;

    public void buildPaths() {
        shoot1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(33.861, 135.935),
                                new Pose(47.380, 92.816)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intake1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(47.380, 92.816),
                                new Pose(49.814, 53),
                                new Pose(13.531, 55)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shoot2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(13.531, 55),
                                new Pose(53, 70)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intake2_1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(53, 70),
                                new Pose(10, 61.5)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(158)) // was 152
                .build();
        intake2_2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(10, 61.5),
                                new Pose (2, 61.5)//was 2
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(158), Math.toRadians(158)) // was 152
                .build();
        shoot3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose (2, 61.5),//was 2
                                new Pose(49, 75.208)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(158), Math.toRadians(158)) // was 152
                .build();
        intake3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(49, 75.208),
                                new Pose(10, 78)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(158), Math.toRadians(180))//152
                .build();
        shoot4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(10, 78),
                                new Pose(48, 78)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        park = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(48, 78),
                                new Pose(25, 70)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                    setPathState(1);
                    intakeSpindexer.manualOverrideSpindexerBlock(true);
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
                if(follower.getPose().getX() > 45) {
                    intakeSpindexer.manualOverrideSpindexerBlock(true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 0.2) {
                        setPathState(9);
                    }
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    shoot = true;
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    shoot = false;
                    if(pathTimer.getElapsedTimeSeconds() > 0.75) {
                        setPathState(11);
                        intakeSpindexer.manualOverrideSpindexerBlock(false);
                    }
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    follower.followPath(intake2_1, 0.8, true);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    follower.followPath(intake2_2, 0.4, true);
                    setPathState(13);
                }
                break;
            case 13:
                if(!follower.isBusy()) {
                    if(intakeSpindexer.getBallCount() == 3 || pathTimer.getElapsedTimeSeconds() > 4.0) {
                        follower.followPath(shoot3, 0.8, true);
                        setPathState(14);
                    }
                }
                break;
            case 14:
                if(follower.getPose().getX() > 40) {
                    intakeSpindexer.manualOverrideSpindexerBlock(true);
                    setPathState(15);
                }
                break;
            case 15:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                        shoot = true;
                        setPathState(16);
                    }
                }
                break;
            case 16:
                if(!follower.isBusy()) {
                    shoot = false;
                    if(pathTimer.getElapsedTimeSeconds() > 0.1) {
                        setPathState(17);
                    }
                }
                break;
            case 17:
                if(!follower.isBusy()) {
                    intakeSpindexer.manualOverrideSpindexerBlock(false);
                    follower.followPath(intake2_1, 0.95, true);
                    setPathState(18);
                }
                break;
            case 18:
                if(!follower.isBusy()) {
                    follower.followPath(intake2_2, 0.4, true);
                    setPathState(19);
                }
                break;
            case 19:
                if(!follower.isBusy()) {
                    if(intakeSpindexer.getBallCount() == 3 || pathTimer.getElapsedTimeSeconds() > 4.0) {
                        follower.followPath(shoot3, 0.95, true);
                        setPathState(20);
                    }
                }
                break;
            case 20:
                if(follower.getPose().getX() > 40) {
                    intakeSpindexer.manualOverrideSpindexerBlock(true);
                    setPathState(21);
                }
                break;
            case 21:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 0.1) {
                        shoot = true;
                        setPathState(22);
                    }
                }
                break;
            case 22:
                if(!follower.isBusy()) {
                    shoot = false;
                    if(pathTimer.getElapsedTimeSeconds() > 0.75) {
                        setPathState(23);
                        intakeSpindexer.manualOverrideSpindexerBlock(false);
                    }
                }
                break;
            case 23:
                if(!follower.isBusy()) {
                    follower.followPath(intake3, 0.75, true);
                    setPathState(24);
                }
                break;
            case 24:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 0.2) {
                        follower.followPath(shoot4, 0.9, true);
                        setPathState(25);
                    }
                }
                break;
            case 25:
                if(!follower.isBusy()) {
                    intakeSpindexer.manualOverrideSpindexerBlock(true);
                    if(pathTimer.getElapsedTimeSeconds() > 1.75) {
                        setPathState(26);
                    }
                }
                break;
            case 26:
                if(!follower.isBusy()) {
                    shoot = true;
                    setPathState(27);
                }
                break;
            case 27:
                if(!follower.isBusy()) {
                    shoot = false;
                    if(pathTimer.getElapsedTimeSeconds() > 0.75) {
                        setPathState(28);
                        intakeSpindexer.manualOverrideSpindexerBlock(false);
                    }
                }
                break;
            case 28:
                if(!follower.isBusy()) {
                    follower.followPath(park, 1.0, true);
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