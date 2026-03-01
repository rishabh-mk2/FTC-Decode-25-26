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

@Autonomous(name = "Blue Far Detection")
public class BlueFarDetection extends OpMode {

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

    private final Pose startPose = new Pose(101.875, 8.1, Math.toRadians(0)).mirror();

    private PathChain intake1_1, intake1_2, intake1_3,intake1_4, shoot1, intake2, shoot2, intake3, shoot3, intake4, shoot4, park;

    public void buildPaths() {
        intake1_1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(101.875, 8.1).mirror(),
                                new Pose(122, 7.9).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStart(40)
                .build();
        intake1_2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(122, 7.9).mirror(),
                                new Pose(133, 7.9).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intake1_3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(133, 7.9).mirror(),
                                new Pose(128, 7.9).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intake1_4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(128, 7.9).mirror(),
                                new Pose(133, 7.9).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


        shoot1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(133, 7.9).mirror(),
                                new Pose(91.75, 15.5).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStart(40)
                .setBrakingStrength(4.0)
                .build();
        intake2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(91.75, 15.5).mirror(),
                                new Pose(90.31836734693877, 40).mirror(),
                                new Pose(125, 39).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .setBrakingStart(40)
                .setBrakingStrength(4.0)
                .build();
        shoot2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125, 39).mirror(),
                                new Pose(91.5, 19).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStart(40)
                .setBrakingStrength(4.0)
                .build();
        intake3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(91.5, 19).mirror(),
                                new Pose(130, 12).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStart(40)
                .setBrakingStrength(4.0)
                .build();
        shoot3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(130, 12).mirror(),
                                new Pose(91.5, 12).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStart(40)
                .setBrakingStrength(4.0)
                .build();
        intake4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(91.5, 12).mirror(),
                                new Pose(130, 37).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStart(40)
                .setBrakingStrength(4.0)
                .build();
        shoot4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(130, 37).mirror(),
                                new Pose(91.5, 19).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStart(40)
                .setBrakingStrength(4.0)
                .build();
        park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(91.5, 12).mirror(),
                                new Pose(130, 12).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStart(40)
                .setBrakingStrength(4.0)
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                    if(turretShooter.getMotor(TurretShooter.MotorNames.leftShooter).getVelocity() > 2000) {
                        shoot = true;
                        setPathState(1);
                    }
                }
                break;
            case 1:
                if(!follower.isBusy()) {
                    shoot = false;
                    if(pathTimer.getElapsedTimeSeconds() > 0.8) {
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    intakeSpindexer.manualOverrideSpindexerBlock(false);
                    follower.followPath(intake1_1, 0.8, true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(intake1_2, 0.55, true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(intake1_3, 0.55, true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(intake1_4, 0.6, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 1.25) {
                        setPathState(8);
                    }
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    follower.followPath(shoot1, 0.8, true);
                    setPathState(16);
                }
                break;
            case 16:
                if(follower.getPose().getX() > 141.5 - 100) {
                    intakeSpindexer.manualOverrideSpindexerBlock(true);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 1.4) {
                        shoot = true;
                        setPathState(10);
                    }
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    shoot = false;
                    if(pathTimer.getElapsedTimeSeconds() > 0.8) {
                        setPathState(11);
                    }
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    intakeSpindexer.manualOverrideSpindexerBlock(false);
                    follower.followPath(intake2, 0.8, true);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 0.35) {
                        setPathState(13);
                    }
                }
                break;
            case 13:
                if(!follower.isBusy()) {
                    follower.followPath(shoot2, 1.0, true);
                    setPathState(17);
                }
                break;
            case 17:
                if(follower.getPose().getX() > 141.5 - 105) {
                    intakeSpindexer.manualOverrideSpindexerBlock(true);
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 1.2) {
                        shoot = true;
                        setPathState(15);
                    }
                }
                break;
            case 15:
                if(!follower.isBusy()) {
                    shoot = false;
                    if(pathTimer.getElapsedTimeSeconds() > 0.8) {
                        setPathState(18);
                    }
                }
                break;
            case 18:
                if(!follower.isBusy()) {
                    follower.followPath(intake3, 0.9, true);
                    intakeSpindexer.manualOverrideSpindexerBlock(false);
                    setPathState(19);
                }
                break;
            case 19:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 0.35) {
                        setPathState(20);
                    }
                }
                break;
            case 20:
                if(!follower.isBusy()) {
                    follower.followPath(shoot3, 0.9, true);
                    setPathState(21);
                }
                break;
            case 21:
                if(follower.getPose().getX() > 141.5 - 100) {
                    intakeSpindexer.manualOverrideSpindexerBlock(true);
                    setPathState(22);
                }
                break;
            case 22:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 1.2) {
                        shoot = true;
                        setPathState(23);
                    }
                }
                break;
            case 23:
                if(!follower.isBusy()) {
                    shoot = false;
                    if(pathTimer.getElapsedTimeSeconds() > 0.8) {
                        setPathState(24);
                    }
                }
                break;

            case 24:
                if(!follower.isBusy()) {
                    follower.followPath(intake4, 0.9, true);
                    intakeSpindexer.manualOverrideSpindexerBlock(false);
                    setPathState(25);
                }
                break;
            case 25:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 0.35) {
                        setPathState(26);
                    }
                }
                break;
            case 26:
                if(!follower.isBusy()) {
                    follower.followPath(shoot4, 0.9, true);
                    setPathState(27);
                }
                break;
            case 27:
                if(follower.getPose().getX() > 141.5 - 100) {
                    intakeSpindexer.manualOverrideSpindexerBlock(true);
                    setPathState(28);
                }
                break;
            case 28:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 1.2) {
                        shoot = true;
                        setPathState(29);
                    }
                }
                break;
            case 29:
                if(!follower.isBusy()) {
                    shoot = false;
                    if(pathTimer.getElapsedTimeSeconds() > 0.8) {
                        setPathState(30);
                    }
                }
                break;

            case 30:
                if(!follower.isBusy()) {
                    follower.followPath(intake3, 0.9, true);
                    intakeSpindexer.manualOverrideSpindexerBlock(false);
                    setPathState(31);
                }
                break;
            case 31:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 0.35) {
                        setPathState(32);
                    }
                }
                break;
            case 32:
                if(!follower.isBusy()) {
                    follower.followPath(shoot3, 0.9, true);
                    setPathState(33);
                }
                break;
            case 33:
                if(follower.getPose().getX() > 141.5 - 100) {
                    intakeSpindexer.manualOverrideSpindexerBlock(true);
                    setPathState(34);
                }
                break;
            case 34:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 1.2) {
                        shoot = true;
                        setPathState(35);
                    }
                }
                break;
            case 35:
                if(!follower.isBusy()) {
                    shoot = false;
                    if(pathTimer.getElapsedTimeSeconds() > 0.8) {
                        setPathState(36);
                    }
                }
                break;

            case 36:
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

        turretShooter.shooterVel = 2050;
        turretShooter.hoodPos = 1.0;

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