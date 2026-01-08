package org.firstinspires.ftc.teamcode.Robot.Decode.Auto.Rishabh; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
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

    private int pathState = 0;

    // START POSE
    private final Pose startPose = new Pose(95, 12, Math.toRadians(45));



    private PathChain intake1_1, intake1_2, shoot1, intake2_1, intake2_2, intake2_3, shoot2, intake3, shoot3, intake4, shoot4, intake5, shoot5;

    public void buildPaths() {
        intake1_1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(95.000, 12.000),
                                new Pose(107, 29.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();
        intake1_2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(107, 29.5),
                                new Pose(124.5, 29.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();

        shoot1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(124.5, 29.5),
                                new Pose(93, 14)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();
        intake2_1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(95, 12),
                                new Pose(116.5, 18)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(-30))
                .build();
        intake2_2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(116.5, 18),
                                new Pose(116.5, 12)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(-30))
                .build();

        intake2_3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(116.5, 12),
                                new Pose(116.5, 9.5)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(-30))
                .build();

        shoot2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(116.5, 9.5),
                                new Pose(90, 9)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(15))
                .build();

        intake3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(90, 9),
                                new Pose(125, 6.5)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        shoot3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125, 6.5),
                                new Pose(90, 6.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        intake4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(90, 6.5),
                                new Pose(125, 6.5)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        shoot4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125, 6.5),
                                new Pose(90, 6.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        intake5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(90, 6.5),
                                new Pose(130, 24)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))
                .build();

        shoot5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(130, 24),
                                new Pose(90, 12)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(15))
                .build();
    }
    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                if(!follower.isBusy()) {
                    turretShooter.shoot(1500, 100);
                    setPathState(1);
                }
                break;
            case 1:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTime() > 1000) {
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    intakeSpindexer.kickstartShootChain = true;
                    intakeSpindexer.shootDone = false;
                    intakeSpindexer.shootCallTime = opmodeTimer.getElapsedTime();
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.SHOOTING);
                    if(pathTimer.getElapsedTime() > 100 + 100 + 400 + 100 + 100 + 400 + 100 + 100) {
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    turretShooter.shoot(500, 1.0);
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.INTAKING);
                    follower.followPath(intake1_1, 0.8, false);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(intake1_2, 0.4, true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(shoot1, 0.8, true);
                    setPathState(7);
                    turretShooter.shoot(1500, 1.0);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTime() > 1000) {
                        setPathState(8);
                    }
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    // TODO: REMOVE LINE BELOW LATER
                    intakeSpindexer.ballsLoaded = 3;
                    intakeSpindexer.kickstartShootChain = true;
                    intakeSpindexer.shootDone = false;
                    intakeSpindexer.shootCallTime = opmodeTimer.getElapsedTime();
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.SHOOTING);
                    if(pathTimer.getElapsedTime() > 100 + 100 + 400 + 100 + 100 + 400 + 100 + 100) {
                        setPathState(10);
                    }
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    turretShooter.shoot(500, 1.0);
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.INTAKING);
                    follower.followPath(intake2_1, 0.8, true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    follower.followPath(intake2_2, 0.35, true);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    follower.followPath(intake2_3, 0.35, true);
                    setPathState(13);
                }
                break;
            case 13:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTime() > 500) {
                        setPathState(14);
                    }
                }
                break;
            case 14:
                if(!follower.isBusy()) {
                    turretShooter.shoot(1500, 1.0);
                    follower.followPath(shoot2, 0.8, true);
                    setPathState(15);
                }
                break;
            case 15:
                if(!follower.isBusy()) {
                    // TODO: REMOVE LINE BELOW LATER
                    intakeSpindexer.ballsLoaded = 3;
                    intakeSpindexer.kickstartShootChain = true;
                    intakeSpindexer.shootDone = false;
                    intakeSpindexer.shootCallTime = opmodeTimer.getElapsedTime();
                    setPathState(16);
                }
                break;
            case 16:
                if(!follower.isBusy()) {
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.SHOOTING);
                    if(pathTimer.getElapsedTime() > 100 + 100 + 400 + 100 + 100 + 400 + 100 + 100) {
                        setPathState(17);
                    }
                }
                break;
            case 17:
                if(!follower.isBusy()) {
                    turretShooter.shoot(500, 1.0);
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.INTAKING);
                    follower.followPath(intake3, 0.8, true);
                    setPathState(18);
                }
                break;
            case 18:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTime() > 500) {
                        setPathState(19);
                    }
                }
                break;
            case 19:
                if(!follower.isBusy()) {
                    turretShooter.shoot(1500, 1.0);
                    follower.followPath(shoot3, 0.8, true);
                    setPathState(20);
                }
                break;
            case 20:
                if(!follower.isBusy()) {
                    // TODO: REMOVE LINE BELOW LATER
                    intakeSpindexer.ballsLoaded = 3;
                    intakeSpindexer.kickstartShootChain = true;
                    intakeSpindexer.shootDone = false;
                    intakeSpindexer.shootCallTime = opmodeTimer.getElapsedTime();
                    setPathState(21);
                }
                break;
            case 21:
                if(!follower.isBusy()) {
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.SHOOTING);
                    if(pathTimer.getElapsedTime() > 100 + 100 + 400 + 100 + 100 + 400 + 100 + 100) {
                        setPathState(22);
                    }
                }
                break;
            case 22:
                if(!follower.isBusy()) {
                    turretShooter.shoot(500, 1.0);
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.INTAKING);
                    follower.followPath(intake4, 0.8, true);
                    setPathState(23);
                }
                break;

            case 23:
                if(!follower.isBusy()) {
                    if (pathTimer.getElapsedTime() > 500) {
                        setPathState(24);
                    }
                }
                break;
            case 24:
                if(!follower.isBusy()) {
                    turretShooter.shoot(1500, 1.0);
                    follower.followPath(shoot4, 0.8, true);
                    setPathState(25);
                }
                break;
            case 25:
                if(!follower.isBusy()) {
                    // TODO: REMOVE LINE BELOW LATER
                    intakeSpindexer.ballsLoaded = 3;
                    intakeSpindexer.kickstartShootChain = true;
                    intakeSpindexer.shootDone = false;
                    intakeSpindexer.shootCallTime = opmodeTimer.getElapsedTime();
                    setPathState(26);
                }
                break;
            case 26:
                if(!follower.isBusy()) {
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.SHOOTING);
                    if(pathTimer.getElapsedTime() > 100 + 100 + 400 + 100 + 100 + 400 + 100 + 100) {
                        setPathState(27);
                    }
                }
                break;
            case 27:
                if(!follower.isBusy()) {
                    turretShooter.shoot(500, 1.0);
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.INTAKING);
                    follower.followPath(intake5, 0.8, true);
                    setPathState(28);
                }
                break;

            case 28:
                if(!follower.isBusy()) {
                    if (pathTimer.getElapsedTime() > 3500) {
                        setPathState(29);
                    }
                }
                break;
            case 29:
                if(!follower.isBusy()) {
                    turretShooter.shoot(1500, 1.0);
                    follower.followPath(shoot5, 0.8, true);
                    setPathState(30);
                }
                break;
            case 30:
                if(!follower.isBusy()) {
                    // TODO: REMOVE LINE BELOW LATER
                    intakeSpindexer.ballsLoaded = 3;
                    intakeSpindexer.kickstartShootChain = true;
                    intakeSpindexer.shootDone = false;
                    intakeSpindexer.shootCallTime = opmodeTimer.getElapsedTime();
                    setPathState(31);
                }
                break;
            case 31:
                if(!follower.isBusy()) {
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.SHOOTING);
                    if(pathTimer.getElapsedTime() > 100 + 100 + 400 + 100 + 100 + 400 + 100 + 100) {
                        setPathState(-1);
                    }
                }
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
        telemetry.addData("Path Time", pathTimer.getElapsedTime());
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
        intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin1).setPosition(0.097);
        intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin2).setPosition(0.097);
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