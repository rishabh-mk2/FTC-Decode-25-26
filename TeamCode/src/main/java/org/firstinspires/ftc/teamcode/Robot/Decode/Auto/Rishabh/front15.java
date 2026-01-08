package org.firstinspires.ftc.teamcode.Robot.Decode.Auto.Rishabh; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.IntakeSpindexer_shreyas;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.TurretShooter_shreyas;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Front 15", group = "Autonomous")
public class front15 extends OpMode {

    private Follower follower;
    IntakeSpindexer_shreyas intakeSpindexer;
    TurretShooter_shreyas turretShooter;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState = 0;

    // START POSE
    private final Pose startPose = new Pose(127, 112.5, Math.toRadians(0));



    private PathChain shoot1, intake1_1, intake1_2, shoot2, intake2_1, intake2_2;

    public void buildPaths() {
        shoot1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(127, 112.5),
                                new Pose(96, 88.5)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        intake1_1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(96, 88.5),
                                new Pose(104, 60)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        intake1_2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(104, 60),
                                new Pose(120, 60)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        shoot2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(120, 60),
                                new Pose(88, 78)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        intake2_1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(88, 78),
                                new Pose(133, 61)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(27.75))
                .build();

    }
    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                if(!follower.isBusy()) {
                    turretShooter.shoot(1550, 0.7);
                    follower.followPath(shoot1, 1.0, true);
                    setPathState(1);
                }
                break;
            case 1:
                if(!follower.isBusy()) {
                    intakeSpindexer.kickstartShootChain = true;
                    intakeSpindexer.shootDone = false;
                    intakeSpindexer.shootCallTime = opmodeTimer.getElapsedTime();
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.SHOOTING);
                    if(pathTimer.getElapsedTime() > intakeSpindexer.shootTime) {
                        setPathState(3);
                    }
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    turretShooter.shoot(500, 1.0);
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.INTAKING);
                    follower.followPath(intake1_1, 1.0, false);
                    setPathState(4);
                }
                break;

            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(intake1_2, 0.3, true);
                    setPathState(5);
                }
                break;

            case 5:
                if(!follower.isBusy()) {
                    turretShooter.shoot(1600, 0.7);
                    follower.followPath(shoot2, 1.0, true);
                    setPathState(6);
                }
                break;

            case 6:
                if(!follower.isBusy()) {
                    intakeSpindexer.kickstartShootChain = true;
                    intakeSpindexer.shootDone = false;
                    intakeSpindexer.shootCallTime = opmodeTimer.getElapsedTime();
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.SHOOTING);
                    if(pathTimer.getElapsedTime() > intakeSpindexer.shootTime) {
                        setPathState(8);
                    }
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    turretShooter.shoot(500, 1.0);
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.INTAKING);
                    follower.followPath(intake2_1, 0.9, true);
                    setPathState(-1);
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