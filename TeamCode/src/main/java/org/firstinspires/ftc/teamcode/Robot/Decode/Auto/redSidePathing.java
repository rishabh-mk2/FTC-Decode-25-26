package org.firstinspires.ftc.teamcode.Robot.Decode.Auto;

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

@Autonomous(name = "Red Side Pathing", group = "Autonomous")

public class redSidePathing extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private double speed = 1.0;
    TurretShooter_shreyas turretShooter;
    IntakeSpindexer_shreyas intakeSpindexer;

    // Tracking flags for different phases
    private boolean hasStartedShooting = false;
    private boolean hasStartedIntaking = false;
    private boolean hasCalledShootFunction = false;

    /* ---------- Poses ---------- */
    private final Pose startPose   = new Pose(122, 122, Math.toRadians(270));
    private final Pose scorePose1  = new Pose(98, 98, Math.toRadians(270));
    private final Pose scorePose2  = new Pose(94, 79, Math.toRadians(0));
    private final Pose scorePose3  = new Pose(89, 74, Math.toRadians(270));
    private final Pose scorePose4  = new Pose(85, 73, Math.toRadians(270));
    private final Pose last = new Pose(88, 66, Math.toRadians(270));

    private final Pose pickup1Pose = new Pose(130, 73, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(128, 43.5, Math.toRadians(270));
    private final Pose pickup3Pose = new Pose(121, 35, Math.toRadians(307));
    private final Pose releasePose = new Pose(132, 70, Math.toRadians(0));
    private final Pose pushReleasePose = new Pose(138.5, 69, Math.toRadians(0));

    /* ---------- Paths ---------- */
    private PathChain scorePreload;
    //private PathChain pickup1, releasePreload, pushReleaseHook, scorePickup1;
    private PathChain pickup1, ScorePickup1;
    private PathChain pickup2, scorePickup2;
    private PathChain pickup3, scorePickup3, leaveTriangle;

    /* ---------- Build Paths ---------- */
    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(122, 122), scorePose1))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                .build();

        pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose1, pickup1Pose))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))
                .build();

        /*releasePreload = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, releasePose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        pushReleaseHook = follower.pathBuilder()
                .addPath(new BezierLine(releasePose, pushReleasePose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pushReleasePose, scorePose2))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(330))
                .build();*/
        ScorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose2))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(330))
                .build();

        pickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, pickup2Pose))
                .setLinearHeadingInterpolation(Math.toRadians(330), Math.toRadians(340))
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose3))
                .setLinearHeadingInterpolation(Math.toRadians(340), Math.toRadians(308))
                .build();

        pickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose3, pickup3Pose))
                .setLinearHeadingInterpolation(Math.toRadians(308), Math.toRadians(308))
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose4))
                .setLinearHeadingInterpolation(Math.toRadians(308), Math.toRadians(308))
                .build();

        leaveTriangle = follower.pathBuilder()
                .addPath(new BezierLine(scorePose4, last))
                .setLinearHeadingInterpolation(Math.toRadians(308), Math.toRadians(308))
                .build();
    }

    /* ---------- State Machine ---------- */
    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                // Start following path
                if (!hasStartedShooting) {
                    follower.followPath(scorePreload, 0.5, true);
                    hasStartedShooting = true;
                }
                if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    turretShooter.getMotor(TurretShooter_shreyas.MotorNames.leftShooter).setVelocity(400);
                    turretShooter.getMotor(TurretShooter_shreyas.MotorNames.rightShooter).setVelocity(400);
                }

                // Once path is complete, call shooting function ONCE
                if (!follower.isBusy() && !hasCalledShootFunction) {
//                    turretShooter.simpleShootSequence(400, 0.5, 0,750, intakeSpindexer);
                    hasCalledShootFunction = true;
                }

                // Once shooting is complete (ballsLoaded == 0), move to next case
                if (intakeSpindexer.ballsLoaded == 0) {
                    setPathState(1);
                }
                break;

            case 1:
                // Start path immediately
                if (!hasStartedIntaking) {
                    follower.followPath(pickup1, 0.3, true); // TODO; changeed from 0.3
                }

                // Start intaking 1 second after case starts
                if (pathTimer.getElapsedTimeSeconds() >= 1.0 && !hasStartedIntaking) {
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.INTAKING);
                    hasStartedIntaking = true;
                }

                // Once path complete, wait 500ms then switch
                if (!follower.isBusy() && hasStartedIntaking && pathTimer.getElapsedTimeSeconds() >= 1.5) {
                    setPathState(4);
                }
                break;

            /*case 2:
                if (!follower.isBusy()) {
                    follower.followPath(releasePreload, speed, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(pushReleaseHook, 0.6, true);
                }
                if (pathTimer.getElapsedTimeSeconds() >= 1) {
                    setPathState(4);
                }
                break;*/

            case 4:
                // Start following path to score position
                if (!hasStartedShooting) {
                    if (intakeSpindexer.ballsLoaded != 3) {
                        intakeSpindexer.rotateSpindexer60(1);
                    }
                    follower.followPath(ScorePickup1, 0.5, true);
                    hasStartedShooting = true;
                }

                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    turretShooter.getMotor(TurretShooter_shreyas.MotorNames.leftShooter).setVelocity(400);
                    turretShooter.getMotor(TurretShooter_shreyas.MotorNames.rightShooter).setVelocity(400);
                }

                // Once path complete, call shooting function ONCE
                if (!follower.isBusy() && !hasCalledShootFunction) {
//                    turretShooter.shootIndexed(TurretShooter_shreyas.ShootCase.PPG, 400, 0.75); // Adjust shoot case as needed
                    hasCalledShootFunction = true;
                }

                // Once shooting complete (check ballsLoaded), move to next case
                if (intakeSpindexer.ballsLoaded == 0) {
                    setPathState(5);
                }
                break;

            case 5:
                // Start path immediately
                if (!hasStartedIntaking) {
                    follower.followPath(pickup2, 0.3, true);
                }

                // Start intaking 1 second after case starts
                if (pathTimer.getElapsedTimeSeconds() >= 1.0 && !hasStartedIntaking) {
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.INTAKING);
                    hasStartedIntaking = true;
                }

                // Once path complete, wait 500ms then switch
                if (!follower.isBusy() && hasStartedIntaking && pathTimer.getElapsedTimeSeconds() >= 7.5) {
                    setPathState(6);
                }
                break;

            case 6:
                // Start following path to score position
                if (!hasStartedShooting) {
                    if (intakeSpindexer.ballsLoaded != 3) {
                        intakeSpindexer.rotateSpindexer60(1);
                    }
                    follower.followPath(scorePickup2, 0.5, true);
                    hasStartedShooting = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    turretShooter.getMotor(TurretShooter_shreyas.MotorNames.leftShooter).setVelocity(400);
                    turretShooter.getMotor(TurretShooter_shreyas.MotorNames.rightShooter).setVelocity(400);
                }

                // Once path complete, call shooting function ONCE
                if (!follower.isBusy() && !hasCalledShootFunction) {
//                    turretShooter.shootIndexed(TurretShooter_shreyas.ShootCase.PPG, 400, 0.75); // Adjust shoot case as needed
                    hasCalledShootFunction = true;
                }

                // Once shooting complete (check ballsLoaded), move to next case
                if (intakeSpindexer.ballsLoaded == 0 && hasCalledShootFunction) {
                    setPathState(-1);
                }
                break;

            case 7:
                // Start path immediately
                if (!hasStartedIntaking) {
                    follower.followPath(pickup3, speed, true);
                }

                // Start intaking 1 second after case starts
                if (pathTimer.getElapsedTimeSeconds() >= 1.0 && !hasStartedIntaking) {
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.INTAKING);
                    hasStartedIntaking = true;
                }

                // Once path complete, wait 500ms then switch
                if (!follower.isBusy() && hasStartedIntaking && pathTimer.getElapsedTimeSeconds() >= 1.5) {
                    setPathState(8);
                }
                break;

            case 8:
                // Start following path to score position
                if (!hasStartedShooting) {
                    follower.followPath(scorePickup3, speed, true);
                    hasStartedShooting = true;
                }

                // Once path complete, call shooting function ONCE
                if (!follower.isBusy() && !hasCalledShootFunction) {
//                    turretShooter.shootIndexed(TurretShooter_shreyas.ShootCase.PPG, 1200, 0.75); // Adjust shoot case as needed
                    hasCalledShootFunction = true;
                }

                // Once shooting complete (check ballsLoaded), move to next case
                if (intakeSpindexer.ballsLoaded == 0 && hasCalledShootFunction) {
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(leaveTriangle, speed, true);
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
        hasStartedIntaking = false; // Reset flag when changing states
        hasStartedShooting = false; // Reset shooting flag when changing states
        hasCalledShootFunction = false; // Reset function call flag when changing states
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Initialize the subsystems
        intakeSpindexer = new IntakeSpindexer_shreyas(this, false);
        turretShooter = new TurretShooter_shreyas(this, Alliance.RED);
        intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin1).setPosition(0.097);
        intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin2).setPosition(0.097);
        while (gamepad1.aWasPressed()) {
            intakeSpindexer.getMotor(IntakeSpindexer_shreyas.MotorNames.intake).setPower(0.6);
        }
        buildPaths();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        intakeSpindexer.ballsLoaded = 3;
        intakeSpindexer.state = IntakeSpindexer_shreyas.IntakeState.IDLE;
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        intakeSpindexer.update(System.currentTimeMillis());
        turretShooter.trackAprilTag();
        autonomousPathUpdate();

        // Stop shooter motors when out of balls
        if (intakeSpindexer.ballsLoaded == 0) {
            turretShooter.getMotor(TurretShooter_shreyas.MotorNames.leftShooter).setVelocity(0);
            turretShooter.getMotor(TurretShooter_shreyas.MotorNames.rightShooter).setVelocity(0);
        }
        if (intakeSpindexer.ballsLoaded > 0 &&
                intakeSpindexer.state == IntakeSpindexer_shreyas.IntakeState.IDLE) {
            intakeSpindexer.getMotor(IntakeSpindexer_shreyas.MotorNames.intake).setPower(0.8);
        }

        telemetry.addData("Path State", pathState);
        telemetry.addData("Color Array", intakeSpindexer.ballColors.toString());
        telemetry.addData("Balls Loaded", intakeSpindexer.ballsLoaded);
        telemetry.addData("Has Started Shooting", hasStartedShooting);
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }
}