package org.firstinspires.ftc.teamcode.Robot.Decode.Auto.Rishabh; // make sure this aligns with class location

import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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
    double targetVel = 0.0;
    double limelightOffset = 0.0;
    private PIDController controller;
    final double p = 0.75, i = 0, d = 0.05, f = 1.0;

    private PathChain shoot0, intake1_1, intake1_2, shoot1, intake2_1, intake2_2, intake2_3, shoot2, intake3, shoot3, intake4, shoot4, intake5, shoot5;

    public void buildPaths() {
        shoot0 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(95.000, 12.000),
                                new Pose(92, 15)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(35))
                .build();

        intake1_1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(92, 15),
                                new Pose(106, 28)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(45))
                .build();
        intake1_2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(106, 28),
                                new Pose(124.5, 28)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();

        shoot1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(124.5, 28),
                                new Pose(93, 15.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();
        intake2_1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(93, 15.5),
                                new Pose(127, 9.5)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        intake2_2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(127, 9.5),
                                new Pose(118, 5.5)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        intake2_3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(118, 5.5),
                                new Pose(125.25, 5.5)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        shoot2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125.25, 5.5),
                                new Pose(90, 9)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(15))
                .build();

        intake3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(90, 9),
                                new Pose(129.604, 9.724),
                                new Pose(134.759, 44.620)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        shoot3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(134.759, 44.620),
                                new Pose(90, 15)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(33.4))
                .build();

        intake4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(90, 15),
                                new Pose(129.604, 9.724),
                                new Pose(134.759, 44.620)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        shoot4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(134.759, 44.620),
                                new Pose(90, 15)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(33.4))
                .build();

        intake5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(90, 15),
                                new Pose(129.604, 9.724),
                                new Pose(134.759, 44.620)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        shoot5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(134.759, 44.620),
                                new Pose(90, 15)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(33.4))
                .build();
    }
    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                if(!follower.isBusy()) {
                    limelightOffset = 0.0;
                    targetVel = 1875;
                    follower.followPath(shoot0,1.0, true);
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
//                    intakeSpindexer.ballsLoaded = 3;
                    intakeSpindexer.kickstartShootChain = true;
                    intakeSpindexer.shootDone = false;
                    intakeSpindexer.shootCallTime = opmodeTimer.getElapsedTime();
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.SHOOTING);
                    if(pathTimer.getElapsedTime() > 150 + 150 + 400 + 150 + 150 + 400 + 150 + 150) {
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    targetVel = 1400;
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.INTAKING);
                    follower.followPath(intake1_1, 1.0, false);
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
                    follower.followPath(shoot1, 1.0, true);
                    setPathState(7);
                    targetVel = 1830;
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTime() > 1000) {
                        setPathState(8);
                        limelightOffset = -3;
                    }
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    // TODO: REMOVE LINE BELOW LATER
//                    intakeSpindexer.ballsLoaded = 3;
                    intakeSpindexer.kickstartShootChain = true;
                    intakeSpindexer.shootDone = false;
                    intakeSpindexer.shootCallTime = opmodeTimer.getElapsedTime();
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.SHOOTING);
                    if(pathTimer.getElapsedTime() > 150 + 150 + 400 + 150 + 150 + 400 + 150 + 150) {
                        setPathState(10);
                    }
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    targetVel = 1200;
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.INTAKING);
                    follower.followPath(intake2_1, 1.0, true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTime() > 750) {
                        setPathState(14);
                    }
                }
                break;
//            case 12:
//                if(!follower.isBusy()) {
//                    follower.followPath(intake2_3, 1.0, true);
//                    setPathState(13);
//                }
//                break;
//            case 13:
//                if(!follower.isBusy()) {
//                    if(pathTimer.getElapsedTime() > 200) {
//                        setPathState(14);
//                    }
//                }
//                break;
            case 14:
                if(!follower.isBusy()) {
                    targetVel = 1850;
                    limelightOffset = 1;
                    follower.followPath(shoot2, 1.0, true);
                    setPathState(32);
                }
                break;
            case 32:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTime() > 1000) {
                        setPathState(15);
                    }
                }
                break;
            case 15:
                if(!follower.isBusy()) {
                    // TODO: REMOVE LINE BELOW LATER
//                    intakeSpindexer.ballsLoaded = 3;
                    intakeSpindexer.kickstartShootChain = true;
                    intakeSpindexer.shootDone = false;
                    intakeSpindexer.shootCallTime = opmodeTimer.getElapsedTime();
                    setPathState(16);
                }
                break;
            case 16:
                if(!follower.isBusy()) {
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.SHOOTING);
                    if(pathTimer.getElapsedTime() > 150 + 150 + 400 + 150 + 150 + 400 + 150 + 150) {
                        setPathState(17);
                    }
                }
                break;
            case 17:
                if(!follower.isBusy()) {
                    targetVel = 1250;
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.INTAKING);
                    follower.followPath(intake3, 1.0, true);
                    setPathState(19);
                }
                break;
            case 19:
                if(!follower.isBusy()) {
                    targetVel = 1850;
                    follower.followPath(shoot3, 1.0, true);
                    setPathState(20);
                }
                break;
            case 20:
                if(!follower.isBusy()) {
                    // TODO: REMOVE LINE BELOW LATER
//                    intakeSpindexer.ballsLoaded = 3;
                    intakeSpindexer.kickstartShootChain = true;
                    intakeSpindexer.shootDone = false;
                    intakeSpindexer.shootCallTime = opmodeTimer.getElapsedTime();
                    setPathState(21);
                }
                break;
            case 21:
                if(!follower.isBusy()) {
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.SHOOTING);
                    if(pathTimer.getElapsedTime() > 150 + 150 + 400 + 150 + 150 + 400 + 150 + 150) {
                        setPathState(22);
                    }
                }
                break;
            case 22:
                if(!follower.isBusy()) {
                    targetVel = 1250;
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.INTAKING);
                    follower.followPath(intake4, 1.0, true);
                    setPathState(24);
                }
                break;
            case 24:
                if(!follower.isBusy()) {
                    targetVel = 1850;
                    follower.followPath(shoot4, 1.0, true);
                    setPathState(25);
                }
                break;
            case 25:
                if(!follower.isBusy()) {
                    // TODO: REMOVE LINE BELOW LATER
//                    intakeSpindexer.ballsLoaded = 3;
                    intakeSpindexer.kickstartShootChain = true;
                    intakeSpindexer.shootDone = false;
                    intakeSpindexer.shootCallTime = opmodeTimer.getElapsedTime();
                    setPathState(26);
                }
                break;
            case 26:
                if(!follower.isBusy()) {
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.SHOOTING);
                    if(pathTimer.getElapsedTime() > 150 + 150 + 400 + 150 + 150 + 400 + 150 + 150) {
                        setPathState(27);
                    }
                }
                break;
            case 27:
                if(!follower.isBusy()) {
                    targetVel = 1250;
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.INTAKING);
                    follower.followPath(intake5, 1.0, true);
                    setPathState(29);
                }
                break;
            case 29:
                if(!follower.isBusy()) {
                    targetVel = 1950;
                    follower.followPath(shoot5, 1.0, true);
                    setPathState(30);
                }
                break;
            case 30:
                if(!follower.isBusy()) {
                    // TODO: REMOVE LINE BELOW LATER
//                    intakeSpindexer.ballsLoaded = 3;
                    intakeSpindexer.kickstartShootChain = true;
                    intakeSpindexer.shootDone = false;
                    intakeSpindexer.shootCallTime = opmodeTimer.getElapsedTime();
                    setPathState(31);
                }
                break;
            case 31:
                if(!follower.isBusy()) {
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.SHOOTING);
                    if(pathTimer.getElapsedTime() > 150 + 150 + 400 + 150 + 150 + 400 + 150 + 150) {
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
        turretShooter.trackAprilTag(limelightOffset);
        turretShooter.getServo(TurretShooter_shreyas.ServoNames.hood).setPosition(1.0);

        double currentVelocity = turretShooter.getMotor(TurretShooter_shreyas.MotorNames.rightShooter).getVelocity();
        double pid = controller.calculate(currentVelocity, targetVel);
        double velocity = pid + f*targetVel;

        turretShooter.setShooterVelocity(velocity);

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

        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);

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