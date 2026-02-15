package org.firstinspires.ftc.teamcode.Robot.Decode.Auto.Rishabh; // make sure this aligns with class location

import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
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

@Autonomous(name = "Blue Front 15 Back Spike", group = "Autonomous")
public class front15BackSpike_BLUE extends OpMode {

    private Follower follower;
    IntakeSpindexer intakeSpindexer;
    TurretShooter turretShooter;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState = 0;

    // START POSE
    private final Pose startPose = new Pose(127, 112.5, Math.toRadians(0)).mirror();
// PRELOAD - 3 Balls shot
    private final Pose shoot1End = new Pose(91, 88.5, Math.toRadians(0)).mirror();
// FIRST INTAKING -> SHOOTING; 6 Balls shot
    private final Pose intake1_1End = new Pose(100, 60, Math.toRadians(0)).mirror();
    private final Pose intake1_2End = new Pose(121, 60, Math.toRadians(0)).mirror();
    private final Pose shoot2End = new Pose(92, 76, Math.toRadians(0)).mirror();

    // SECOND INTAKING -> SHOOTING; 9 Balls shot
    private final Pose intake2_1End = new Pose(124, 59, Math.toRadians(27.75)).mirror();
    private final Pose intake2_2End = new Pose(124, 55, Math.toRadians(40)).mirror();
    private final Pose intake2_3End = new Pose(124, 59, Math.toRadians(27.75)).mirror();
    private final Pose shoot3End = new Pose(92, 76, Math.toRadians(0)).mirror();

    // THIRD INTAKING -> SHOOTING; 12 Balls shot
    private final Pose intake3_1End = new Pose(95, 36, Math.toRadians(0)).mirror();
    private final Pose intake3_2End = new Pose(121, 36, Math.toRadians(0)).mirror();
    private final Pose shoot4End = new Pose(92, 76, Math.toRadians(0)).mirror();

    // FOURTH INTAKING -> SHOOTING; 15 Balls shot
    private final Pose intake4End = new Pose(124, 90, Math.toRadians(0)).mirror();
    private final Pose shoot5End = new Pose(110, 100, Math.toRadians(0)).mirror();



    double targetVel = 0.0;
    double limelightOffset = 0.0;
    private PIDController controller;
    final double p = 0.75, i = 0, d = 0.05, f = 1.0;

    private PathChain shoot1, intake1_1, intake1_2, shoot2, intake2_1, intake2_2, intake2_3, shoot3, intake3_1, intake3_2, shoot4, intake4, shoot5;

    public void buildPaths() {
        shoot1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPose,
                                shoot1End
                        )
                ).setConstantHeadingInterpolation(shoot1End.getHeading())
                .build();

        intake1_1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                shoot1End,
                                intake1_1End
                        )
                ).setConstantHeadingInterpolation(intake1_1End.getHeading())
                .build();

        intake1_2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                intake1_1End,
                                intake1_2End
                        )
                ).setConstantHeadingInterpolation(intake1_2End.getHeading())
                .build();
        shoot2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                intake1_2End,
                                shoot2End
                        )
                ).setConstantHeadingInterpolation(shoot2End.getHeading())
                .build();

        intake2_1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                shoot2End,
                                intake2_1End
                        )
                ).setLinearHeadingInterpolation(shoot2End.getHeading(), intake2_1End.getHeading())
                .build();

        intake2_2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                intake2_1End,
                                intake2_2End
                        )
                ).setConstantHeadingInterpolation(intake2_2End.getHeading())
                .build();

        intake2_3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                intake2_2End,
                                intake2_3End
                        )
                ).setConstantHeadingInterpolation(intake2_3End.getHeading())
                .build();

        shoot3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                intake2_3End,
                                shoot3End
                        )
                ).setConstantHeadingInterpolation(shoot3End.getHeading())
                .build();

        intake3_1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                shoot3End,
                                intake3_1End
                        )
                ).setConstantHeadingInterpolation(intake3_1End.getHeading())
                .build();

        intake3_2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                intake3_1End,
                                intake3_2End
                        )
                ).setConstantHeadingInterpolation(intake3_2End.getHeading())
                .build();

        shoot4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                intake3_2End,
                                shoot4End
                        )
                ).setConstantHeadingInterpolation(shoot4End.getHeading())
                .build();

        intake4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                shoot4End,
                                intake4End
                        )
                ).setConstantHeadingInterpolation(intake4End.getHeading())
                .build();

        shoot5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                intake4End,
                                shoot5End
                        )
                ).setConstantHeadingInterpolation(shoot5End.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                if(!follower.isBusy()) {
                    targetVel = 1500;
                    intakeSpindexer.ballsLoaded = 3;
                    turretShooter.getServo(TurretShooter.ServoNames.hood).setPosition(0.6);
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
                    intakeSpindexer.setIntakeState(IntakeSpindexer.IntakeState.SHOOTING);
                    if(pathTimer.getElapsedTime() > 150 + 150 + 400 + 150 + 150 + 400 + 150 + 150) {
                        setPathState(3);
                    }
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    targetVel = 1300;
                    intakeSpindexer.setIntakeState(IntakeSpindexer.IntakeState.INTAKING);
                    follower.followPath(intake1_1, 1.0, true);
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
                    targetVel = 1500;
//                    intakeSpindexer.ballsLoaded = 3;
                    turretShooter.getServo(TurretShooter.ServoNames.hood).setPosition(0.7);
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
                    intakeSpindexer.setIntakeState(IntakeSpindexer.IntakeState.SHOOTING);
                    if(pathTimer.getElapsedTime() > 150 + 150 + 400 + 150 + 150 + 400 + 150 + 150) {
                        setPathState(8);
                    }
                }
                break;
            case 8:
                if(!follower.isBusy() || intakeSpindexer.ballsLoaded == 3) {
                    targetVel = 1300;
                    intakeSpindexer.setIntakeState(IntakeSpindexer.IntakeState.INTAKING);
                    follower.followPath(intake2_1, 0.9, true);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy() && pathTimer.getElapsedTime() > 100) {
                    setPathState(10);
                }
                break;

            case 10:
                if(!follower.isBusy() || intakeSpindexer.ballsLoaded == 3) {
                    follower.followPath(intake2_2, 0.35, true);
                    setPathState(11);
                }
                break;

            case 11:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTime() > 1750 || intakeSpindexer.ballsLoaded == 3) {
                        setPathState(12);
                    }
                }
                break;

            case 12:
                if(!follower.isBusy() || intakeSpindexer.ballsLoaded == 3) {
                    follower.followPath(intake2_3, 0.35, true);
                    setPathState(13);
                }
                break;

            case 13:
                if(!follower.isBusy() || intakeSpindexer.ballsLoaded == 3) {
                    follower.followPath(intake2_2, 0.35, true);
                    setPathState(14);
                }
                break;

            case 14:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > 500 || intakeSpindexer.ballsLoaded == 3) {
                    setPathState(15);
                }
                break;
            case 15:
                if(!follower.isBusy()) {
                    targetVel = 1500;
//                    intakeSpindexer.ballsLoaded = 3;
                    follower.followPath(shoot3, 1.0, true);
                    setPathState(16);
                }
                break;

            case 16:
                if(!follower.isBusy()) {
                    intakeSpindexer.kickstartShootChain = true;
                    intakeSpindexer.shootDone = false;
                    intakeSpindexer.shootCallTime = opmodeTimer.getElapsedTime();
                    setPathState(17);
                }
                break;
            case 17:
                if(!follower.isBusy()) {
                    intakeSpindexer.setIntakeState(IntakeSpindexer.IntakeState.SHOOTING);
                    if(pathTimer.getElapsedTime() > 150 + 150 + 400 + 150 + 150 + 400 + 150 + 150) {
                        setPathState(18);
                    }
                }
                break;
//
            case 18:
                if(!follower.isBusy()) {
                    targetVel = 1300;
                    intakeSpindexer.setIntakeState(IntakeSpindexer.IntakeState.INTAKING);
                    follower.followPath(intake3_1, 1.0, true);
                    setPathState(19);
                }
                break;

            case 19:
                if(!follower.isBusy()) {
                    follower.followPath(intake3_2, 0.28, true);
                    setPathState(20);
                }
                break;

            case 20:
                if(!follower.isBusy()) {
                    targetVel = 1500;
                    turretShooter.getServo(TurretShooter.ServoNames.hood).setPosition(0.7);
                    follower.followPath(shoot4, 1.0, true);
                    setPathState(21);
                }
                break;

            case 21:
                if(!follower.isBusy()) {
                    intakeSpindexer.kickstartShootChain = true;
                    intakeSpindexer.shootDone = false;
                    intakeSpindexer.shootCallTime = opmodeTimer.getElapsedTime();
                    setPathState(22);
                }
                break;
            case 22:
                if(!follower.isBusy()) {
                    intakeSpindexer.setIntakeState(IntakeSpindexer.IntakeState.SHOOTING);
                    if(pathTimer.getElapsedTime() > 150 + 150 + 400 + 150 + 150 + 400 + 150 + 150) {
                        setPathState(23);
                    }
                }
                break;

            case 23:
                if(!follower.isBusy()) {
                    targetVel = 1500;
                    intakeSpindexer.setIntakeState(IntakeSpindexer.IntakeState.INTAKING);
                    follower.followPath(intake4, 1.0, true);
                    setPathState(24);
                }
                break;

            case 24:
                if(!follower.isBusy() && pathTimer.getElapsedTime() > 250) {
                    setPathState(25);
                }
                break;

            case 25:
                if(!follower.isBusy()) {
                    intakeSpindexer.getMotor(IntakeSpindexer.MotorNames.intake).setPower(0.7);
                    intakeSpindexer.getServo(IntakeSpindexer.ServoNames.spin1).setPosition(0.0);
                    intakeSpindexer.getServo(IntakeSpindexer.ServoNames.spin2).setPosition(0.0);
                }
                break;
//            case 26:
//                if(!follower.isBusy()) {
//                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.SHOOTING);
//                    if(pathTimer.getElapsedTime() > 150 + 150 + 400 + 150 + 150 + 400 + 150 + 150) {
//                        setPathState(-1);
//                    }
//                }
//                break;
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
        double distance = turretShooter.getAprilTagDistance();
        intakeSpindexer.update(opmodeTimer.getElapsedTime());
        turretShooter.trackAprilTag(10);

        double currentVelocity = turretShooter.getMotor(TurretShooter.MotorNames.rightShooter).getVelocity();
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

        intakeSpindexer = new IntakeSpindexer(this, true);
        turretShooter = new TurretShooter(this, Alliance.BLUE);

        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        intakeSpindexer.ballsLoaded = 3;
        intakeSpindexer.getServo(IntakeSpindexer.ServoNames.spin1).setPosition(0.097);
        intakeSpindexer.getServo(IntakeSpindexer.ServoNames.spin2).setPosition(0.097);
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