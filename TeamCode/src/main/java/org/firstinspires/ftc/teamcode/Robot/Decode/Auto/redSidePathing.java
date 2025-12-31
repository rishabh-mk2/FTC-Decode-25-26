package org.firstinspires.ftc.teamcode.Robot.Decode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Side Pathing", group = "Autonomous")
public class redSidePathing extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private double speed = 0.5;

    /* ---------- Poses ---------- */
    private final Pose startPose   = new Pose(122, 122, Math.toRadians(270));
    private final Pose scorePose1  = new Pose(98, 98, Math.toRadians(270));
    private final Pose scorePose2  = new Pose(94, 79, Math.toRadians(0));
    private final Pose scorePose3  = new Pose(89, 74, Math.toRadians(270));
    private final Pose scorePose4  = new Pose(85, 73, Math.toRadians(270));
    private final Pose last = new Pose(88, 66, Math.toRadians(270));



    private final Pose pickup1Pose = new Pose(126, 77, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(122, 52.5, Math.toRadians(270));
    private final Pose pickup3Pose = new Pose(121, 35, Math.toRadians(307));
    private final Pose releasePose = new Pose(132, 70, Math.toRadians(0));
    private final Pose pushReleasePose = new Pose(138.5, 69, Math.toRadians(0));


    /* ---------- Paths ---------- */
    private PathChain scorePreload;
    private PathChain pickup1, releasePreload, pushReleaseHook, scorePickup1;
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

        releasePreload = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, releasePose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        pushReleaseHook = follower.pathBuilder()
                .addPath(new BezierLine(releasePose, pushReleasePose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(releasePose, scorePose2))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(330))
                .build();

        pickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, pickup2Pose))
                .setLinearHeadingInterpolation(Math.toRadians(330), Math.toRadians(330))
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose3))
                .setLinearHeadingInterpolation(Math.toRadians(330), Math.toRadians(308))
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
                follower.followPath(scorePreload, speed, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(pickup1, speed, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(releasePreload, speed, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(pushReleaseHook, 0.35, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1, speed, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(pickup2, speed, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2, speed, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(pickup3, speed, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3, speed, true);
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

    /* ---------- Helpers ---------- */
    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    /* ---------- OpMode ---------- */
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
