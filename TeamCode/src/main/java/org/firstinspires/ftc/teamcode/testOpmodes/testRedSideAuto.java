package org.firstinspires.ftc.teamcode.testOpmodes; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Test Red Side Auto", group = "Auto")
@Disabled
public class testRedSideAuto extends OpMode {

    private Follower follower;
    private double speed = 0.9;
    private Timer pathTimer, actionTimer, opmodeTimer;
    ElapsedTime waitTimer = new ElapsedTime();

    private int pathState;
    private final Pose startPose = new Pose(122, 122, Math.toRadians(225));
    private final Pose scorePose = new Pose(104, 104, Math.toRadians(225)); // Scoring Pose (Facing away from goal)
    private final Pose pickup1Pose = new Pose(120, 84, Math.toRadians(0)); // Closest to the goal
    private final Pose pickup2Pose = new Pose(120, 60, Math.toRadians(0)); // Middle
    private final Pose pickup3Pose = new Pose(120, 36, Math.toRadians(0)); // Furthest from goal
    private final Pose releasePose = new Pose(126, 72, Math.toRadians(180));
    private double waitTest;

    private PathChain scorePreload, pickup1, releasePreload, scorePickup1, pickup2, scorePickup2, pickup3, scorePickup3;

    public void buildPaths() {

        // Preload stuff
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        // Using a curve to pickup the first set of balls
        pickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(83.767, 86.759), pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        waitTest = 3000;

        // Simple line to release the balls from the thing
        // TODO: may need to add another position to move the robot back into the lever
        releasePreload = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, releasePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), releasePose.getHeading())
                .build();

        // Going to scoring position with a simple line
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(releasePose, scorePose))
                .setLinearHeadingInterpolation(releasePose.getHeading(), scorePose.getHeading())
                .build();

        // Curve to pickup second pair of balls
        pickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(72.797, 66.615), pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        // Going to scoring position with a simple line
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        // Curve to pickup third pair of balls
        pickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(77.584, 37.496), pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        // Going to scoring position with a simple line
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, speed, true);
                        setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                        follower.followPath(pickup1, speed,true);
                        setPathState(8);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                        follower.followPath(releasePreload, speed,true);
                        setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                        follower.followPath(scorePickup1, speed, true);
                        setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                        follower.followPath(pickup2, speed,true);
                        setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                        follower.followPath(scorePickup2, speed,true);
                        setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                        follower.followPath(pickup3, speed,true);
                        setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                        follower.followPath(scorePickup3, speed,true);
                        setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }
    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {}
}