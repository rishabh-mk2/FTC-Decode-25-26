package org.firstinspires.ftc.teamcode.testOpmodes;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Configurable
@TeleOp(name = "test teleop", group = "TeleOp")
@Disabled
public class testTeleOp extends OpMode {

    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;

    PIDController pidController;

    DcMotorEx intake, leftShooter, rightShooter, turret;
    Servo spin1, spin2, kicker, hood;

    Limelight3A limelight;
    LLResult result;
    double pid;

    RevColorSensorV3 front;

    ElapsedTime runtime = new ElapsedTime();
    double time;

    static final double BALL_DIST_MM = 30;
    static final double SPIN_STEP = 0.169;

    int ballsLoaded = 0;
    boolean ballDetectedLast = false;

    enum IntakeState {
        IDLE,
        INTAKING,
        INDEXING
    }

    IntakeState intakeState = IntakeState.IDLE;

    boolean shootRequested = false;
    boolean AisPressed = false;

    enum ShootState {
        IDLE,
        KICK_UP,
        KICK_DOWN,
        REINDEX
    }

    ShootState shootState = ShootState.IDLE;

    void rotateSpindexer120() {
        spin1.setPosition(spin1.getPosition() + SPIN_STEP);
        spin2.setPosition(spin2.getPosition() + SPIN_STEP);
    }

    @Override
    public void init() {
        automatedDrive = false;
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        spin1 = hardwareMap.get(Servo.class, "spin1");
        spin2 = hardwareMap.get(Servo.class, "spin2");
        kicker = hardwareMap.get(Servo.class, "kicker");
        hood = hardwareMap.get(Servo.class, "hood");

        front = hardwareMap.get(RevColorSensorV3.class, "front");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        kicker.setPosition(0.225);
        spin1.setPosition(0.06);
        spin2.setPosition(0.06);

        pidController = new PIDController(0.02, 0, 0.5);

        runtime.reset();
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {

        // DRIVE
        follower.update();
        if (!automatedDrive) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true
            );
        }

        // LIMELIGHT TURRET
        result = limelight.getLatestResult();
        if (result.isValid()) {
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                pid = pidController.calculate(
                        turret.getCurrentPosition(),
                        turret.getCurrentPosition()
                                - (537.7 * fiducial.getTargetYDegrees()) / 360.0
                );
                turret.setPower(pid);
            }
        } else {
            turret.setPower(0);
        }

        double dist = front.getDistance(DistanceUnit.MM);
        boolean ballDetectedNow = dist < BALL_DIST_MM;

        // INTAKE STATE MACHINE
        switch (intakeState) {

            case IDLE:
                intake.setPower(0);
                if (ballsLoaded < 3) {
                    intakeState = IntakeState.INTAKING;
                }
                break;

            case INTAKING:
                intake.setPower(0.8);

                if (ballDetectedNow && !ballDetectedLast) {
                    ballsLoaded++;
                    rotateSpindexer120();
                    intakeState = IntakeState.INDEXING;
                    time = runtime.milliseconds();
                }
                break;

            case INDEXING:
                intake.setPower(0.5);

                if (runtime.milliseconds() - time > 200) {
                    intakeState = ballsLoaded < 3 ? IntakeState.INTAKING : IntakeState.IDLE;
                }
                break;
        }

        ballDetectedLast = ballDetectedNow;

        // SHOOT BUTTON
        if (gamepad1.a && !AisPressed) AisPressed = true;
        if (!gamepad1.a && AisPressed) {
            AisPressed = false;
            if (ballsLoaded > 0 && shootState == ShootState.IDLE) {
                shootRequested = true;
            }
        }

        // SHOOT STATE MACHINE
        switch (shootState) {

            case IDLE:
                if (shootRequested) {
                    leftShooter.setVelocity(-2250);
                    rightShooter.setVelocity(-2250);
                    kicker.setPosition(0.4);
                    time = runtime.milliseconds();
                    shootState = ShootState.KICK_UP;
                    shootRequested = false;
                }
                break;

            case KICK_UP:
                if (runtime.milliseconds() - time > 120) {
                    kicker.setPosition(0.225);
                    time = runtime.milliseconds();
                    shootState = ShootState.KICK_DOWN;
                }
                break;

            case KICK_DOWN:
                if (runtime.milliseconds() - time > 100) {
                    rotateSpindexer120();
                    ballsLoaded--;
                    shootState = ShootState.IDLE;
                }
                break;
        }
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
