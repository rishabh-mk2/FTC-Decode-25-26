package org.firstinspires.ftc.teamcode.testOpmodes;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "test teleop", group = "TeleOp")

public class testTeleOp extends OpMode {


    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    Telemetry telemetry;

    DcMotorEx intake;
    DcMotorEx leftShooter;
    DcMotorEx rightShooter;
    DcMotorEx turret;

    Servo spindexer1;
    Servo spindexer2;
    Servo kicker;
    Servo hood;

    Limelight3A limelight;
    PIDController pidController;

    RevColorSensorV3 front;
    RevColorSensorV3 back1;
    RevColorSensorV3 back2;

    boolean intaking = false;
    boolean spindexing = false;

    boolean kickerUp1 = false;
    boolean kickerDown1 = false;

    boolean kickerUp2 = false;
    boolean kickerDown2 = false;

    boolean kickerUp3 = false;
    boolean kickerDown3 = false;
    boolean rotateSpin120_1 = false;
    boolean rotateSpin120_2 = false;
    boolean rotateSpin120_3 = false;


    boolean rotateSpin60 = false;

    boolean shoot = false;

    ElapsedTime runtime = new ElapsedTime();
    double time = 0.0;
    double frontDistance = 0.0;

    @Override
    public void init() {
        automatedDrive = false;
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);

        intake = this.hardwareMap.get(DcMotorEx.class, "intake");
        leftShooter = this.hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = this.hardwareMap.get(DcMotorEx.class, "rightShooter");
        turret = this.hardwareMap.get(DcMotorEx.class, "turret");

        spindexer1 = this.hardwareMap.get(Servo.class, "spin1");
        spindexer2 = this.hardwareMap.get(Servo.class, "spin2");
        kicker = this.hardwareMap.get(Servo.class, "kicker");
        hood = this.hardwareMap.get(Servo.class, "hood");

        limelight = this.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        front = this.hardwareMap.get(RevColorSensorV3.class, "front");
        back1 = this.hardwareMap.get(RevColorSensorV3.class, "back1");
        back2 = this.hardwareMap.get(RevColorSensorV3.class, "back2");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        kicker.setPosition(0.225);

        leftShooter.setDirection(DcMotorEx.Direction.FORWARD);
        rightShooter.setDirection(DcMotorEx.Direction.REVERSE);
        leftShooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightShooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.setAutoClear(true);

        limelight.start();

        follower.update();

        runtime.reset();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        follower.update();
        if (!automatedDrive) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true
            );
        }

        if(intaking) {
            intake.setPower(0.8);
            if(Math.abs(front.getDistance(DistanceUnit.MM) - frontDistance) > 50) {
                //TODO: ROTATE SPINDEXER 120 DEG
            }
        } else if (spindexing) {
            intake.setPower(0.5);
        } else {
            intake.setPower(0.0);
        }

        // SHOOT SEQUENCE
        if(shoot) {
            leftShooter.setVelocity(-2250);
            rightShooter.setVelocity(-2250);
            time = runtime.milliseconds();
            kickerUp1 = true;
            shoot = false;
            spindexing = true;
        }
        // FIRST SHOT
        if(kickerUp1 && runtime.milliseconds() - time > 3500) {
            leftShooter.setVelocity(-2250);
            rightShooter.setVelocity(-2250);
            kicker.setPosition(0.4);
            kickerUp1 = false;
            time = runtime.milliseconds();
            kickerDown1 = true;
        }
        if(kickerDown1 && runtime.milliseconds() - time > 1000) {
            leftShooter.setVelocity(-2250);
            rightShooter.setVelocity(-2250);
            kicker.setPosition(0.225);
            kickerDown1 = false;
            time = runtime.milliseconds();
            rotateSpin120_1 = true;
        }
        if(rotateSpin120_1 && runtime.milliseconds() - time > 2000) {
            leftShooter.setVelocity(-2250);
            rightShooter.setVelocity(-2250);
            //TODO: ROTATE SPINDEXER 120 DEG
            rotateSpin120_1 = false;
            time = runtime.milliseconds();
            kickerUp2 = true;
        }

        // SECOND SHOT
        if(kickerUp2 && runtime.milliseconds() - time > 3500) {
            leftShooter.setVelocity(-2250);
            rightShooter.setVelocity(-2250);
            kicker.setPosition(0.4);
            kickerUp2 = false;
            time = runtime.milliseconds();
            kickerDown2 = true;
        }
        if(kickerDown2 && runtime.milliseconds() - time > 1000) {
            leftShooter.setVelocity(-2250);
            rightShooter.setVelocity(-2250);
            kicker.setPosition(0.225);
            kickerDown2 = false;
            time = runtime.milliseconds();
            rotateSpin120_2 = true;
        }
        if(rotateSpin120_2 && runtime.milliseconds() - time > 2000) {
            leftShooter.setVelocity(-2250);
            rightShooter.setVelocity(-2250);
            //TODO: ROTATE SPINDEXER 120 DEG
            rotateSpin120_2 = false;
            time = runtime.milliseconds();
            kickerUp3 = true;
        }

        // THIRD SHOT
        if(kickerUp3 && runtime.milliseconds() - time > 3500) {
            leftShooter.setVelocity(-2250);
            rightShooter.setVelocity(-2250);
            kicker.setPosition(0.4);
            kickerUp3 = false;
            time = runtime.milliseconds();
            kickerDown3 = true;
        }
        if(kickerDown3 && runtime.milliseconds() - time > 1000) {
            leftShooter.setVelocity(0);
            rightShooter.setVelocity(0);
            kicker.setPosition(0.225);
            kickerDown3 = false;
        }


        frontDistance = front.getDistance(DistanceUnit.MM);
        telemetry.update();
    }
}