package org.firstinspires.ftc.teamcode.Robot.Decode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Config
@TeleOp(name = "TeleOp", group = "TeleOp")
public class Teleop extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    Follower follower;
    PIDController spindexerPID;

    DcMotorEx spindexer;
    Servo turret1, turret2, turret3;
    DcMotorEx leftShooter, rightShooter, intake;

    RevColorSensorV3 frontSensor1, frontSensor2;
    RevColorSensorV3 backRightSensor1, backRightSensor2;
    RevColorSensorV3 backLeftSensor1, backLeftSensor2;

    Pose startPose = new Pose(0, 0, Math.toRadians(0));

    public static double shooterPower = 0.0;
    public static double turretPosition = 0.475;

    public static double p = 0.01;
    public static double i = 0.0;
    public static double d = 0.0005;

    public static double homeTolerance = 15;

    int loopCounter = 0;

    double f1Dist, f2Dist, br1Dist, br2Dist, bl1Dist, bl2Dist;

    boolean sort = false;

    boolean ballFront = false;
    boolean ballBackLeft = false;
    boolean ballBackRight = false;
    boolean spindexerHomed = false;

    double spindexerTargetPosition = 0.0;

    // --- INTAKE CONTROL ---
    double intakePower = 1.0;
    boolean intakeReversing = false;
    ElapsedTime intakeReverseTimer = new ElapsedTime();
    boolean rotatedForThree = false;

    enum SpindexerState {
        INTAKING,
        READY_TO_SHOOT,
        SHOOTING
    }

    double spindexerHomePosition;
    SpindexerState spindexerState = SpindexerState.INTAKING;

    // READY_TO_SHOOT back-step
    boolean readyBackStepDone = false;

    double originalPosition;
    ElapsedTime spinTimer = new ElapsedTime();

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        turret3 = hardwareMap.get(Servo.class, "turret3");

        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontSensor1 = hardwareMap.get(RevColorSensorV3.class, "f1");
        frontSensor2 = hardwareMap.get(RevColorSensorV3.class, "f2");
        backRightSensor1 = hardwareMap.get(RevColorSensorV3.class, "br1");
        backRightSensor2 = hardwareMap.get(RevColorSensorV3.class, "br2");
        backLeftSensor1 = hardwareMap.get(RevColorSensorV3.class, "bl1");
        backLeftSensor2 = hardwareMap.get(RevColorSensorV3.class, "bl2");

        spindexerHomePosition = spindexer.getCurrentPosition();
        spindexerTargetPosition = spindexerHomePosition;
        spindexerState = SpindexerState.INTAKING;

        spindexerPID = new PIDController(p, i, d);

        follower.setStartingPose(startPose);

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry.setAutoClear(true);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
        runtime.reset();
    }

    @Override
    public void loop() {

        loopCounter++;

        // --- DRIVE ---
        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x * 0.58,
                true
        );

        leftShooter.setPower(shooterPower);
        rightShooter.setPower(-shooterPower);

        turret1.setPosition(turretPosition);
        turret2.setPosition(turretPosition);
        turret3.setPosition(turretPosition);

        spindexerPID.setPID(p, i, d);

        // --- BUTTON LOGIC ---
        if (gamepad1.xWasReleased()) {
            spinTimer.reset();
            spindexerState = SpindexerState.SHOOTING;
        }

        if (gamepad1.yWasReleased()) {
            spindexerState = SpindexerState.READY_TO_SHOOT;
            readyBackStepDone = false; // reset back-step
        }

        // --- STATE MACHINE ---
        switch (spindexerState) {

            case INTAKING:
                spindexerTargetPosition = spindexerHomePosition;
                break;

            case READY_TO_SHOOT:
                if (!readyBackStepDone) {
                    // Move to +175
                    spindexerTargetPosition = spindexerHomePosition + 175;

                    // When it reaches target, do small back-step
                    if (Math.abs(spindexer.getCurrentPosition() - spindexerTargetPosition) < 2) {
                        spindexerTargetPosition -= 35; // small back-step
                        readyBackStepDone = true;
                    }
                }
                break;

            case SHOOTING:
                spindexer.setPower(-0.4);
                if (spinTimer.seconds() >= 2.0) {
                    spindexerState = SpindexerState.INTAKING;
                }
                return; // skip PID
        }

        // --- PID WITH CAP ---
        double pid = spindexerPID.calculate(spindexer.getCurrentPosition(), spindexerTargetPosition);

        // Cap power at 0.1 for small back-step
        if (spindexerState == SpindexerState.READY_TO_SHOOT && readyBackStepDone) {
            pid = Math.max(-0.1, Math.min(0.1, pid));
        } else {
            pid = Math.max(-0.45, Math.min(0.45, pid));
        }

        spindexer.setPower(pid);

        // --- SENSOR POLLING ---
        if (loopCounter % 10 == 0) {
            f1Dist = frontSensor1.getDistance(DistanceUnit.MM);
            f2Dist = frontSensor2.getDistance(DistanceUnit.MM);
            br1Dist = backRightSensor1.getDistance(DistanceUnit.MM);
            br2Dist = backRightSensor2.getDistance(DistanceUnit.MM);
            bl1Dist = backLeftSensor1.getDistance(DistanceUnit.MM);
            bl2Dist = backLeftSensor2.getDistance(DistanceUnit.MM);
        }

        // --- BALL DETECTION ---
        spindexerHomed = Math.abs(spindexer.getCurrentPosition()) < homeTolerance;

        if (spindexerHomed) {
            ballFront = (f1Dist < 45) || (f2Dist < 45);
            ballBackLeft = (bl1Dist < 15) || (bl2Dist < 15);
            ballBackRight = (br2Dist < 15);
        } else {
            ballFront = false;
            ballBackLeft = false;
            ballBackRight = false;
        }

        int ballCount = 0;
        if (ballFront) ballCount++;
        if (ballBackLeft) ballCount++;
        if (ballBackRight) ballCount++;

        // --- INTAKE LOGIC ---
        double intakeCurrent = intake.getCurrent(CurrentUnit.AMPS);

        if (intakeCurrent > 9 && !intakeReversing) {
            intakeReversing = true;
            intakeReverseTimer.reset();
        }

        if (intakeReversing) {
            intakePower = -0.5;
            if (intakeReverseTimer.seconds() >= 0.2) {
                intakeReversing = false;
            }
        } else {
            if (ballCount == 3) {
                if (!rotatedForThree) {
                    spindexerTargetPosition = spindexer.getCurrentPosition() + 175;
                    rotatedForThree = true;
                }
                intakePower = 0;
            } else if (spindexerState == SpindexerState.INTAKING){
                intakePower = 1;
                rotatedForThree = false;
            }
        }

        intake.setPower(intakePower);

        // --- TELEMETRY ---
        telemetry.addData("Spindexer Pos", spindexer.getCurrentPosition());
        telemetry.addData("Spindexer Target", spindexerTargetPosition);
        telemetry.addData("Spindexer Homed", spindexerHomed);

        telemetry.addData("Intake Current (A)", "%.2f", intakeCurrent);
        telemetry.addData("Intake Power", intakePower);

        telemetry.addLine("\n=== BALL DETECTION ===");
        telemetry.addData("Ball Front", ballFront);
        telemetry.addData("Ball Back Left", ballBackLeft);
        telemetry.addData("Ball Back Right", ballBackRight);
        telemetry.addData("Ball Count", ballCount);

        telemetry.addLine("\n=== DISTANCES ===");
        telemetry.addData("F1", "%.1f", f1Dist);
        telemetry.addData("F2", "%.1f", f2Dist);
        telemetry.addData("BR1", "%.1f", br1Dist);
        telemetry.addData("BR2", "%.1f", br2Dist);
        telemetry.addData("BL1", "%.1f", bl1Dist);
        telemetry.addData("BL2", "%.1f", bl2Dist);

        telemetry.update();
    }

    @Override
    public void stop() {}
}
