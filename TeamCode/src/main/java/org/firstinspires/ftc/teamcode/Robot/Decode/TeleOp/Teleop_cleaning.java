package org.firstinspires.ftc.teamcode.Robot.Decode.TeleOp;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.IntakeSpindexer_shreyas;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.TurretShooter_shreyas;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@TeleOp(name = "Teleop cleaned", group = "TeleOp")
public class Teleop_cleaning extends OpMode {
    Follower follower;
    TurretShooter_shreyas turretShooter;
    IntakeSpindexer_shreyas intakeSpindexer;

    private boolean XisPressed = false;
    private boolean YisPressed = false;
    private boolean BisPressed = false;
    private boolean AisPressed = false;
    private boolean AisPressed2 = false;
    private  boolean BisPressed2 = false;

    public int velocity = 1400;
    public double hoodPose = 0.5;
    public int firstWait = 800;
    public int secondThirdWait = 300;
    Servo hood;
    DcMotorEx turret, intake, leftShooter, rightShooter;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        intakeSpindexer = new IntakeSpindexer_shreyas(this, true);
        turretShooter = new TurretShooter_shreyas(this, Alliance.RED, intakeSpindexer);
        telemetry.addData("Status", "Initialized");

        hood = turretShooter.getServo(TurretShooter_shreyas.ServoNames.hood);
        turret = turretShooter.getMotor(TurretShooter_shreyas.MotorNames.turret);
        leftShooter = turretShooter.getMotor(TurretShooter_shreyas.MotorNames.leftShooter);
        rightShooter = turretShooter.getMotor(TurretShooter_shreyas.MotorNames.rightShooter);
        intake = intakeSpindexer.getMotor(IntakeSpindexer_shreyas.MotorNames.intake);
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        // --- DRIVE ---
        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x * 0.4,
                true
        );

        // --- SUBSYSTEM UPDATES ---
        turretShooter.trackAprilTag();
        intakeSpindexer.update();

        // --- INTAKE TOGGLE (Button X) ---
        if (gamepad1.x && !XisPressed) {
            XisPressed = true;
            if (intakeSpindexer.state == IntakeSpindexer_shreyas.IntakeState.INTAKING) {
                // if its running and clicked again, it stops
                intake.setPower(1);
                intakeSpindexer.rotateSpindexer60();
                new Thread(() -> {
                    try {
                        Thread.sleep(200);
                    } catch (Exception e) {
                        throw new RuntimeException(e);
                    } finally {
                        intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.IDLE);
                    }
                }).start();
            } else {
                if (intakeSpindexer.getBallCount() < 3) { // keeps running if more balls can be loaded / till 3 balls are loaded
                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.INTAKING);
                } else if (intakeSpindexer.getBallCount() == 3) {
                    intake.setPower(1);
                    intakeSpindexer.rotateSpindexer60();
                    new Thread(() -> {
                        try {
                            Thread.sleep(200);
                            intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.IDLE);
                        } catch (Exception e) {
                            throw new RuntimeException(e);
                        }
                    }).start();
                }
            }
        } else if (!gamepad1.x) {
            XisPressed = false;
        }

        //this is for driver 2 to control the turret and hood positions
        if(gamepad2.left_bumper) {
            turret.setPower(0.2);
        }
        else if (gamepad2.right_bumper){
            turret.setPower(-0.2);
        }
        else {
            turret.setPower(0);
        }

        if(gamepad2.a && !AisPressed2) {
            AisPressed2 = true;
        }
        if (!gamepad2.a && AisPressed2) {
            hood.setPosition(hood.getPosition() - 0.05);
            AisPressed2 = false;
        }
        if(gamepad2.b && !BisPressed2){
            BisPressed2 = true;
        }
        if (!gamepad2.b && BisPressed2) {
            hood.setPosition(hood.getPosition() + 0.05);
            BisPressed2 = false;
        }

        // --- INDEXED SHOOT BUTTON (Y) ---
        // Trigger the PPG / PGP / GPP sequence (endgame Teleop)
        if (gamepad1.yWasReleased()) {
            YisPressed = true;
            leftShooter.setVelocity(300);
            rightShooter.setVelocity(300);
            new Thread(() -> {
                try {
                    turretShooter.shootIndexed(TurretShooter_shreyas.ShootCase.PPG, 1000, 0.5);
                } catch (Exception e) {
                    throw new RuntimeException(e);
                }
            }).start();
            /* TODO: when we find the order (PPG/PGP/GPP) in auto, get the value into teleup and change
             *   the parameter above based on the parameter from AUTO*/
        } else if (!gamepad1.y) {
            YisPressed = false;
        }

        // --- MANUAL SHOOT BUTTON (B) ---
        // Regular shoot for teleop
        if (gamepad1.b && !BisPressed) {
            BisPressed = true;
            turretShooter.shoot(2500, 0.95);
            /*TODO: add the kicker/spindexer stuff for 3 ball shoot (not indexed);
               after the 3 ball shoot routine, stop shooter --> reset the array list*/
            //intakeSpindexer.ballColors.clear();
        } else if (!gamepad1.b) {
            BisPressed = false;
        }

        if (gamepad1.a && !AisPressed && intakeSpindexer.ballsLoaded > 0) {
            AisPressed = true;
            turretShooter.simpleShootSequence(velocity, hoodPose, firstWait, secondThirdWait,
                    intakeSpindexer);
        } else if (!gamepad1.a) {
            AisPressed = false;
        }

        if (intakeSpindexer.ballsLoaded == 0) {
            leftShooter.setVelocity(0);
            rightShooter.setVelocity(0);
        }

        if (gamepad1.dpadUpWasReleased()) {
            velocity = velocity + 100;
        }
        if (gamepad1.dpadDownWasReleased()) {
            velocity = velocity - 100;
        }
        if (gamepad2.dpadUpWasReleased()) {
            velocity = velocity + 50;
        }
        if (gamepad2.dpadDownWasReleased()) {
            velocity = velocity - 50;
        }
        if (gamepad1.dpadRightWasReleased()) {
            hoodPose = hoodPose + 0.05;
        }
        if (gamepad1.dpadLeftWasReleased()) {
            hoodPose = hoodPose - 0.05;
        }
        if (gamepad1.rightBumperWasReleased()){
            firstWait = firstWait + 100;
        }
        if (gamepad1.leftBumperWasReleased()){
            firstWait = firstWait - 100;
        }
        if (gamepad2.rightBumperWasReleased()){
            secondThirdWait = secondThirdWait + 100;
        }
        if (gamepad2.leftBumperWasReleased()){
            secondThirdWait = secondThirdWait - 100;
        }

        //TODO: fix the intake motor not spinning (possible fix below)
        if (intakeSpindexer.ballsLoaded > 0 &&
                intakeSpindexer.state == IntakeSpindexer_shreyas.IntakeState.IDLE) {
            intake.setPower(0.6);
        } // hopefully a fix for the intake not spinning

        // --- TELEMETRY ---
        telemetry.addData("Balls Loaded", intakeSpindexer.getBallCount());
        telemetry.addData("Intake State", intakeSpindexer.hasBalls());
        telemetry.addData("Velocity", velocity);
        telemetry.addData("Hood Pose", hoodPose);
        telemetry.addData("First Wait", firstWait);
        telemetry.addData("2nd / 3rd", secondThirdWait);
        telemetry.addData("Ball Colors", intakeSpindexer.ballColors.toString());
        telemetry.addData("Balls Loaded", intakeSpindexer.ballsLoaded);
        telemetry.addData("Array Size", intakeSpindexer.ballColors.size());
        for (int i=0; i < intakeSpindexer.ballColors.size(); i++) {
            telemetry.addData("Ball" + (i + 1), intakeSpindexer.ballColors.get(i).name());
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        turretShooter.stop();
    }
}