package org.firstinspires.ftc.teamcode.Robot.Decode.TeleOp;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.IntakeSpindexer_shreyas;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.TurretShooter_shreyas;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@TeleOp(name = "TeleOp_shreyas", group = "TeleOp")
public class Teleop_shreyas extends OpMode {
    Follower follower;
    TurretShooter_shreyas turretShooter;
    IntakeSpindexer_shreyas intakeSpindexer;

    private boolean XisPressed = false;
    private boolean YisPressed = false;
    private boolean BisPressed = false;
    private boolean AisPressed = false;
    private boolean isShooterSpinning = false;
    public int velocity = 2800;
    public double hoodPose = 1;
    public int firstWait = 800;
    public int secondThirdWait = 300;


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        // Important: Initialize Intake FIRST, then pass it to Shooter
        intakeSpindexer = new IntakeSpindexer_shreyas(this);
        turretShooter = new TurretShooter_shreyas(this, Alliance.RED, intakeSpindexer);
        telemetry.addData("Status", "Initialized");
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
        turretShooter.updateTurretTracking();
        intakeSpindexer.update();

        // --- INTAKE TOGGLE (Button X) ---
        if (gamepad1.x && !XisPressed) {
            XisPressed = true;
            if (intakeSpindexer.state == IntakeSpindexer_shreyas.IntakeState.INTAKING) {
                // if its running and clicked again, it stops
                intakeSpindexer.getMotor(IntakeSpindexer_shreyas.MotorNames.intake).setPower(1);
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
                    intakeSpindexer.getMotor(IntakeSpindexer_shreyas.MotorNames.intake).setPower(1);
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

        // --- INDEXED SHOOT BUTTON (Y) ---
        // Trigger the PPG / PGP / GPP sequence (endgame Teleop)
        if (gamepad1.y && !YisPressed) {
            YisPressed = true;
            turretShooter.shootIndexed(TurretShooter_shreyas.ShootCase.PPG);
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
            isShooterSpinning = true;
            turretShooter.getMotor(TurretShooter_shreyas.MotorNames.leftShooter).setVelocity(velocity);
            turretShooter.getMotor(TurretShooter_shreyas.MotorNames.rightShooter).setVelocity(velocity);
            turretShooter.getServo(TurretShooter_shreyas.ServoNames.hood).setPosition(hoodPose);

            new Thread(() -> {
                try {
                    Thread.sleep(firstWait);
                    turretShooter.shootCCW();
                    intakeSpindexer.ballsLoaded--;
                    Thread.sleep(secondThirdWait);
                    turretShooter.shootCCW();
                    intakeSpindexer.ballsLoaded--;
                    Thread.sleep(secondThirdWait);
                    turretShooter.shootCCW();
                    intakeSpindexer.ballsLoaded--;
                    intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin1).setPosition(0);
                    intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin2).setPosition(0);
                } catch (Exception e) {
                    throw new RuntimeException(e);
                } finally {
                    isShooterSpinning = false;
                }
            }).start();
        } else if (!gamepad1.a) {
            AisPressed = false;
        }

        if ((!isShooterSpinning && !gamepad1.a) || intakeSpindexer.ballsLoaded == 0) {
            turretShooter.getMotor(TurretShooter_shreyas.MotorNames.leftShooter).setVelocity(0);
            turretShooter.getMotor(TurretShooter_shreyas.MotorNames.rightShooter).setVelocity(0);
        }

        if (gamepad1.dpadUpWasReleased()) {
            velocity = velocity + 100;
        }
        if (gamepad1.dpadDownWasReleased()) {
            velocity = velocity - 100;
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
            intakeSpindexer.getMotor(IntakeSpindexer_shreyas.MotorNames.intake).setPower(0.6);
        } // hopefully a fix for the intake not spinning


        // --- TELEMETRY ---
        telemetry.addData("Balls Loaded", intakeSpindexer.getBallCount());
        telemetry.addData("Intake State", intakeSpindexer.hasBalls());
        telemetry.addData("Velocity", velocity);
        telemetry.addData("Hood Pose", hoodPose);
        telemetry.addData("First Wait", firstWait);
        telemetry.addData("2nd / 3rd", secondThirdWait);
        /*for (IntakeSpindexer_shreyas.BallColor color : intakeSpindexer.ballColors) {
            telemetry.addData("color:", color.ordinal());
        }*/
        telemetry.update();
    }

    @Override
    public void stop() {
        turretShooter.stop();
    }
}