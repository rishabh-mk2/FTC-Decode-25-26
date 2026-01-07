package org.firstinspires.ftc.teamcode.Robot.Decode.TeleOp;

import android.graphics.Color;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

    public int velocity = 500;
    public double hoodPose = 0.5;
    public int firstWait = 800;
    public int secondThirdWait = 300;

    // --- COLOR SENSOR LATCH ---
    private boolean colorPrinted = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        intakeSpindexer = new IntakeSpindexer_shreyas(this, true);
        turretShooter = new TurretShooter_shreyas(this, Alliance.RED, intakeSpindexer);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {

//        // --- DRIVE ---
//        follower.update();
//        follower.setTeleOpDrive(
//                -gamepad1.left_stick_y,
//                -gamepad1.left_stick_x,
//                -gamepad1.right_stick_x * 0.4,
//                true
//        );
//
//        // --- SUBSYSTEM UPDATES ---
//        turretShooter.trackAprilTag();
//        intakeSpindexer.update();
//
//        // --- INTAKE TOGGLE (X) ---
//        if (gamepad1.x && !XisPressed) {
//            XisPressed = true;
//
//            if (intakeSpindexer.state == IntakeSpindexer_shreyas.IntakeState.INTAKING) {
//                intakeSpindexer.getMotor(IntakeSpindexer_shreyas.MotorNames.intake).setPower(1);
//                intakeSpindexer.rotateSpindexer60(1);
//
//                new Thread(() -> {
//                    try {
//                        Thread.sleep(200);
//                    } catch (Exception e) {
//                        throw new RuntimeException(e);
//                    }
//                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.IDLE);
//                }).start();
//
//            } else {
//                if (intakeSpindexer.getBallCount() < 3) {
//                    intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.INTAKING);
//                } else {
//                    intakeSpindexer.getMotor(IntakeSpindexer_shreyas.MotorNames.intake).setPower(1);
//                    intakeSpindexer.rotateSpindexer60(1);
//
//                    new Thread(() -> {
//                        try {
//                            Thread.sleep(200);
//                        } catch (Exception e) {
//                            throw new RuntimeException(e);
//                        }
//                        intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.IDLE);
//                    }).start();
//                }
//            }
//        } else if (!gamepad1.x) {
//            XisPressed = false;
//        }
//
//        // --- INDEXED SHOOT (Y) ---
//        if (gamepad1.yWasReleased()) {
//            turretShooter.getMotor(TurretShooter_shreyas.MotorNames.leftShooter).setVelocity(300);
//            turretShooter.getMotor(TurretShooter_shreyas.MotorNames.rightShooter).setVelocity(300);
//
//            new Thread(() -> {
//                try {
//                    turretShooter.shootIndexed(TurretShooter_shreyas.ShootCase.PPG, 1000, 0.5);
//                } catch (Exception e) {
//                    throw new RuntimeException(e);
//                }
//            }).start();
//        }
//
//        // --- MANUAL SHOOT (B) ---
//        if (gamepad1.b && !BisPressed) {
//            BisPressed = true;
//            turretShooter.shoot(2500, 0.95);
//        } else if (!gamepad1.b) {
//            BisPressed = false;
//        }
//
//        // --- SIMPLE SHOOT (A) ---
//        if (gamepad1.a && !AisPressed && intakeSpindexer.ballsLoaded > 0) {
//            AisPressed = true;
//            turretShooter.simpleShootSequence(
//                    velocity, hoodPose, firstWait, secondThirdWait, intakeSpindexer
//            );
//        } else if (!gamepad1.a) {
//            AisPressed = false;
//        }
//
//        if (intakeSpindexer.ballsLoaded == 0) {
//            turretShooter.getMotor(TurretShooter_shreyas.MotorNames.leftShooter).setVelocity(0);
//            turretShooter.getMotor(TurretShooter_shreyas.MotorNames.rightShooter).setVelocity(0);
//        }
//
//        // --- TUNING CONTROLS ---
//        if (gamepad1.dpadUpWasReleased()) velocity += 100;
//        if (gamepad1.dpadDownWasReleased()) velocity -= 100;
//        if (gamepad2.dpadUpWasReleased()) velocity += 50;
//        if (gamepad2.dpadDownWasReleased()) velocity -= 50;
//        if (gamepad1.dpadRightWasReleased()) hoodPose += 0.05;
//        if (gamepad1.dpadLeftWasReleased()) hoodPose -= 0.05;
//        if (gamepad1.rightBumperWasReleased()) firstWait += 100;
//        if (gamepad1.leftBumperWasReleased()) firstWait -= 100;
//        if (gamepad2.rightBumperWasReleased()) secondThirdWait += 100;
//        if (gamepad2.leftBumperWasReleased()) secondThirdWait -= 100;
//
//        // --- INTAKE FAILSAFE ---
//        if (intakeSpindexer.ballsLoaded > 0 &&
//                intakeSpindexer.state == IntakeSpindexer_shreyas.IntakeState.IDLE) {
//            intakeSpindexer.getMotor(IntakeSpindexer_shreyas.MotorNames.intake).setPower(0.8);
//        }
//
//        double d1 = intakeSpindexer.getColorSensor1().getDistance(DistanceUnit.CM);
//        double d2 = intakeSpindexer.getColorSensor2().getDistance(DistanceUnit.CM);
//
//        boolean ballDetected = (d1 < 3.0 || d2 < 3.0);
//
//
//            NormalizedRGBA c1 = intakeSpindexer.getColorSensor1().getNormalizedColors();
//            NormalizedRGBA c2 = intakeSpindexer.getColorSensor2().getNormalizedColors();
//
//        float red = Math.max(c1.red, c2.red);
//        float green = Math.max(c1.green,c2.green);
//        float blue = Math.max(c1.blue,c2.blue);
//        int rgbRED = (int) (10000 * red);
//        int rgbGREEN = (int) (10000 * green);
//        int rgbBLUE = (int) (10000 * blue);
//        float[] hsv = new float[3];
//        Color.RGBToHSV(rgbRED,rgbGREEN,rgbBLUE, hsv);
//        float hue = hsv[0];
//
//            telemetry.addLine("=== BALL DETECTED (<3cm) ===");
//            telemetry.addData("RED",   rgbRED);
//            telemetry.addData("GREEN", rgbGREEN);
//            telemetry.addData("BLUE",  rgbBLUE);
//            telemetry.addData("HUE",  hue);
//
//        if (d1 < 3 || d2 < 3 ) {
//                if (green - red > 0.003) {
//                    telemetry.addLine("Purple Ball detected");
//                } else {
//                    telemetry.addLine("Green Ball detected");
//                }
//            }


        // --- TELEMETRY ---
//        telemetry.addData("Balls Loaded", intakeSpindexer.getBallCount());
        /*telemetry.addData("Velocity", velocity);
        telemetry.addData("Hood Pose", hoodPose);
        telemetry.addData("First Wait", firstWait);
        telemetry.addData("2nd / 3rd", secondThirdWait);*/
        telemetry.addData("Ball Colors", intakeSpindexer.ballColors.toString());
        telemetry.update();
    }

    @Override
    public void stop() {
        turretShooter.stop();
    }
}