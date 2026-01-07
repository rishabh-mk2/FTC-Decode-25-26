package org.firstinspires.ftc.teamcode.Robot.Decode.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.IntakeSpindexer_shreyas;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.TurretShooter_shreyas;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@TeleOp(name = "Teleop FINAL Actual", group = "TeleOp")
public class Teleopo_FINAL extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
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
    double time = 0.0;
    boolean shoot1 = false;
    boolean shoot2 = false;
    double stopShooterTime = 0.0;
    DcMotorEx turret, intake, leftShooter, rightShooter;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        intakeSpindexer = new IntakeSpindexer_shreyas(this, true);
        turretShooter = new TurretShooter_shreyas(this, Alliance.RED, intakeSpindexer);

        telemetry.setAutoClear(true);
        runtime.reset();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
        follower.startTeleopDrive(true);
        runtime.reset();
    }

    @Override
    public void loop() {
        // --- DRIVE ---
        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x * 0.5,
                true
        );

        // --- SUBSYSTEM UPDATES ---
        turretShooter.trackAprilTag();
        intakeSpindexer.update(runtime.milliseconds());

        // INTAKE
        if(gamepad1.xWasReleased()) {
            intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.INTAKING);
        }

        // SHOOT SEQUENCE
        if(gamepad1.yWasReleased()) {
            turretShooter.setShooterVelocity(1750);
            time = runtime.milliseconds();
            shoot1 = true;
        }
        if(shoot1 && runtime.milliseconds() - time > 1500) {
            shoot1 = false;
            if (intakeSpindexer.ballsLoaded == 1) {
                stopShooterTime = 400 + 100 + 250;
            } else if (intakeSpindexer.ballsLoaded == 2) {
                stopShooterTime = 400 + 100 + 100 + 800 + 100 + 250;
            } else if (intakeSpindexer.ballsLoaded == 3) {
                stopShooterTime = 100 + 100 + 400 + 100 + 100 + 400 + 100 + 100 + 250;
            }
            intakeSpindexer.kickstartShootChain = true;
            intakeSpindexer.shootDone = false;
            intakeSpindexer.shootCallTime = runtime.milliseconds();
            intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.SHOOTING);
            time = runtime.milliseconds();
            shoot2 = true;
        }
        if(shoot2 && runtime.milliseconds() - time > stopShooterTime) {
            shoot2 = false;
            turretShooter.setShooterVelocity(0);
        }

        // SORTING

        // FAILSAFES



        // --- TELEMETRY ---
//        telemetry.addData("Balls Loaded", intakeSpindexer.getBallCount());
//        telemetry.addData("Intake State", intakeSpindexer.hasBalls());
//        telemetry.addData("Velocity", velocity);
//        telemetry.addData("Hood Pose", hoodPose);
//        telemetry.addData("First Wait", firstWait);
//        telemetry.addData("2nd / 3rd", secondThirdWait);
//        telemetry.addData("Ball Colors", intakeSpindexer.ballColors.toString());
        telemetry.addData("Balls Loaded", intakeSpindexer.ballsLoaded);
        telemetry.addData("Shoot call time",intakeSpindexer.shootCallTime);
        telemetry.addData("runtime",runtime.milliseconds());
//        telemetry.addData("Array Size", intakeSpindexer.ballColors.size());
//        for (int i=0; i < intakeSpindexer.ballColors.size(); i++) {
//            telemetry.addData("Ball" + (i + 1), intakeSpindexer.ballColors.get(i).name());
//        }
        telemetry.update();
    }

    @Override
    public void stop() {
        turretShooter.stop();
    }
}