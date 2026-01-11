package org.firstinspires.ftc.teamcode.Robot.Decode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.IntakeSpindexer_shreyas;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.TurretShooter_shreyas;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Config
@TeleOp(name = "Red TeleOp", group = "TeleOp")
public class RED_Teleopo_FINAL extends OpMode {
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
    public double hoodPose = 0.5;
    public int firstWait = 800;
    public int secondThirdWait = 300;
    Servo hood;
    double time = 0.0;
    boolean shoot1 = false;
    boolean shoot2 = false;
    double stopShooterTime = 0.0;
    DcMotorEx turret, intake, leftShooter, rightShooter;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashBoardTelemetry = dashboard.getTelemetry();
    public static double targetVel = 0.0;
    double limelightOffset = 0.0;
    private PIDController controller;
    public static double p = 0.75, i = 0, d = 0.05, f = 1.0;

    boolean slowMode = false;
    boolean manualTurret = false;
//    boolean manualSpindexer = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        intakeSpindexer = new IntakeSpindexer_shreyas(this, true);
        turretShooter = new TurretShooter_shreyas(this, Alliance.RED);

        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);

        intakeSpindexer.getMotor(IntakeSpindexer_shreyas.MotorNames.intake).setPower(1.0);

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
        if(gamepad2.bWasPressed()) {
            slowMode = !slowMode;
        }
        if(gamepad2.aWasPressed()) {
            manualTurret = !manualTurret;
        }
//        if(gamepad2.yWasPressed()) {
//            manualSpindexer = !manualSpindexer;
//        }

        if(slowMode) {
            follower.setTeleOpDrive(
                    -0.15*gamepad1.left_stick_y,
                    -0.15*gamepad1.left_stick_x,
                    -0.15*gamepad1.right_stick_x * 0.4,
                    true
            );
        } else {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x * 0.4,
                    true
            );
        }

        if(manualTurret) {
            if(gamepad2.dpad_right) {
                turretShooter.getMotor(TurretShooter_shreyas.MotorNames.turret).setPower(-0.2);
            } else if (gamepad2.dpad_left) {
                turretShooter.getMotor(TurretShooter_shreyas.MotorNames.turret).setPower(0.2);
            } else {
                turretShooter.getMotor(TurretShooter_shreyas.MotorNames.turret).setPower(0.0);
            }
        } else {
            turretShooter.trackAprilTag(3);
        }

        // --- SUBSYSTEM UPDATES ---
        double distance = turretShooter.getAprilTagDistance();
        limelightOffset = turretShooter.getLLOffset(distance);
//        turretShooter.trackAprilTag(turretShooter.getLLOffset(distance));

        intakeSpindexer.update(runtime.milliseconds());
        turretShooter.getServo(TurretShooter_shreyas.ServoNames.hood).setPosition(turretShooter.getHoodPosFromLL(distance));

        double currentVelocity = turretShooter.getMotor(TurretShooter_shreyas.MotorNames.rightShooter).getVelocity();
        double pid = controller.calculate(currentVelocity, targetVel);
        double velocity = pid + f*targetVel;

        turretShooter.setShooterVelocity(velocity);
        // INTAKE
        if(gamepad1.xWasReleased()) {
            intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.INTAKING);
        }

        // SHOOT SEQUENCE
        if(gamepad1.yWasReleased()) {
            targetVel = turretShooter.getRPMFromLL(distance);
            time = runtime.milliseconds();
            shoot1 = true;
        }
        if(shoot1 && runtime.milliseconds() - time > 1500) {
            shoot1 = false;
            if(distance < 90) {
                if (intakeSpindexer.ballsLoaded == 1) {
                    stopShooterTime = 250 + 150 + 150 + 250;
                } else if (intakeSpindexer.ballsLoaded == 2) {
                    stopShooterTime = 250 + 150 + 150 + 400 + 150 + 150 + 250;
                } else if (intakeSpindexer.ballsLoaded == 3) {
                    stopShooterTime = 150 + 150 + 400 + 150 + 150 + 400 + 150 + 150 + 250;
                }
                intakeSpindexer.kickstartShootChain = true;
                intakeSpindexer.shootDone = false;
                intakeSpindexer.shootCallTime = runtime.milliseconds();
                intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.SHOOTING);
            } else {
                if (intakeSpindexer.ballsLoaded == 1) {
                    stopShooterTime = 250 + 150 + 150 + 250;
                } else if (intakeSpindexer.ballsLoaded == 2) {
                    stopShooterTime = 250 + 150 + 150 + 500 + 150 + 150 + 250;
                } else if (intakeSpindexer.ballsLoaded == 3) {
                    stopShooterTime = 150 + 150 + 500 + 150 + 150 + 500 + 150 + 150 + 250;
                }
                intakeSpindexer.kickstartShootChain = true;
                intakeSpindexer.shootDone = false;
                intakeSpindexer.shootCallTime = runtime.milliseconds();
                intakeSpindexer.setIntakeState(IntakeSpindexer_shreyas.IntakeState.SHOOTING_FAR);
            }



            time = runtime.milliseconds();
            shoot2 = true;
        }
        if(shoot2 && runtime.milliseconds() - time > stopShooterTime) {
            shoot2 = false;
            targetVel = 0;
        }

        // SORTING

        // FAILSAFES

//        dashBoardTelemetry.addData("PID", pid);
        dashBoardTelemetry.addData("Target Velocity", targetVel);
        dashBoardTelemetry.addData("Distance", turretShooter.getAprilTagDistance());
        dashBoardTelemetry.addData("Current Velocity", turretShooter.getMotor(TurretShooter_shreyas.MotorNames.rightShooter).getVelocity());
        dashBoardTelemetry.update();

        telemetry.update();
    }

    @Override
    public void stop() {
        turretShooter.stop();
    }
}