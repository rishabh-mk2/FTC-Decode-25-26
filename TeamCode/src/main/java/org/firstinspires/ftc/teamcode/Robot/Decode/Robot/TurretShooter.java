package org.firstinspires.ftc.teamcode.Robot.Decode.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;

import java.util.ArrayList;

@Config
public class TurretShooter {

    // region ===== SETUP =====

    public ArrayList<DcMotorEx> DcMotorsEx;
    public ArrayList<Servo>     Servos;

    public OpMode     opmode;
    public Telemetry  telemetry;
    public HardwareMap hardwareMap;

    private PIDController shooterVelocityPID;
    private FtcDashboard  dashboard;

    private Pose   currentPose;
    private Pose   GOAL_POSE;
    private Vector robotVelocity;
    private Vector robotToGoalVector;

    public boolean isTelemetryEnabled = true;

    // endregion

    // region ===== ENUMS =====

    public enum MotorNames {
        leftShooter,
        rightShooter
    }

    public enum ServoNames {
        hood, turret1, turret2
    }

    // endregion

    // region ===== TUNABLE CONSTANTS =====

    // Velocity PID + feedforward (tune via FTC Dashboard)
    public static double p = 0.1, i = 0.0, d = 0.0, f = 0.0;
    public static double targetVelocity = 0.0; // ticks/sec — set from dashboard or TeleOp

    // Ballistic constants (tune experimentally)
    public static double SCORE_HEIGHT               = 26;              // inches
    public static double SCORE_ANGLE                = Math.toRadians(-30);
    public static double PASS_THROUGH_POINT_RADIUS  = 5;              // inches

    // Hood servo limits in radians (tune experimentally)
    public static double HOOD_MAX_ANGLE = 0;
    public static double HOOD_MIN_ANGLE = 0;

    // endregion

    // region ===== CONSTRUCTOR =====

    public TurretShooter(OpMode opMode, Alliance alliance) {
        this.opmode      = opMode;
        this.telemetry   = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

        DcMotorsEx = new ArrayList<>();
        Servos     = new ArrayList<>();

        // Motors
        for (MotorNames name : MotorNames.values()) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name.toString());
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            DcMotorsEx.add(motor);
        }

        getMotor(MotorNames.leftShooter ).setDirection(DcMotorSimple.Direction.FORWARD);
        getMotor(MotorNames.rightShooter).setDirection(DcMotorSimple.Direction.REVERSE);

        // Servos
        for (ServoNames name : ServoNames.values()) {
            Servo servo = hardwareMap.get(Servo.class, name.toString());
            Servos.add(servo);
        }

        // PID
        shooterVelocityPID = new PIDController(p, i, d);

        // Goal pose by alliance
        if (alliance == Alliance.RED) {
            GOAL_POSE = new Pose(138, 138);
        } else {
            GOAL_POSE = new Pose(138, 138).mirror();
        }

        // Dashboard for velocity tuning graph
        dashboard = FtcDashboard.getInstance();

        addTelemetry("TurretShooter", "Ready");
    }

    // endregion

    // region ===== ACCESSORS =====

    public DcMotorEx getMotor(MotorNames name) {
        return DcMotorsEx.get(name.ordinal());
    }

    public Servo getServo(ServoNames name) {
        return Servos.get(name.ordinal());
    }

    // endregion

    // region ===== POSE / VELOCITY =====

    /** Call every loop with the robot's current field pose and velocity vector. */
    public void updateValues(Pose pose, Vector velocity) {
        currentPose      = pose;
        robotVelocity    = velocity;
        robotToGoalVector = GOAL_POSE.getAsVector().minus(pose.getAsVector());
    }

    // endregion

    // region ===== FULL AUTO-AIM UPDATE =====

    /**
     * Computes and sets hood angle, flywheel speed, and turret angle with
     * full velocity compensation.  Call every loop after updateValues().
     */
    public void updateTurretShooter() {
        double g = 32.174 * 12; // in/s²
        double x = robotToGoalVector.getMagnitude() - PASS_THROUGH_POINT_RADIUS;
        double y = SCORE_HEIGHT;
        double a = SCORE_ANGLE;

        // First-pass ballistics (no velocity compensation)
        double hoodAngle    = MathFunctions.clamp(Math.atan(2 * y / x - Math.tan(a)), HOOD_MAX_ANGLE, HOOD_MIN_ANGLE);
        double flywheelSpeed = Math.sqrt(g * x * x /
                (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));

        // Decompose robot velocity into parallel / perpendicular to shot direction
        double coordinateTheta       = robotVelocity.getTheta() - robotToGoalVector.getTheta();
        double parallelComponent     = -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
        double perpendicularComponent =  Math.sin(coordinateTheta) * robotVelocity.getMagnitude();

        // Velocity compensation
        double vz   = flywheelSpeed * Math.sin(hoodAngle);
        double time = x / (flywheelSpeed * Math.cos(hoodAngle));
        double ivr  = x / time + parallelComponent;
        double nvr  = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
        double ndr  = nvr * time;

        // Second-pass ballistics (with compensation)
        hoodAngle    = MathFunctions.clamp(Math.atan(vz / nvr), HOOD_MAX_ANGLE, HOOD_MIN_ANGLE);
        flywheelSpeed = Math.sqrt(g * ndr * ndr /
                (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle) - y)));

        // Turret angle
        double turretVelCompOffset = Math.atan(perpendicularComponent / ivr);
        double turretAngle = Math.toDegrees(
                currentPose.getHeading() - robotToGoalVector.getTheta() + turretVelCompOffset);

        // Apply to mechanisms
        setTurretDegrees(turretAngle);
//        setShooterVelocity(getFlywheelTicksFromVelocity(flywheelSpeed));
        setShooterVelocity(600);
        setHoodPosition(getHoodPositionFromAngle(hoodAngle));
    }

    // endregion

    // region ===== SHOOTER VELOCITY PID =====

    /**
     * Drives both flywheel motors to match {@code targetVelocity} using PID + feedforward.
     * Uses the same pattern as ShooterVelocityPID test opmode.
     */
    public void updateShooterVelocityPID() {
        // Allow live tuning via FTC Dashboard
        shooterVelocityPID.setPID(p, i, d);

        double currentVelocity = getMotor(MotorNames.leftShooter).getVelocity();

        targetVelocity = 400;

        double pid   = shooterVelocityPID.calculate(currentVelocity, targetVelocity);
        double ff    = targetVelocity * f;
        double power = pid + ff;

        power = Math.max(0, Math.min(power, 1));

        getMotor(MotorNames.leftShooter ).setPower( power);
        getMotor(MotorNames.rightShooter).setPower( power);

        // Push to dashboard graph for PID tuning
        if (dashboard != null) {
            dashboard.getTelemetry().addData("Shooter Target",   targetVelocity);
            dashboard.getTelemetry().addData("Shooter Velocity", currentVelocity);
            dashboard.getTelemetry().addData("Shooter Power",    power);
            dashboard.getTelemetry().update();
        }

        addTelemetry("Shooter Target",   targetVelocity);
        addTelemetry("Shooter Velocity", currentVelocity);
        addTelemetry("Shooter Power",    power);
    }

    /**
     * Convenience — set a specific velocity and immediately run the PID.
     */
    public void setShooterVelocity(double velocityTicksPerSec) {
        targetVelocity = velocityTicksPerSec;
        updateShooterVelocityPID();
    }

    public void stopShooter() {
        targetVelocity = 0;
        getMotor(MotorNames.leftShooter ).setPower(0);
        getMotor(MotorNames.rightShooter).setPower(0);
    }

    // endregion

    // region ===== HOOD =====

    /** Maps a calculated launch angle (radians) to a servo position [0,1]. */
    public double getHoodPositionFromAngle(double angleRadians) {
        // TODO: experimentally calibrate this mapping
        return 0;
    }

    public void setHoodPosition(double position) {
        // TODO: implement once hood servo is characterised
        // getServo(ServoNames.hood).setPosition(position);
    }

    // endregion

    // region ===== TURRET =====

    public void setTurretDegrees(double degrees) {
        // TODO: implement once turret servo/motor is characterised
        // double pos = degrees / 270.0; // example linear mapping
        // getServo(ServoNames.turret1).setPosition(pos);
    }

    // endregion

    // region ===== CONVERSIONS =====

    /**
     * Convert a physical ball velocity (in/s) to motor ticks/s.
     * TODO: calibrate experimentally.
     */
    public double getFlywheelTicksFromVelocity(double velocityInchesPerSec) {
        return 0;
    }

    // endregion

    // region ===== TELEMETRY =====

    public void addTelemetry(String caption, Object value) {
        if (isTelemetryEnabled) {
            telemetry.addData(caption, value);
        }
    }

    // endregion

    // region ===== STOP =====

    public void stop() {
        stopShooter();
    }

    // endregion
}