package org.firstinspires.ftc.teamcode.Robot.Decode.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.geometry.Pose;
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

    public OpMode      opmode;
    public Telemetry   telemetry;
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
    public static double p = 0.002, i = 0.0, d = 0.0, f = 0.00036;
    public static double targetVelocity = 0.0; // ticks/sec

    // Turret servo calibration:
    //   The servo spans [0.04, 1.0] linearly over one full 360 deg rotation.
    //   The 0.04/1.0 overlap (dead zone) is NOT at robot-forward — it sits at
    //   TURRET_OVERLAP_OFFSET_RAD counterclockwise from robot-forward (top-down view).
    //   83.5/155.0 rad CCW from forward = ~30.87 deg CCW from forward.
    public static double TURRET_SERVO_MIN          = 0.0;    // servo position at the overlap boundary
    public static double TURRET_SERVO_MAX          = 0.965;  // servo position at the other overlap edge
    public static double SHOOTER_OFFSET_INCHES = 1.9;    // forward offset from odometry center to shooter
    public static double TURRET_OVERLAP_OFFSET_RAD = -(Math.PI - 28.5 / 155.0); // CW from robot-forward to overlap (negative = CW)


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
        robotVelocity = velocity;

        // Translate the raw pose 1.9 inches forward along the robot's heading
        // so all targeting calculations originate from the shooter position,
        // not the robot center reported by the odometry.
        double heading = pose.getHeading();
        double shooterX = pose.getX() + SHOOTER_OFFSET_INCHES * Math.cos(heading);
        double shooterY = pose.getY() + SHOOTER_OFFSET_INCHES * Math.sin(heading);
        currentPose = new Pose(shooterX, shooterY, heading);

        robotToGoalVector = GOAL_POSE.getAsVector().minus(currentPose.getAsVector());
    }

    // endregion

    // region ===== TURRET AIM =====

    /**
     * Aims the turret at the goal using only the current robot pose (no velocity compensation).
     * Call every loop after updateValues().
     *
     * Servo mapping:
     *   0 deg relative to robot (facing goal-forward) -> TURRET_FORWARD_POSITION (default 0.04)
     *   The full 360 deg of rotation spans [TURRET_FORWARD_POSITION, 1.0] linearly.
     *   Wrapping is handled naturally: atan2 clamps the angle to [-180, 180] deg, and
     *   a modulo ensures the result never leaves [0.0, 1.0].
     *
     *   Example positions:
     *     0 deg   (forward)  -> 0.04
     *     90 deg  (right)    -> 0.04 + 0.96 * (90/360)  = 0.28
     *     180 deg (backward) -> 0.04 + 0.96 * (180/360) = 0.52
     *     -90 deg (left)     -> 0.04 + 0.96 * (-90/360) = modulo -> 0.80
     */
    public void updateTurret() {
        // Angle from robot to goal in the field frame
        double goalAngleField = robotToGoalVector.getTheta();

        // Robot-relative angle in radians, normalised to [-pi, pi]
        double relativeRad = goalAngleField - currentPose.getHeading();
        relativeRad = Math.atan2(Math.sin(relativeRad), Math.cos(relativeRad));

        // Shift into the servo's own frame:
        // The servo zero (0.04) sits at the overlap point, which is TURRET_OVERLAP_OFFSET_RAD
        // CCW from robot-forward. So subtract that offset to convert from robot-frame to
        // servo-frame. A positive robot-relative angle (CCW) now correctly maps to a
        // higher servo value, and the overlap boundary only appears at the one blind spot
        // that is TURRET_OVERLAP_OFFSET_RAD CCW from forward.
        double servoFrameRad = relativeRad - TURRET_OVERLAP_OFFSET_RAD;

        // Normalise servo-frame angle to [-pi, pi] so the wrap is always clean
        servoFrameRad = Math.atan2(Math.sin(servoFrameRad), Math.cos(servoFrameRad));
        double servoFrameDeg = Math.toDegrees(servoFrameRad); // [-180, 180]

        // Linear map: servo-frame 0 deg -> TURRET_SERVO_MIN (0.0)
        //             full 360 deg spans [TURRET_SERVO_MIN, TURRET_SERVO_MAX]
        double range         = TURRET_SERVO_MAX - TURRET_SERVO_MIN;
        double servoPosition = TURRET_SERVO_MIN + (servoFrameDeg / 360.0) * range;

        // Wrap into [TURRET_SERVO_MIN, TURRET_SERVO_MAX]
        servoPosition = ((servoPosition % TURRET_SERVO_MAX) + TURRET_SERVO_MAX) % TURRET_SERVO_MAX;

        setTurretPosition(servoPosition);

        addTelemetry("Turret Rel Angle",   String.format("%.1f deg", Math.toDegrees(relativeRad)));
        addTelemetry("Turret Servo Frame", String.format("%.1f deg", servoFrameDeg));
        addTelemetry("Turret Servo Pos",   String.format("%.4f", servoPosition));
    }

    // endregion

    // region ===== SHOOTER VELOCITY PID =====

    /**
     * Drives both flywheel motors to match targetVelocity using PID + feedforward.
     */
    public void updateShooterVelocityPID() {
        shooterVelocityPID.setPID(p, i, d);

        double currentVelocity = getMotor(MotorNames.leftShooter).getVelocity();

        targetVelocity = 0;

        double pid   = shooterVelocityPID.calculate(currentVelocity, targetVelocity);
        double ff    = targetVelocity * f;
        double power = Math.max(-1, Math.min(pid + ff, 1));

        getMotor(MotorNames.leftShooter ).setPower( power);
        getMotor(MotorNames.rightShooter).setPower( power);

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

    /** Set a specific velocity and immediately run the PID. */
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

    // region ===== TURRET POSITION =====

    /** Writes the servo position to all turret servos. */
    public void setTurretPosition(double position) {
        getServo(ServoNames.turret1).setPosition(position);
        getServo(ServoNames.turret2).setPosition(position);
    }

    // endregion

    // region ===== HOOD =====

    public double getHoodPositionFromAngle(double angleRadians) {
        // TODO: experimentally calibrate this mapping
        return 0;
    }

    public void setHoodPosition(double position) {
        // TODO: implement once hood servo is characterised
        // getServo(ServoNames.hood).setPosition(position);
    }

    // endregion

    // region ===== CONVERSIONS =====

    public double getFlywheelTicksFromVelocity(double velocityInchesPerSec) {
        // TODO: calibrate experimentally
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