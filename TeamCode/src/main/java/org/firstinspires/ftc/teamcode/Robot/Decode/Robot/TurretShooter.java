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
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private Vector shooterToGoalVector;

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
    //   The servo spans [0.0, 0.965] linearly over one full 360 deg rotation.
    //   The 0.0/0.965 overlap (dead zone) sits at (pi - 28.5/155.0) rad CW from robot-forward.
    public static double TURRET_SERVO_MIN          = 0.0;
    public static double TURRET_SERVO_MAX          = 0.965;
    public static double SHOOTER_OFFSET_INCHES     = 1.9;
    public static double TURRET_OVERLAP_OFFSET_RAD = -(Math.PI - 28.5 / 155.0);

    public static double RECOIL = 0.0;

    public Vector GOAL_POSE_VECTOR;

    public static double HOOD_MAX_ANGLE = 0;
    public static double HOOD_MIN_ANGLE = 0;

    // shoot latches true on button release and stays true for SHOOT_LATCH_SECS, then resets
    public static boolean shoot = false;
    public static double  SHOOT_LATCH_SECS  = 1.0;
    public static double  RECOIL_DELAY_SECS = 0.2;
    private final ElapsedTime shootTimer = new ElapsedTime();

    // endregion

    // region ===== LOOKUP TABLE =====

    // Lookup table for hood and recoil only — sorted by distance.
    // RPM is computed from the linear equation: rpm = 6.49775 * distance + 1209.64627
    // Hood and recoil: snap to the first row whose distance >= current distance (ceil lookup).
    private static final double[][] LOOKUP = {
            //  distance     hood    recoil
            {    38.5621,   0.225,   0.00  },
            {    46.4716,   0.400,   0.15  },
            {     52.478,   0.600,   0.15  },
            {      61.98,   0.750,   0.15  },
            {       69.3,   0.800,   0.15  },
            {      76.86,   0.925,   0.15  },
            {       83.5,   1.000,   0.15  },
            {       91.5,   1.000,   0.15  },
            {      94.13,   1.000,   0.15  },
            {       97.4,   1.000,   0.15  },
            {    99.9075,   1.000,   0.15  },
            {   116.3175,   1.000,   0.15  },
            {    121.573,   1.000,   0.15  },
            {   126.2259,   1.000,   0.15  },
            {    131.703,   1.000,   0.15  },
            {   136.3135,   1.000,   0.15  },
            {   140.3279,   1.000,   0.15  },
    };

    /** RPM from linear regression: y = 6.49775x + 1209.64627 */
    private static double rpmFromDistance(double distance) {
        return 6.49775 * distance + 1209.64627;
    }

    /**
     * Ceil lookup: finds the first row whose distance >= current distance and returns that value.
     * col: 1 = hood, 2 = recoil. Clamps to last row if distance exceeds all rows.
     */
    private static double lookupCeil(double distance, int col) {
        for (int r = 0; r < LOOKUP.length; r++) {
            if (distance <= LOOKUP[r][0]) {
                return LOOKUP[r][col];
            }
        }
        return LOOKUP[LOOKUP.length - 1][col];
    }




    // endregion

    // region ===== CONSTRUCTOR =====

    public TurretShooter(OpMode opMode, Alliance alliance) {
        this.opmode      = opMode;
        this.telemetry   = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

        DcMotorsEx = new ArrayList<>();
        Servos     = new ArrayList<>();

        for (MotorNames name : MotorNames.values()) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name.toString());
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            DcMotorsEx.add(motor);
        }

        getMotor(MotorNames.leftShooter ).setDirection(DcMotorSimple.Direction.FORWARD);
        getMotor(MotorNames.rightShooter).setDirection(DcMotorSimple.Direction.REVERSE);

        for (ServoNames name : ServoNames.values()) {
            Servo servo = hardwareMap.get(Servo.class, name.toString());
            Servos.add(servo);
        }

        shooterVelocityPID = new PIDController(p, i, d);

        if (alliance == Alliance.RED) {
            GOAL_POSE = new Pose(138, 138);
        } else {
            GOAL_POSE = new Pose(138, 138).mirror();
        }

        GOAL_POSE_VECTOR = GOAL_POSE.getAsVector();
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

    public void update(Pose pose, Vector velocity, boolean shootP) {
        updateValues(pose, velocity);
        updateTurret();
        updateShooter(shootP);
    }

    // region ===== POSE / VELOCITY =====

    // Computed each loop from the lookup table — used by updateShooter()
    private double hoodPos    = 0.0;
    private double shooterVel = 0.0;
    private double recoil     = 0.0;

    /** Call every loop with the robot's current field pose and velocity vector. */
    public void updateValues(Pose pose, Vector velocity) {
        robotVelocity = velocity;

        // Translate the raw pose 1.9 inches forward along the robot's heading
        double heading  = pose.getHeading();
        double shooterX = pose.getX() + SHOOTER_OFFSET_INCHES * Math.cos(heading);
        double shooterY = pose.getY() + SHOOTER_OFFSET_INCHES * Math.sin(heading);
        currentPose = new Pose(shooterX, shooterY, heading);

        GOAL_POSE_VECTOR    = GOAL_POSE.getAsVector().minus(velocity);
        shooterToGoalVector = GOAL_POSE_VECTOR.minus(currentPose.getAsVector());

        double dist = shooterToGoalVector.getMagnitude();
        // RPM from linear equation; hood and recoil from ceil lookup table
        shooterVel = rpmFromDistance(dist);
        hoodPos    = lookupCeil(dist, 1);
        recoil     = lookupCeil(dist, 2);

        addTelemetry("DistanceToGoal", dist);
        addTelemetry("Hood",           String.format("%.3f", hoodPos));
        addTelemetry("RPM",            String.format("%.0f", shooterVel));
        addTelemetry("Recoil",         String.format("%.3f", recoil));
        addTelemetry("Shoot",          shoot);
    }

    public void updateShooter(boolean shootP) {
//        setShooterVelocity(shooterVel);
        setShooterVelocity(500);
        setRecoil(recoil);

        // Latch shoot true on rising edge of shootP; keep latched for SHOOT_LATCH_SECS
        if (shootP && !shoot) {
            shoot = true;
            shootTimer.reset();
        }
        if (shoot && shootTimer.seconds() >= SHOOT_LATCH_SECS) {
            shoot = false;
        }

        // Hood: pull back by recoil amount only after RECOIL_DELAY_SECS into the shoot latch
        if (shoot && shootTimer.seconds() >= RECOIL_DELAY_SECS) {
            setHoodPosition(hoodPos - RECOIL);
        } else {
            setHoodPosition(hoodPos);
        }
    }

    // endregion

    // region ===== TURRET AIM =====

    public void updateTurret() {
        double goalAngleField = shooterToGoalVector.getTheta();

        double relativeRad = goalAngleField - currentPose.getHeading();
        relativeRad = Math.atan2(Math.sin(relativeRad), Math.cos(relativeRad));

        double servoFrameRad = relativeRad - TURRET_OVERLAP_OFFSET_RAD;
        servoFrameRad = Math.atan2(Math.sin(servoFrameRad), Math.cos(servoFrameRad));
        double servoFrameDeg = Math.toDegrees(servoFrameRad);

        double range         = TURRET_SERVO_MAX - TURRET_SERVO_MIN;
        double servoPosition = TURRET_SERVO_MIN + (servoFrameDeg / 360.0) * range;
        servoPosition = ((servoPosition % TURRET_SERVO_MAX) + TURRET_SERVO_MAX) % TURRET_SERVO_MAX;

        setTurretPosition(servoPosition);

        addTelemetry("Turret Rel Angle",   String.format("%.1f deg", Math.toDegrees(relativeRad)));
        addTelemetry("Turret Servo Frame", String.format("%.1f deg", servoFrameDeg));
        addTelemetry("Turret Servo Pos",   String.format("%.4f", servoPosition));
    }

    // endregion

    // region ===== SHOOTER VELOCITY PID =====

    public void updateShooterVelocityPID() {
        shooterVelocityPID.setPID(p, i, d);

        double currentVelocity = getMotor(MotorNames.leftShooter).getVelocity();

        double pid   = shooterVelocityPID.calculate(currentVelocity, targetVelocity);
        double ff    = targetVelocity * f;
        double power = Math.max(-1, Math.min(pid + ff, 1));

        getMotor(MotorNames.leftShooter ).setPower(power);
        getMotor(MotorNames.rightShooter).setPower(power);

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

    public void setTurretPosition(double position) {
        getServo(ServoNames.turret1).setPosition(position);
        getServo(ServoNames.turret2).setPosition(position);
    }

    // endregion

    // region ===== HOOD =====

    public void setHoodPosition(double position) {
        getServo(ServoNames.hood).setPosition(position);
    }

    // endregion

    public void setRecoil(double recoil) {
        RECOIL = recoil;
    }

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