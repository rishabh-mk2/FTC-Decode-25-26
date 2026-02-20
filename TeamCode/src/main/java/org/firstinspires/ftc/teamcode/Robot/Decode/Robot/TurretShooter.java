package org.firstinspires.ftc.teamcode.Robot.Decode.Robot;

import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;

import java.util.ArrayList;
import java.util.List;

public class TurretShooter {
    //region SETUP
    public ArrayList<DcMotorEx> DcMotorsEx;
    public ArrayList<Servo> Servos;

    public OpMode opmode;
    public Telemetry telemetry;
    public HardwareMap hardwareMap;

    PIDController shooterPID;

    Pose currentPose;
    Pose GOAL_POSE;
    Vector robotVelocity;
    Vector robotToGoalVector;

    public boolean isTelemetryEnabled = true;

    public enum MotorNames {
        leftShooter,
        rightShooter
    }

    public enum ServoNames {
        hood, turret1, turret2, turret3
    }
    public TurretShooter(OpMode opMode, Alliance alliance){
        this.opmode = opMode;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

        DcMotorsEx = new ArrayList<>();
        Servos = new ArrayList<>();

        for (MotorNames name : MotorNames.values()) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name.toString());
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            DcMotorsEx.add(motor);
        }

        for (ServoNames name : ServoNames.values()) {
            Servo servo = hardwareMap.get(Servo.class, name.toString());
            Servos.add(servo);
        }

        getMotor(MotorNames.leftShooter).setDirection(DcMotorSimple.Direction.FORWARD);
        getMotor(MotorNames.rightShooter).setDirection(DcMotorSimple.Direction.REVERSE);

        shooterPID = new PIDController(p, i, d);

        //TODO: play around with these values

        if (alliance == Alliance.RED) {
//            limelight.pipelineSwitch(0);
            GOAL_POSE = new Pose(138, 138);
        } else if (alliance == Alliance.BLUE) {
//            limelight.pipelineSwitch(1);
            GOAL_POSE = new Pose(138, 138).mirror();
        }
//        limelight.start();
        addTelemetry("TurretShooter", "Ready");
    }
    //endregion

    public DcMotorEx getMotor(MotorNames name) {
        return DcMotorsEx.get(name.ordinal());
    }

    public Servo getServo(ServoNames name) {
        return Servos.get(name.ordinal());
    }

    public void updateValues(Pose pose, Vector velocity) {
        currentPose = pose;
        robotVelocity = velocity;
        robotToGoalVector = GOAL_POSE.getAsVector().minus(pose.getAsVector());

    }

    public void updateTurretShooter() {
        // constants
        double g = 32.174 * 12;
        double x = robotToGoalVector.getMagnitude() - PASS_THROUGH_POINT_RADIUS;
        double y = SCORE_HEIGHT;
        double a = SCORE_ANGLE;

        // calculate initial launch components
        double hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x - Math.tan(a)), HOOD_MAX_ANGLE, HOOD_MIN_ANGLE);

        double flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));

        // Use robot velocity and convert into parallel and perpendicular components
        double coordinateTheta = robotVelocity.getTheta() - robotToGoalVector.getTheta();

        double parallelComponent = -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();

        // velocity compensation variables
        double vz = flywheelSpeed * Math.sin(hoodAngle);
        double time = x / (flywheelSpeed * Math.cos(hoodAngle));
        double ivr = x / time + parallelComponent;
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
        double ndr = nvr * time;

        // recalculate launch components
        hoodAngle = MathFunctions.clamp(Math.atan(vz / nvr), HOOD_MAX_ANGLE, HOOD_MIN_ANGLE);

        flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle) - y)));

        // update turret
        double turretVelCompOffset = Math.atan(perpendicularComponent / ivr);
        double turretAngle = Math.toDegrees(currentPose.getHeading() - robotToGoalVector.getTheta() + turretVelCompOffset);

        // move mechanisms
        setTurretDegree(turretAngle);
        setShooterVelocity(getFlywheelTicksFromVelocity(flywheelSpeed));
        setHoodPosition(getHoodTicksFromDegrees(hoodAngle));
    }

    // TODO: EDIT THESE
    double SCORE_HEIGHT = 26;
    double SCORE_ANGLE = Math.toRadians(-30);
    double PASS_THROUGH_POINT_RADIUS = 5;

    // in radians remember
    double HOOD_MAX_ANGLE = 0;
    double HOOD_MIN_ANGLE = 0;

    public static double p, i, d, f = 0.0;

    public void setShooterVelocity(double targetVelocity) {
        double currentVelocity = getMotor(MotorNames.leftShooter).getVelocity();

        double pid = shooterPID.calculate(currentVelocity, targetVelocity);
        double ff = targetVelocity * f;

        double power = pid + ff;

        power = Math.max(0, Math.min(power, 1));

        getMotor(MotorNames.leftShooter).setPower(power);
        getMotor(MotorNames.rightShooter).setPower(-power);
    }

    public void setHoodPosition(double position) {
        // TODO: IMPLEMENT LATER
    }

    public double getFlywheelTicksFromVelocity(double velocity) {
        return 0;

        //TODO: DO THIS Experimentally later
    }

    public double getHoodTicksFromDegrees(double degrees) {
        return 0;

        //TODO: Experimentally find this.
    }

    public void setTurretDegree(double degrees) {
        // TODO: IMPLEMENT LATER
    }


    public void addTelemetry(String caption, Object value) {
        if (isTelemetryEnabled) {
            telemetry.addData(caption, value);
        }
    }

    public void stop() {
//        limelight.stop();
//        stopShooter();
    }
}