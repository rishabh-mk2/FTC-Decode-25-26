package org.firstinspires.ftc.teamcode.Robot.Decode.Robot;

import com.arcrobotics.ftclib.controller.PIDController;
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

    public boolean isTelemetryEnabled = true;
    private Limelight3A limelight;
    private PIDController turretPID;
    private PIDController shooterPID;
    public LLResult result;
    private static final double TICKS_PER_REV = 384.5;
    public int id;

    //endregion
//region ENUMS
    public enum MotorNames {
        leftShooter,
        rightShooter,
        turret
    }

    public enum ServoNames {
        hood
    }
    public enum ShootCase {
        PPG,
        PGP,
        GPP,
        GG,
        INVALID
    }
    public enum intakedOrder {
        PP,
        PG,
        GP,
        GG
    }
    public boolean isShooting = false;


    //endregion
    //region CONSTRUCTOR
    public TurretShooter(OpMode opMode, Alliance alliance){
        this.opmode = opMode;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

        DcMotorsEx = new ArrayList<>();
        Servos = new ArrayList<>();

        for (MotorNames name : MotorNames.values()) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name.toString());
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            DcMotorsEx.add(motor);
        }

        for (ServoNames name : ServoNames.values()) {
            Servo servo = hardwareMap.get(Servo.class, name.toString());
            Servos.add(servo);
        }

        getMotor(MotorNames.leftShooter).setDirection(DcMotorSimple.Direction.REVERSE);
        getMotor(MotorNames.rightShooter).setDirection(DcMotorSimple.Direction.FORWARD);

        // TUNABLE: Start with these values and tune using the method described
        turretPID = new PIDController(0.015, 0.0, 0.0);
        turretPID.setPID(0.015, 0.0, 0.0);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        if (alliance == Alliance.RED) {
            limelight.pipelineSwitch(0);
        } else if (alliance == Alliance.BLUE) {
            limelight.pipelineSwitch(1);
        }
        limelight.start();
        addTelemetry("TurretShooter", "Ready");

        getServo(ServoNames.hood).setPosition(0.95);
    }
    //endregion

    public DcMotorEx getMotor(MotorNames name) {
        return DcMotorsEx.get(name.ordinal());
    }

    public Servo getServo(ServoNames name) {
        return Servos.get(name.ordinal());
    }
    public Limelight3A getLimelight() {
        return limelight;
    }

    double limelightMountAngleDegrees = 20.0;
    double limelightLensHeightInches = 16.5;
    double goalHeightInches = 29.5;

    public double getAprilTagDistance() {
        result = limelight.getLatestResult();

        if(result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            LLResultTypes.FiducialResult fiducial = fiducials.get(0);

            double angleToGoalDegrees = limelightMountAngleDegrees + fiducial.getTargetYDegrees();
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);;

            return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
        }
        return 0;
    }

    public double getAprilTagXDegrees() {
        result = limelight.getLatestResult();

        if(result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            LLResultTypes.FiducialResult fiducial = fiducials.get(0);

            return fiducial.getTargetXDegrees();
        }
        return 0;
    }

    public void trackAprilTag() {

        result = limelight.getLatestResult();

        if (!result.isValid()) {
            telemetry.addData("info", "no april tag being detected");
            getMotor(MotorNames.turret).setPower(0);
            return;
        }
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) {
            getMotor(MotorNames.turret).setPower(0);
            return;
        }

        LLResultTypes.FiducialResult fiducial = fiducials.get(0);

        id = fiducial.getFiducialId();

        double currentTargetDeg = fiducial.getTargetXDegrees();
        double turretPos = getMotor(MotorNames.turret).getCurrentPosition();

        double targetTicks = turretPos - (TICKS_PER_REV * currentTargetDeg) / 360.0;

        double pid = turretPID.calculate(turretPos, targetTicks);

        getMotor(MotorNames.turret).setPower(pid + 0.01*targetTicks);

        telemetry.addData("Fiducial ID", id);
        telemetry.addData("Current Target Degrees", currentTargetDeg);
        telemetry.addData("Current Pos", turretPos);
        telemetry.addData("PID Power", pid);
    }

    public double getRPMFromLL(double distance) {
        double rounded = Math.round(distance/5.0)*5.0;
        switch ((int) rounded){
            case 20:
                return 1500;
            case 25:
                return 1500;
            case 30:
                return 1500;
            case 35:
                return 1600;
            case 40:
                return 1600;
            case 45:
                return 1600;
            case 50:
                return 1600;
            case 55:
                return 1600;
            case 60:
                return 1600;
            case 65:
                return 1650;
            case 70:
                return 1650;
            case 75:
                return 1700;
            case 80:
                return 1700;
            case 85:
                return 1750;
            case 90:
                return 1750;
            case 95:
                return 1800;
            case 100:
                return 1900;
            case 105:
                return 1925;
            case 110:
                return 1950;
            case 115:
                return 1975;
            case 120:
                return 2000;
            case 125:
                return 2000;
            case 130:
                return 2000;
            case 135:
                return 2000;
            case 140:
                return 2000;
            case 145:
                return 2000;
            case 150:
                return 2000;
        }
        return 1700;
    }

    public double getLLOffset(double distance) {
        double rounded = Math.round(distance/5.0)*5.0;
        switch ((int) rounded){
            case 20:
                return -10;
            case 25:
                return -5;
            case 30:
                return -5;
            case 35:
                return -5;
            case 40:
                return 0;
            case 45:
                return 0;
            case 50:
                return 0;
            case 55:
                return 0;
            case 60:
                return 0;
            case 65:
                return -2;
            case 70:
                return 5;
            case 75:
                return 5;
            case 80:
                return 0;
            case 85:
                return 0;
            case 90:
                return 10;
            case 95:
                return 10;
            case 100:
                return 3;
            case 105:
                return 3;
            case 110:
                return 3;
            case 115:
                return 3;
            case 120:
                return 3;
            case 125:
                return 3;
            case 130:
                return 3;
            case 135:
                return 3;
            case 140:
                return 3;
            case 145:
                return 3;
            case 150:
                return 3;
        }
        return 0;
    }

    public double getHoodPosFromLL(double distance){
        double rounded = Math.round(distance/5.0)*5.0;
        switch ((int) rounded){
            case 20:
                return 0.0;
            case 25:
                return 0.3;
            case 30:
                return 0.4;
            case 35:
                return 0.55;
            case 40:
                return 0.8;
            case 45:
                return 0.8;
            case 50:
                return 0.85;
            case 55:
                return 0.85;
            case 60:
                return 0.85;
            case 65:
                return 0.85;
            case 70:
                return .85;
            case 75:
                return 0.85;
            case 80:
                return 0.85;
            case 85:
                return 0.85;
            case 90:
                return 0.9;
            case 95:
                return 0.9;
            case 100:
                return 1.0;
            case 105:
                return 1.0;
            case 110:
                return 1.0;
            case 115:
                return 1.0;
            case 120:
                return 1.0;
            case 125:
                return 1.0;
            case 130:
                return 1.0;
            case 135:
                return 1.0;
            case 140:
                return 1.0;
            case 145:
                return 1.0;
            case 150:
                return 1.0;
        }
        return 1.0;
    }

    public void trackAprilTag(double offset) {

        result = limelight.getLatestResult();

        if (!result.isValid()) {
            telemetry.addData("info", "no april tag being detected");
            getMotor(MotorNames.turret).setPower(0);
            return;
        }
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) {
            getMotor(MotorNames.turret).setPower(0);
            return;
        }

        LLResultTypes.FiducialResult fiducial = fiducials.get(0);

        id = fiducial.getFiducialId();

        double currentTargetDeg = fiducial.getTargetXDegrees();
        double turretPos = getMotor(MotorNames.turret).getCurrentPosition();

        double targetTicks = turretPos - ((TICKS_PER_REV * currentTargetDeg) / 360.0) + offset;

        double pid = turretPID.calculate(turretPos, targetTicks);

        getMotor(MotorNames.turret).setPower(pid + 0.01*targetTicks);

        telemetry.addData("Fiducial ID", id);
        telemetry.addData("Current Target Degrees", currentTargetDeg);
        telemetry.addData("Current Pos", turretPos);
        telemetry.addData("PID Power", pid);
    }

    public double getPID(double velocity) {
        return shooterPID.calculate(getMotor(MotorNames.rightShooter).getVelocity(), velocity);
    }

    public void setShooterVelocity(double velocity) {
//        double currentVelocity = getMotor(MotorNames.rightShooter).getVelocity();
//        double pid = shooterPID.calculate(currentVelocity, velocity);
//        getMotor(MotorNames.leftShooter).setVelocity(pid + velocity);
//        getMotor(MotorNames.rightShooter).setVelocity(pid + velocity);
        getMotor(MotorNames.leftShooter).setVelocity(velocity);
        getMotor(MotorNames.rightShooter).setVelocity(velocity);
    }

    public void setShooterPower(double power) {
        getMotor(MotorNames.leftShooter).setPower(power);
        getMotor(MotorNames.rightShooter).setPower(power);
    }

    public void shoot(double velocity, double hoodPosition){
        getServo(ServoNames.hood).setPosition(hoodPosition);
        setShooterVelocity(velocity);
    }

    public void stopShooter() {
        getServo(ServoNames.hood).setPosition(0);
        getMotor(MotorNames.leftShooter).setVelocity(0);
        getMotor(MotorNames.rightShooter).setVelocity(0);
    }

    public void addTelemetry(String caption, Object value) {
        if (isTelemetryEnabled) {
            telemetry.addData(caption, value);
        }
    }

    public void stop() {
        limelight.stop();
        stopShooter();
        getMotor(MotorNames.turret).setPower(0);
    }
}