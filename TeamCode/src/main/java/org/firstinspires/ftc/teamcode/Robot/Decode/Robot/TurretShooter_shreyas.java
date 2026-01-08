package org.firstinspires.ftc.teamcode.Robot.Decode.Robot;

import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;

import java.util.ArrayList;
import java.util.List;

public class TurretShooter_shreyas {
    //region SETUP
    public ArrayList<DcMotorEx> DcMotorsEx;
    public ArrayList<Servo> Servos;

    public OpMode opmode;
    public Telemetry telemetry;
    public HardwareMap hardwareMap;

    public boolean isTelemetryEnabled = true;
    private Limelight3A limelight;
    public PIDController turretPID;
    public LLResult result;
    private static final double TICKS_PER_REV = 537.7;
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
    public TurretShooter_shreyas(OpMode opMode, Alliance alliance){
        this.opmode = opMode;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

        DcMotorsEx = new ArrayList<>();
        Servos = new ArrayList<>();

        for (MotorNames name : MotorNames.values()) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name.toString());
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            DcMotorsEx.add(motor);
        }

        for (ServoNames name : ServoNames.values()) {
            Servo servo = hardwareMap.get(Servo.class, name.toString());
            Servos.add(servo);
        }

        getMotor(MotorNames.leftShooter).setDirection(DcMotorEx.Direction.REVERSE);
        getMotor(MotorNames.rightShooter).setDirection(DcMotorEx.Direction.FORWARD);

        // TUNABLE: Start with these values and tune using the method described
        turretPID = new PIDController(0.02, 0.0, 0.5);
        turretPID.setPID(0.02, 0.0, 0.5);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        if (alliance == Alliance.RED) {
            limelight.pipelineSwitch(0);
        } else if (alliance == Alliance.BLUE) {
            limelight.pipelineSwitch(1);
        }
        limelight.start();
        addTelemetry("TurretShooter", "Ready");
    }
    //endregion

    public DcMotorEx getMotor(MotorNames name) {
        return DcMotorsEx.get(name.ordinal());
    }

    public Servo getServo(ServoNames name) {
        return Servos.get(name.ordinal());
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

        getMotor(MotorNames.turret).setPower(pid);

        telemetry.addData("Fiducial ID", id);
        telemetry.addData("Current Target Degrees", currentTargetDeg);
        telemetry.addData("Current Pos", turretPos);
        telemetry.addData("PID Power", pid);
    }

    public void setShooterVelocity(double velocity) {
        getMotor(MotorNames.leftShooter).setVelocity(velocity);
        getMotor(MotorNames.rightShooter).setVelocity(velocity);
    }

    public void shoot(double velocity, double hoodPosition){
        getServo(ServoNames.hood).setPosition(hoodPosition);
        getMotor(MotorNames.leftShooter).setVelocity(velocity);
        getMotor(MotorNames.rightShooter).setVelocity(velocity);
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