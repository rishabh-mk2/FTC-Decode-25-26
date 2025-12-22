package org.firstinspires.ftc.teamcode.Robot.Decode.Robot;

import com.arcrobotics.ftclib.controller.PIDController;
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

public class TurretShooter {

    public ArrayList<DcMotorEx> DcMotorsEx;
    public ArrayList<Servo> Servos;

    public OpMode opmode;
    public Telemetry telemetry;
    public HardwareMap hardwareMap;

    public boolean isTelemetryEnabled = true;

    PIDController turretPID;
    Limelight3A limelight;

    public enum MotorNames {
        leftShooter,
        rightShooter,
        turret
    }

    public enum ServoNames {
        hood
    }

    public TurretShooter(OpMode opMode, Alliance alliance) {
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

        getMotor(MotorNames.leftShooter).setDirection(DcMotorEx.Direction.FORWARD);
        getMotor(MotorNames.rightShooter).setDirection(DcMotorEx.Direction.REVERSE);

        turretPID = new PIDController(0.02, 0, 0.5);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        if (alliance == Alliance.RED) {

        } else if (alliance == Alliance.BLUE) {

        }

        limelight.start();

        addTelemetry("TurretShooter", "Ready");
    }

    public DcMotorEx getMotor(MotorNames name) {
        return DcMotorsEx.get(name.ordinal());
    }

    public Servo getServo(ServoNames name) {
        return Servos.get(name.ordinal());
    }

    public void setShooterVelocity(double velocity) {
        getMotor(MotorNames.leftShooter).setVelocity(velocity);
        getMotor(MotorNames.rightShooter).setVelocity(velocity);
    }

    public void stopShooter() {
        getMotor(MotorNames.leftShooter).setVelocity(0);
        getMotor(MotorNames.rightShooter).setVelocity(0);
    }

    public void updateTurretTracking() {
        LLResult result = limelight.getLatestResult();
        if (!result.isValid()) {
            getMotor(MotorNames.turret).setPower(0);
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            double target = getMotor(MotorNames.turret).getCurrentPosition()
                    - (537.7 * fiducial.getTargetYDegrees()) / 360.0;

            double power = turretPID.calculate(
                    getMotor(MotorNames.turret).getCurrentPosition(),
                    target
            );

            getMotor(MotorNames.turret).setPower(power);
        }
    }

    public void addTelemetry(String caption, Object value) {
        if (isTelemetryEnabled) {
            telemetry.addData(caption, value);
            telemetry.update();
        }
    }

    public void stop() {
        limelight.stop();
        stopShooter();
        getMotor(MotorNames.turret).setPower(0);
    }
}
