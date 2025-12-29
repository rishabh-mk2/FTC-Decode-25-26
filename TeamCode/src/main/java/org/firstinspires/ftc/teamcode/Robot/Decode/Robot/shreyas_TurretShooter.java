package org.firstinspires.ftc.teamcode.Robot.Decode.Robot;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;

import java.util.ArrayList;

public class shreyas_TurretShooter {

    public ArrayList<DcMotorEx> DcMotorsEx;
    public ArrayList<Servo> Servos;

    public OpMode opmode;
    public Telemetry telemetry;
    public HardwareMap hardwareMap;

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

    public shreyas_TurretShooter(OpMode opMode, Alliance alliance) {
        this.opmode = opMode;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

        DcMotorsEx = new ArrayList<>();
        Servos = new ArrayList<>();

        for (MotorNames name : MotorNames.values()) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name.toString());
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            DcMotorsEx.add(motor);
        }

        for (ServoNames name : ServoNames.values()) {
            Servo servo = hardwareMap.get(Servo.class, name.toString());
            Servos.add(servo);
        }

        getMotor(MotorNames.leftShooter).setDirection(DcMotorEx.Direction.FORWARD);
        getMotor(MotorNames.rightShooter).setDirection(DcMotorEx.Direction.REVERSE);

        turretPID = new PIDController(0, 0, 0);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
    }

    public DcMotorEx getMotor(MotorNames name) {
        return DcMotorsEx.get(name.ordinal());
    }

    public Servo getServo(ServoNames name) {
        return Servos.get(name.ordinal());
    }

    public void stop() {
        for (DcMotorEx motor : DcMotorsEx) {
            motor.setPower(0);
        }
        limelight.stop();
    }
}
