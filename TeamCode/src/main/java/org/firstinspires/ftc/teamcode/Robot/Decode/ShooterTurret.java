package org.firstinspires.ftc.teamcode.Robot.Decode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
public class ShooterTurret {
    public ArrayList<CRServo> CRServos;
    public ArrayList<DcMotorEx> DcMotorsEx;
    public ArrayList<Servo> Servos;
    public OpMode opmode;
    public Telemetry telemetry;
    public boolean isTelemetryEnabled = true;
    public HardwareMap hardwareMap;
    public void addTelemetry(String caption, Object value) {
        if (this.isTelemetryEnabled) {
            this.telemetry.addData(caption, value);
            this.telemetry.update();
        }
    }
    public enum ServoNames{
        hood, turret,
    }
    public enum MotorNames{
        shooter
    }
    public enum HoodPositions {
        HOOD1, HOOD2, HOOD3
    }
    public enum ShootingPositions{
        NEAR_GOAL, FAR_GOAL
    }
    PIDController pidController;
    double p, i, d, f;

    public ShooterTurret(OpMode opMode) {

        this.opmode = opMode;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

        this.Servos = new ArrayList<>();
        for (ServoNames servoName : ServoNames.values()) {
            Servo servoMotor = this.hardwareMap.servo.get(servoName.toString());
            this.Servos.add(servoMotor);
        }

        this.DcMotorsEx = new ArrayList<>();
        for (MotorNames motorName : MotorNames.values()) {
            DcMotorEx motor = (DcMotorEx) hardwareMap.dcMotor.get(motorName.toString());
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            this.DcMotorsEx.add(motor);
        }
        /*
        p = 0.01;
        i = 0.00;
        d = 0.0005;
        f = 0.00;
        pidController = new PIDController(p, i, d);
        pidController.setPID(p, i, d);
         */
        addTelemetry("ShooterTurret", "Ready");
    }

    public Servo getServo(ServoNames servoName) {
        return this.Servos.get(servoName.ordinal());
    }
    public void moveHood(HoodPositions hoodPosition) {
        switch (hoodPosition) {
            case HOOD1:
                //getServo(ServoNames.hood).setPosition();
                break;
            case HOOD2:
                //getServo(ServoNames.hood).setPosition();
                break;
            case HOOD3:
                //getServo(ServoNames.hood).setPosition();
                break;
        }
    }
    public void shoot(ShootingPositions shootingPositions) {
        switch (shootingPositions) {
            /*case NEAR_GOAL:
                getServo(ServoNames.turret).setPosition();
                getMotor(MotorNames.shooter).setPower();
                break;
            case FAR_GOAL:
                getServo(ServoNames.turret).setPosition();
                getMotor(MotorNames.shooter).setPower();
                break;*/
        }
    }

}