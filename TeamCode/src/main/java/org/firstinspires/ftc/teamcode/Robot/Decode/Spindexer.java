package org.firstinspires.ftc.teamcode.Robot.Decode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

public class Spindexer {
    public ArrayList<CRServo> CRServos;
    public ArrayList<HardwareDevice> Sensors;

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

    public enum CRServoNames {
        spinningServo, shooterServo
    }
    public enum SensorNames{
        color
    }

    public enum SpindexerMode {
        ON, OFF,INTAKE_3
    }
    public Spindexer(OpMode opmode){
        this.opmode = opmode;
        this.telemetry = opmode.telemetry;
        this.hardwareMap = opmode.hardwareMap;

        this.CRServos = new ArrayList<>();
        for (CRServoNames servoName : CRServoNames.values()) {
            CRServo servoMotor = this.hardwareMap.crservo.get(servoName.toString());
            this.CRServos.add(servoMotor);
        }

        this.Sensors = new ArrayList<>();
        for (SensorNames sensorName : SensorNames.values()) {
            RevColorSensorV3 colorSensor = (RevColorSensorV3) this.hardwareMap.colorSensor.get(sensorName.toString());
            this.Sensors.add(colorSensor);
        }
        addTelemetry("Spindexer", "Ready");

    }
    public Servo getCRServo(CRServoNames servoName) {
        return (Servo) this.CRServos.get(servoName.ordinal());
    }
    public void spin(SpindexerMode spindexerMode) {
        switch (spindexerMode) {
            case ON:
                //getCRServo(CRServoNames.rightservo).setPosition();
                //getCRServo(CRServoNames.leftservo).setPosition();
                break;
            case OFF:
                //getCRServo(CRServoNames.spinningServo).setPower(0);
                break;
            case INTAKE_3:
                //getCRServo(CRServoNames.spinningServo).setDirection(getCRServo(CRServoNames.spinningServo).getPosition() + x);
                // TODO: change the "x" val to however much we need to move the servo
        }
    }
}