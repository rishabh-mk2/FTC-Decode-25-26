package org.firstinspires.ftc.teamcode.Robot.Decode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

public class Spindexer {
    public ArrayList<CRServo> CRServos;

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
        rightservo, leftservo
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
                //getCRServo(CRServoNames.rightservo).setPosition();
                //getCRServo(CRServoNames.leftservo).setPosition();
                break;
            case INTAKE_3:
                //getCRServo(CRServoNames.rightservo).setPosition();
                //getCRServo(CRServoNames.leftservo).setPosition();
        }
    }
}