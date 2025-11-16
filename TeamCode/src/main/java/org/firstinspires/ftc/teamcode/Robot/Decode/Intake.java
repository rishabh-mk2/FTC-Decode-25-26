package org.firstinspires.ftc.teamcode.Robot.Decode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
public class Intake {
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
    public enum MotorNames{
        intake
    }
    public enum IntakePositions{
        IN, OUT
    }

    public Intake(OpMode opMode) {
        this.opmode = opMode;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

        this.DcMotorsEx = new ArrayList<>();
        for (MotorNames motorName : MotorNames.values()) {
            DcMotorEx motor = (DcMotorEx) hardwareMap.dcMotor.get(motorName.toString());
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            this.DcMotorsEx.add(motor);
        }

        addTelemetry("Intake", "Ready");
    }
    public void moveIntake(IntakePositions intakePositions) {
        switch (intakePositions) {
            case IN:
                //getMotor(MotorNames.intake).setPower();
                break;
            case OUT:
                //getMotor(MotorNames.intake).setPower();
                break;
        }
    }

}