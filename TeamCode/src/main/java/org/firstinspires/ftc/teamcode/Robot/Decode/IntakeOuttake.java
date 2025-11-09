package org.firstinspires.ftc.teamcode.Robot.Decode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;


public class IntakeOuttake {
    public ArrayList<DcMotor> DcMotors;
    public ArrayList<DcMotorEx> DcMotorsEx;
    public ArrayList<Servo> Servos;
    public ArrayList<CRServo> CRServos;
    public ArrayList<HardwareDevice> Sensors;

    public OpMode opmode;
    public Telemetry telemetry;
    public boolean isTelemetryEnabled = true;
    public HardwareMap hardwareMap;

    //TODO: Create enums for Motors, Servos, CRServos, intake modes, artifact order, servo modes etc.
    public void addTelemetry(String caption, Object value) {
        if (this.isTelemetryEnabled) {
            this.telemetry.addData(caption, value);
            this.telemetry.update();
        }
    }
    //TODO: Unhide this after making sure test velocityPID works
    //PIDController pidController;
    //double p, i, d, f;
    public IntakeOuttake(OpMode opmode){
        this.opmode = opmode;
        this.telemetry = opmode.telemetry;
        this.hardwareMap = opmode.hardwareMap;

        /*
        //TODO: un-comment this block of code after creating enums for motors, servos, etc
        //TODO: repeat this block of code for the servos and sensors
        this.DcMotorsEx = new ArrayList<>();
        for (MotorNames motorName : MotorNames.values()) {
            DcMotorEx motor = (DcMotorEx) hardwareMap.dcMotor.get(motorName.toString());
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            this.DcMotorsEx.add(motor);
        }
        * */

        /*
        //TODO: unhide this stuff for the velocity PID after you know the test velocity PID works
        p = 0.00;
        i = 0.00;
        d = 0.00;
        f = 0.00;

        pidController = new PIDController(p, i, d);
        pidController.setPID(p, i, d);
        */

        addTelemetry("Intake", "Ready");

        //TODO: add the cases for the intake/spindexer/outtake
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // TODO: Add the code/cases for the intake



    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // TODO: Add the code/cases for the transfer through spindexer


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // TODO: Add the code/cases for the outtake



}