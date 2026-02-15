package org.firstinspires.ftc.teamcode.Robot.Decode.Robot;

import android.graphics.Color;



import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

public class IntakeSpindexer {

    public ArrayList<DcMotorEx> DcMotorsEx;
    public ArrayList<Servo> Servos;

    public OpMode opmode;
    public Telemetry telemetry;
    public HardwareMap hardwareMap;

    RevColorSensorV3 frontSensor1;
    RevColorSensorV3 frontSensor2;
    RevColorSensorV3 backRightSensor1;
    RevColorSensorV3 backRightSensor2;
    RevColorSensorV3 backLeftSensor1;
    RevColorSensorV3 backLeftSensor2;
    double ballsLoaded = 0;

    public enum BallColor {
        P,
        G
    }
    public enum IntakeState {

    }

    public enum MotorNames {
        intake, spindexer
    }

    public enum ServoNames {

    }


    public IntakeSpindexer(OpMode opMode, boolean wantStart0) {
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

        frontSensor1 = hardwareMap.get(RevColorSensorV3.class, "f1");
        frontSensor2 = hardwareMap.get(RevColorSensorV3.class, "f2");
        frontSensor1 = hardwareMap.get(RevColorSensorV3.class, "br1");
        frontSensor2 = hardwareMap.get(RevColorSensorV3.class, "br2");
        frontSensor1 = hardwareMap.get(RevColorSensorV3.class, "bl1");
        frontSensor2 = hardwareMap.get(RevColorSensorV3.class, "bl2");

        telemetry.addData("Intake_Spindexer", "Initialized");
    }

    public DcMotorEx getMotor(MotorNames name) {
        return DcMotorsEx.get(name.ordinal());
    }

    public Servo getServo(ServoNames name) {
        return Servos.get(name.ordinal());
    }

    public void homeSpindexer() {

    }

    public void update(double currentTime) {
        telemetry.addData("Balls Loaded", ballsLoaded);
    }
    double d1 = 0.0;
    double d2 = 0.0;
    public boolean ballDetected() {
        double d1 = frontSensor1.getDistance(DistanceUnit.CM);
        double d2 = frontSensor2.getDistance(DistanceUnit.CM);

        if (d1 < 3.0 || d2 < 3.0) {
            return true;
        } else {
            return false;
        }
    }

    private BallColor classifyBallColor(float red, float green, float blue) {
        float[] hsv = new float[3];
        int rgbRED = (int) (10000 * red);
        int rgbGREEN = (int) (10000 * green);
        int rgbBLUE = (int) (10000 * blue);
        Color.RGBToHSV(rgbRED, rgbGREEN, rgbBLUE, hsv);
        float hue = hsv[0];

        if (hue >= 110 && hue <= 179) {
            return BallColor.G;
        } else if (hue >= 180 && hue <= 330) {
            return BallColor.P;
        } else {
            return null;
        }
        //if ((rgbGREEN - rgbRED < 20) && (rgbBLUE > rgbGREEN && rgbGREEN < rgbBLUE + 10)) {
        /*if (rgbGREEN > rgbBLUE + 5){
            return BallColor.G;
        } else if (rgbBLUE > rgbGREEN + 10){
            return BallColor.P;
        } else {
            return null;
        }*/
    }
}