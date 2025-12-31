package org.firstinspires.ftc.teamcode.Robot.Decode.Robot;

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

    public boolean isTelemetryEnabled = true;

    RevColorSensorV3 frontSensor1;
    RevColorSensorV3 frontSensor2;

    static final double BALL_DIST_MM = 20;
    static final double SPIN_STEP = 0.194;

    public static int ballsLoaded = 0;
    boolean lastBallDetected = false;

    public enum IntakeState {
        IDLE,
        INTAKING,
        INDEXING
    }

    IntakeState state = IntakeState.IDLE;
    public void setIntakeState(IntakeState State){
        state = State;
    }
    long indexStartTime = 0;

    public enum MotorNames {
        intake
    }

    public enum ServoNames {
        spin1,
        spin2,
        kicker
    }

    public IntakeSpindexer(OpMode opMode) {
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

        getServo(ServoNames.spin1).setPosition(0.0);
        getServo(ServoNames.spin2).setPosition(0.0);
        getServo(ServoNames.kicker).setPosition(0.225);

        frontSensor1 = hardwareMap.get(RevColorSensorV3.class, "front1");
        frontSensor2= hardwareMap.get(RevColorSensorV3.class, "front2");
        ballsLoaded = 0;
        addTelemetry("IntakeSpindexer", "Ready");
    }

    public DcMotorEx getMotor(MotorNames name) {
        return DcMotorsEx.get(name.ordinal());
    }

    public Servo getServo(ServoNames name) {
        return Servos.get(name.ordinal());
    }

    public void rotateSpindexer120() {
        getServo(ServoNames.spin1)
                .setPosition(getServo(ServoNames.spin1).getPosition() + SPIN_STEP);
        getServo(ServoNames.spin2)
                .setPosition(getServo(ServoNames.spin2).getPosition() + SPIN_STEP);
    }
    public void rotateSpindexer60() {
        getServo(ServoNames.spin1)
                .setPosition(getServo(ServoNames.spin1).getPosition() + 0.096);
        getServo(ServoNames.spin2)
                .setPosition(getServo(ServoNames.spin2).getPosition() + 0.096);
    }

    public void update() {
        double dist = frontSensor1.getDistance(DistanceUnit.MM);
        boolean ballDetected = dist < BALL_DIST_MM;

        switch (state) {

            case INTAKING:
                getMotor(MotorNames.intake).setPower(0.7);

                if (ballDetected && !lastBallDetected && ballsLoaded < 3) {
                    ballsLoaded++;
                    rotateSpindexer120();
                    getMotor(MotorNames.intake).setPower(0);
                    //getServo(ServoNames.kicker).setPosition(0.4); // kick up
                    indexStartTime = System.currentTimeMillis();
                    //state = IntakeState.INDEXING;
                }
                break;

            case INDEXING:
                if (System.currentTimeMillis() - indexStartTime > 100) {
                    getServo(ServoNames.kicker).setPosition(0.225); // kick down
                }
                if (System.currentTimeMillis() - indexStartTime > 250) {
                    state = ballsLoaded < 3 ? IntakeState.INTAKING : IntakeState.IDLE;
                }
                break;

            case IDLE:
                getMotor(MotorNames.intake).setPower(0);
                break;
        }

        lastBallDetected = ballDetected;

        //addTelemetry("Balls Loaded", ballsLoaded);
        //addTelemetry("Intake State", state);
    }

    public boolean hasBalls() {
        return ballsLoaded > 0;
    }

    public void consumeBall() {
        if (ballsLoaded > 0) {
            ballsLoaded--;
            rotateSpindexer120();
        }
    }

    public static int getBallCount() {
        return ballsLoaded;
    }

    public void addTelemetry(String caption, Object value) {
        if (isTelemetryEnabled) {
            telemetry.addData(caption, value);
            telemetry.update();
        }
    }
    public RevColorSensorV3 getColorSensor1() {
        return frontSensor1;
    }

    public RevColorSensorV3 getColorSensor2() {
        return frontSensor2;
    }


}
