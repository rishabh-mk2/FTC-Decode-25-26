package org.firstinspires.ftc.teamcode.Robot.Decode.Robot;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

public class shreyas_IntakeSpindexer {
    double y = 2.56;
    double x = 1.19;
    public ArrayList<DcMotorEx> DcMotorsEx;
    public ArrayList<Servo> Servos;

    public OpMode opmode;
    public Telemetry telemetry;
    public HardwareMap hardwareMap;

    RevColorSensorV3 frontSensor;

    public enum MotorNames {
        intake
    }

    public enum ServoNames {
        spin1,
        spin2,
        kicker
    }

    private static final double INTAKE_POWER = 0.8;
    private static final double BALL_DETECT_DISTANCE_MM = 20.0;
    private static final double SPINDEXER_STEP = 0.169;
    private static final double KICKER_DOWN = 0.225;
    private boolean spindexerMovingForward = true;

    enum BallColor {
        GREEN,
        PURPLE
    }

    public ArrayList<BallColor> loadedBalls = new ArrayList<>();
    int spindexerSlot = 0;

    boolean previousBallDetected = false;

    public shreyas_IntakeSpindexer(OpMode opMode) {
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

        frontSensor = hardwareMap.get(RevColorSensorV3.class, "front");

        getServo(ServoNames.spin1).setPosition(0.0);
        getServo(ServoNames.spin2).setPosition(0.0);
        getServo(ServoNames.kicker).setPosition(KICKER_DOWN);
    }

    // ===================== INTAKE =====================
    public void runIntake(boolean intakeButtonHeld) {
        getMotor(MotorNames.intake).setPower(intakeButtonHeld ? INTAKE_POWER : 0);

        if (!intakeButtonHeld || loadedBalls.size() >= 3) return;

        boolean ballDetected = frontSensor.getDistance(DistanceUnit.MM) < BALL_DETECT_DISTANCE_MM;

        if (ballDetected && !previousBallDetected) {
            //detectAndStoreBall();
            moveSpindexerForNewBall();
        }

        previousBallDetected = ballDetected;
    }

    // ============ OTHER =============
    /* private void detectAndStoreBall() {
        if ((_ < frontSensor.red()) && (_ > frontSensor.red())
                && (_ < frontSensor.green()) && (_ > frontSensor.green())
                && (_ < frontSensor.blue()) && (_ > frontSensor.blue())) {
            loadedBalls.add(BallColor.GREEN);
        } else if ((_ < frontSensor.red()) && (_ > frontSensor.red())
                && (_ < frontSensor.green()) && (_ > frontSensor.green())
                && (_ < frontSensor.blue()) && (_ > frontSensor.blue())) {
            loadedBalls.add(BallColor.PURPLE);
        }
    } */

    private void moveSpindexerForNewBall() {
        double currentPos = getServo(ServoNames.spin1).getPosition();
        double nextPos = currentPos + (spindexerMovingForward ? SPINDEXER_STEP : -SPINDEXER_STEP);

        // Clamp position to [0, 1]
        nextPos = Math.max(0.0, Math.min(1.0, nextPos));

        if (nextPos >= 1.0) spindexerMovingForward = false;
        if (nextPos <= 0.0) spindexerMovingForward = true;

        getServo(ServoNames.spin1).setPosition(nextPos);
        getServo(ServoNames.spin2).setPosition(nextPos);
    }

    public DcMotorEx getMotor(MotorNames name) {
        return DcMotorsEx.get(name.ordinal());
    }

    public Servo getServo(ServoNames name) {
        return Servos.get(name.ordinal());
    }
}