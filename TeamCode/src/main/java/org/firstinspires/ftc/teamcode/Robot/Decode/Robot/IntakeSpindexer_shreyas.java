package org.firstinspires.ftc.teamcode.Robot.Decode.Robot;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

public class IntakeSpindexer_shreyas {

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
    boolean lastBallDetected = false;

    public enum BallColor {
        P,
        G
    }

    public int ballsLoaded = 0;
    public ArrayList<BallColor> ballColors = new ArrayList<>();

    //region COLOR VALUES
    //  PURPLE thresholds
    private static final int PURPLE_RED_MIN = 883;
    private static final int PURPLE_RED_MAX = 892;
    private static final int PURPLE_GREEN_MIN = 1005;
    private static final int PURPLE_GREEN_MAX = 1015;
    private static final int PURPLE_BLUE_MIN = 1718;
    private static final int PURPLE_BLUE_MAX = 1731;

    private static final int PURPLE_HOLE_RED_MIN = 560;
    private static final int PURPLE_HOLE_RED_MAX = 567;
    private static final int PURPLE_HOLE_GREEN_MIN = 637;
    private static final int PURPLE_HOLE_GREEN_MAX = 645;
    private static final int PURPLE_HOLE_BLUE_MIN = 1092;
    private static final int PURPLE_HOLE_BLUE_MAX = 1102;

    //  GREEN thresholds
    private static final int GREEN_RED_MIN = 405;
    private static final int GREEN_RED_MAX = 413;
    private static final int GREEN_GREEN_MIN = 1715;
    private static final int GREEN_GREEN_MAX = 1725;
    private static final int GREEN_BLUE_MIN = 1267;
    private static final int GREEN_BLUE_MAX = 1277;

    private static final int GREEN_HOLE_RED_MIN = 172;
    private static final int GREEN_HOLE_RED_MAX = 183;
    private static final int GREEN_HOLE_GREEN_MIN = 726;
    private static final int GREEN_HOLE_GREEN_MAX = 737;
    private static final int GREEN_HOLE_BLUE_MIN = 540;
    private static final int GREEN_HOLE_BLUE_MAX = 551;
//endregion

    public enum IntakeState {
        IDLE,
        INTAKING,
        INDEXING
    }

    public IntakeState state = IntakeState.IDLE;

    public void setIntakeState(IntakeState State) {
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

    public IntakeSpindexer_shreyas(OpMode opMode) {
        this.opmode = opMode;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;
        ballsLoaded = 0;
        ballColors.clear();

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
        frontSensor2 = hardwareMap.get(RevColorSensorV3.class, "front2");

        addTelemetry("IntakeSpindexer", "Ready");
    }

    public DcMotorEx getMotor(MotorNames name) {
        return DcMotorsEx.get(name.ordinal());
    }

    public Servo getServo(ServoNames name) {
        return Servos.get(name.ordinal());
    }

    public void rotateSpindexer120(int direction, boolean checkForArray) {
        if (ballColors.size() < 3 && checkForArray) {
            int red = (frontSensor1.red() + frontSensor2.red()) / 2;
            int green = (frontSensor1.green() + frontSensor2.green()) / 2;
            int blue = (frontSensor1.blue() + frontSensor2.blue()) / 2;
            BallColor color = classifyBallColor(red, green, blue);
                ballColors.add(color);
        }
        getServo(ServoNames.spin1)
                .setPosition(getServo(ServoNames.spin1).getPosition() + (SPIN_STEP*direction));
        getServo(ServoNames.spin2)
                .setPosition(getServo(ServoNames.spin2).getPosition() + (SPIN_STEP*direction));
    }

    public void rotateSpindexer60() {
        getServo(ServoNames.spin1)
                .setPosition(getServo(ServoNames.spin1).getPosition() + SPIN_STEP / 2);
        getServo(ServoNames.spin2)
                .setPosition(getServo(ServoNames.spin2).getPosition() + SPIN_STEP / 2);
    }

    public void update() {
        double dist = frontSensor1.getDistance(DistanceUnit.MM);
        boolean ballDetected = dist < BALL_DIST_MM;

        switch (state) {

            case INTAKING:
                getMotor(MotorNames.intake).setPower(0.7);

                if (ballDetected && !lastBallDetected && ballsLoaded < 3) {
                    ballsLoaded++;
                    rotateSpindexer120(1, true);
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

        addTelemetry("Balls Loaded", ballsLoaded);
        addTelemetry("Intake State", state);
    }

    public boolean hasBalls() {
        return ballsLoaded > 0;
    }

    public void consumeBall() {
        if (ballsLoaded > 0) {
            ballsLoaded--;
            rotateSpindexer120(1, true);
        }
    }

    public int getBallCount() {
        return ballsLoaded;
    }

    public void addTelemetry(String caption, Object value) {
        if (isTelemetryEnabled) {
            telemetry.addData(caption, value);
        }
    }

    public RevColorSensorV3 getColorSensor1() {
        return frontSensor1;
    }

    public RevColorSensorV3 getColorSensor2() {
        return frontSensor2;
    }

    private BallColor classifyBallColor(int red, int green, int blue) {
        boolean normalPurple =
                (red >= PURPLE_RED_MIN && red <= PURPLE_RED_MAX ) &&
                        (green >= PURPLE_GREEN_MIN && green <= PURPLE_GREEN_MAX) &&
                        (blue  >= PURPLE_BLUE_MIN  && blue  <= PURPLE_BLUE_MAX);
        boolean holePurple =
                (red >= PURPLE_HOLE_RED_MIN && red <= PURPLE_HOLE_RED_MAX ) &&
                        (green >= PURPLE_HOLE_GREEN_MIN && green <= PURPLE_HOLE_GREEN_MAX) &&
                        (blue  >= PURPLE_HOLE_BLUE_MIN  && blue  <= PURPLE_HOLE_BLUE_MAX);

        boolean normalGreen =
                (red >= GREEN_RED_MIN && red <= GREEN_RED_MAX ) &&
                        (green >= GREEN_GREEN_MIN && green <= GREEN_GREEN_MAX) &&
                        (blue  >= GREEN_BLUE_MIN  && blue  <= GREEN_BLUE_MAX);
        boolean holeGreen =
                (red >= GREEN_HOLE_RED_MIN && red <= GREEN_HOLE_RED_MAX ) &&
                        (green >= GREEN_HOLE_GREEN_MIN && green <= GREEN_HOLE_GREEN_MAX) &&
                        (blue  >= GREEN_HOLE_BLUE_MIN  && blue  <= GREEN_HOLE_BLUE_MAX);
        if (normalPurple || holePurple) {
            return BallColor.P;
        }
        if (normalGreen || holeGreen) {
            return BallColor.G;
        }
        return null;
    }
    public void clearBallColors() {
        this.ballColors.clear();
        this.ballsLoaded = 0;
    }

}
