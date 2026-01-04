package org.firstinspires.ftc.teamcode.Robot.Decode.Robot;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
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

    static final double BALL_DIST_MM = 30;
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
    private static final int PURPLE_RED_MIN = 753;
    private static final int PURPLE_RED_MAX = 922;
    private static final int PURPLE_GREEN_MIN = 825;
    private static final int PURPLE_GREEN_MAX = 1135;
    private static final int PURPLE_BLUE_MIN = 1450;
    private static final int PURPLE_BLUE_MAX = 1851;

    private static final int PURPLE_HOLE_RED_MIN = 420;
    private static final int PURPLE_HOLE_RED_MAX = 657;
    private static final int PURPLE_HOLE_GREEN_MIN = 485;
    private static final int PURPLE_HOLE_GREEN_MAX = 675;
    private static final int PURPLE_HOLE_BLUE_MIN = 920;
    private static final int PURPLE_HOLE_BLUE_MAX = 1152;

    //  GREEN thresholds
    private static final int GREEN_RED_MIN = 285;//
    private static final int GREEN_RED_MAX = 433;
    private static final int GREEN_GREEN_MIN = 1595;
    private static final int GREEN_GREEN_MAX = 1805;
    private static final int GREEN_BLUE_MIN = 1147;
    private static final int GREEN_BLUE_MAX = 1397;

    private static final int GREEN_HOLE_RED_MIN = 102;
    private static final int GREEN_HOLE_RED_MAX = 353;
    private static final int GREEN_HOLE_GREEN_MIN = 606;
    private static final int GREEN_HOLE_GREEN_MAX = 857;
    private static final int GREEN_HOLE_BLUE_MIN = 420;
    private static final int GREEN_HOLE_BLUE_MAX = 671;
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
    public boolean wantStart0;

    public IntakeSpindexer_shreyas(OpMode opMode, boolean wantStart0) {
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
        if (wantStart0) {
            getServo(ServoNames.spin1).setPosition(0.0);
            getServo(ServoNames.spin2).setPosition(0.0);
            getServo(ServoNames.kicker).setPosition(0.225);
        } else if (!wantStart0) {
            // dont do anything
            getServo(ServoNames.kicker).setPosition(0.225);
        }

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

        //double newPos1 = getServo(ServoNames.spin1).getPosition() + (SPIN_STEP * direction);
        //double newPos2 = getServo(ServoNames.spin2).getPosition() + (SPIN_STEP * direction);

        /*if (newPos1 > 1.0) newPos1 -= 1.0;
        if (newPos1 < 0.0) newPos1 += 1.0;
        if (newPos2 > 1.0) newPos2 -= 1.0;
        if (newPos2 < 0.0) newPos2 += 1.0;*/

        getServo(ServoNames.spin1)
                .setPosition(getServo(ServoNames.spin1).getPosition() + (SPIN_STEP * direction));
        getServo(ServoNames.spin2)
                .setPosition(getServo(ServoNames.spin2).getPosition() + (SPIN_STEP * direction));
    }

    public void rotateSpindexer60() {
        getServo(ServoNames.spin1)
                .setPosition(getServo(ServoNames.spin1).getPosition() + (0.097));
        getServo(ServoNames.spin2)
                .setPosition(getServo(ServoNames.spin2).getPosition() + (0.097));
    }

    // Variable to lock the intake while a ball is being moved
    private boolean isProcessingBall = false;
    private long lastBallProcessedTime = 0;
    private static final long DETECTION_COOLDOWN_MS = 500; // 100ms cooldown

    public void update() {
        double dist1 = frontSensor1.getDistance(DistanceUnit.MM);
        double dist2 = frontSensor2.getDistance(DistanceUnit.MM);
        boolean ballDetected = (dist1 < BALL_DIST_MM) || (dist2 < BALL_DIST_MM);
        long currentTime = System.currentTimeMillis();

        switch (state) {
            case INTAKING:
                getMotor(MotorNames.intake).setPower(0.75);

                // Rising edge with cooldown timer
                if (ballDetected && !lastBallDetected && ballsLoaded < 3 &&
                        (currentTime - lastBallProcessedTime) > DETECTION_COOLDOWN_MS) {

                    lastBallProcessedTime = currentTime;

                    NormalizedRGBA c1 = frontSensor1.getNormalizedColors();
                    NormalizedRGBA c2 = frontSensor2.getNormalizedColors();
                    float red = (c1.red + c2.red)/2;
                    float green = (c1.green + c2.green)/2;
                    float blue = (c1.blue + c2.blue)/2;
                    BallColor detectedColor = classifyBallColor(red, green, blue);


                    if (ballColors.size() < 3 && detectedColor != null) {
                        ballColors.add(detectedColor);
                    }
                    ballsLoaded++;
                    if (ballsLoaded < 3) {
                        rotateSpindexer120(1, false);
                    } else if (ballsLoaded == 3) {
                        rotateSpindexer60();
                    }

                    indexStartTime = currentTime;
                    state = IntakeState.INDEXING;
                }
                break;

            case INDEXING:
                if (System.currentTimeMillis() - indexStartTime > 100) {
                    getServo(ServoNames.kicker).setPosition(0.225);
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
            rotateSpindexer120(1, false);
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

    private BallColor classifyBallColor(float red, float green, float blue) {
        float[] hsv = new float[3];
        Color.RGBToHSV((int) (red * 255), (int) (green * 255), (int) (blue * 255), hsv);
        float hue = hsv[0];
        /*if (normalPurple || holePurple) {
            return BallColor.P;
        }
        if (normalGreen || holeGreen) {
            return BallColor.G;
        }*/
        if (hue >= 80 && hue <= 160) {
            return BallColor.G;
        } else if (hue >= 260 && hue <= 330) {
            return BallColor.P;
        }
        return null;
    }
    public void clearBallColors() {
        this.ballColors.clear();
        this.ballsLoaded = 0;
    }

}
