package org.firstinspires.ftc.teamcode.Robot.Decode.Robot;

import android.graphics.Color;



import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.JavaUtil;
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
        G,
        NONE
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
    public boolean shootDone = true;
//endregion

    public enum IntakeState {
        IDLE,
        INTAKING,
        SHOOTING
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

    public boolean kickstartShootChain = false;
    boolean shooter1 = false;
    boolean shooter2 = false;
    boolean shooter3 = false;
    boolean shooter4 = false;
    boolean shooter5 = false;
    boolean shooter6 = false;
    boolean shooter7 = false;
    boolean shooter8 = false;


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
        }

        getServo(ServoNames.kicker).setPosition(0.225);

        frontSensor1 = hardwareMap.get(RevColorSensorV3.class, "front1");
        frontSensor2 = hardwareMap.get(RevColorSensorV3.class, "front2");

        telemetry.addData("Intake_Spindexer", "Initialized");

    }

    public DcMotorEx getMotor(MotorNames name) {
        return DcMotorsEx.get(name.ordinal());
    }

    public Servo getServo(ServoNames name) {
        return Servos.get(name.ordinal());
    }

    public void rotateSpindexer120(int direction) {
        getServo(ServoNames.spin1)
                .setPosition(getServo(ServoNames.spin1).getPosition() + (SPIN_STEP * direction));
        getServo(ServoNames.spin2)
                .setPosition(getServo(ServoNames.spin2).getPosition() + (SPIN_STEP * direction));
    }

    public void rotateSpindexer60(int direction) {
        getServo(ServoNames.spin1)
                .setPosition(getServo(ServoNames.spin1).getPosition() + (SPIN_STEP * direction)/2.0);
        getServo(ServoNames.spin2)
                .setPosition(getServo(ServoNames.spin2).getPosition() + (SPIN_STEP * direction)/2.0);
    }
    private double lastBallProcessedTime = 0;
    private static final long DETECTION_COOLDOWN_MS = 500; // 100ms cooldown
    public double shootCallTime = 0.0;

    public void homeSpindexer() {
        getServo(ServoNames.spin1).setPosition(0.0);
        getServo(ServoNames.spin2).setPosition(0.0);
    }

    public void update(double currentTime) {
        switch (state) {
            case INTAKING:
                getMotor(MotorNames.intake).setPower(0.85);
                // Rising edge with cooldown timer
                if (ballDetected() && ballsLoaded < 3 && (currentTime - lastBallProcessedTime) > DETECTION_COOLDOWN_MS) {
                    ballsLoaded += 1;
                    lastBallProcessedTime = currentTime;
                    if(ballsLoaded < 3) {
                        rotateSpindexer120(1);
                    } else {
                        lastBallDetected = true;
                        rotateSpindexer60(1);
                        getMotor(MotorNames.intake).setPower(0.5);
                    }
                }
                break;
            case SHOOTING:
                getMotor(MotorNames.intake).setPower(0.65);
                if(ballsLoaded == 1) {
                    rotateSpindexer60(1);
                    if(currentTime - shootCallTime > 400) {
                        getServo(ServoNames.kicker).setPosition(0.4);
                    }
                    if(currentTime - shootCallTime > 400 + 100) {
                        getServo(ServoNames.kicker).setPosition(0.225);
                    }
                    if(currentTime - shootCallTime > 400 + 100 + 100) {
                        homeSpindexer();
                        shootDone = true;
                    }
                }
                else if (ballsLoaded == 2) {
                    rotateSpindexer60(-1);
                    if(currentTime - shootCallTime > 400) {
                        getServo(ServoNames.kicker).setPosition(0.4);
                    }
                    if(currentTime - shootCallTime > 400 + 100) {
                        getServo(ServoNames.kicker).setPosition(0.225);
                    }
                    if(currentTime - shootCallTime > 400 + 100 + 100) {
                        rotateSpindexer120(1);
                    }
                    if(currentTime - shootCallTime > 400 + 100 + 100 + 800) {
                        getServo(ServoNames.kicker).setPosition(0.4);
                    }
                    if(currentTime - shootCallTime > 400 + 100 + 100 + 800 + 100) {
                        getServo(ServoNames.kicker).setPosition(0.225);
                    }
                    if(currentTime - shootCallTime > 400 + 100 + 100 + 800 + 100 + 100) {
                        homeSpindexer();
                        shootDone = true;
                    }
                }
                else if (ballsLoaded == 3) {
                    if(kickstartShootChain) {
                        getServo(ServoNames.kicker).setPosition(0.4);
                        shooter1 = true;
                        kickstartShootChain = false;
                    }
                    if(shooter1 && currentTime - shootCallTime > 100) {
                        getServo(ServoNames.kicker).setPosition(0.225);
                        shooter1 = false;
                        shooter2 = true;
                    }
                    if(shooter2 && currentTime - shootCallTime > 100 + 100) {
                        rotateSpindexer120(1);
                        shooter2 = false;
                        shooter3 = true;
                    }
                    if(shooter3 && currentTime - shootCallTime > 100 + 100 + 400) {
                        getServo(ServoNames.kicker).setPosition(0.4);
                        shooter3 = false;
                        shooter4 = true;
                    }
                    if(shooter4 && currentTime - shootCallTime > 100 + 100 + 400 + 100) {
                        getServo(ServoNames.kicker).setPosition(0.225);
                        shooter4 = false;
                        shooter5 = true;
                    }
                    if(shooter5 && currentTime - shootCallTime > 100 + 100 + 400 + 100 + 100) {
                        rotateSpindexer120(1);
                        shooter5 = false;
                        shooter6 = true;
                    }
                    if(shooter6 && currentTime - shootCallTime > 100 + 100 + 400 + 100 + 100 + 400) {
                        getServo(ServoNames.kicker).setPosition(0.4);
                        shooter6 = false;
                        shooter7 = true;
                    }
                    if(shooter7 && currentTime - shootCallTime > 100 + 100 + 400 + 100 + 100 + 400 + 100) {
                        getServo(ServoNames.kicker).setPosition(0.225);
                        shooter7 = false;
                        shooter8 = true;
                    }
                    if(shooter8 && currentTime - shootCallTime > 100 + 100 + 400 + 100 + 100 + 400 + 100 + 100) {
                        homeSpindexer();
                        shootDone = true;
                        shooter8 = false;
                    }
                }
                if (shootDone){
                    ballsLoaded = 0;
                }
                break;
            case IDLE:
                getMotor(MotorNames.intake).setPower(0);
                break;
        }
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