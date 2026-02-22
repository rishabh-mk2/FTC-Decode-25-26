package org.firstinspires.ftc.teamcode.Robot.Decode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

@Config
public class IntakeSpindexer {

    // region ===== SETUP =====

    public ArrayList<DcMotorEx> DcMotorsEx;
    public ArrayList<Servo> Servos;

    public OpMode opmode;
    public Telemetry telemetry;
    public HardwareMap hardwareMap;

    RevColorSensorV3 frontSensor1, frontSensor2;
    RevColorSensorV3 backRightSensor1, backRightSensor2;
    RevColorSensorV3 backLeftSensor1, backLeftSensor2;

    PIDController spindexerPID;

    // ---- TUNABLE CONSTANTS ----
    public static double p = 0.01, i = 0.0, d = 0.0005;
    public static double homeTolerance       = 15;
    public static double FRONT_THRESHOLD     = 45;   // mm
    public static double BACK_THRESHOLD      = 15;   // mm
    public static double GREEN_THRESHOLD     = 200;  // raw green value
    public static double spindexerBlockClosed = 0.2525; // hood closed (3 balls)
    public static double spindexerBlockOpen   = 0.105; // hood open  (<3 balls)
    public static double INTAKE_STALL_AMPS   = 9.0;
    public static double INTAKE_REVERSE_SECS = 0.2;

    // ---- REHOMING ----
    public static int    REHOME_EVERY_N_SHOTS  = 5;    // trigger rehome after this many shots
    public static double REHOME_POWER          = 0.35; // slow creep power during homing
    public static double REHOME_BLUE_THRESHOLD = 200;  // raw blue value that indicates the home mark
    public static double REHOME_DISTANCE_GATE  = 15;   // sensor must also be within this mm to confirm

    // endregion

    // region ===== ENUMS =====

    public enum BallColor { P, G }

    public enum IntakeState { INTAKE, EXPEL, IDLE }

    public enum SpindexerState { INTAKING, READY_TO_SHOOT, SHOOTING, REHOMING }

    public enum MotorNames { intake, spindexer }

    public enum ServoNames { spindexerBlock }

    // endregion

    // region ===== STATE =====

    // Ball tracking – count can only increase
    private int confirmedBallCount = 0;

    // Spindexer
    private double spindexerHomePosition;
    private double spindexerTargetPosition;
    private SpindexerState spindexerState = SpindexerState.INTAKING;
    private boolean readyBackStepDone     = false;
    private boolean rotatedForThree       = false;

    // Intake stall reversal
    private double  intakePower = 1.0;
    private boolean intakeReversing = false;
    private final ElapsedTime intakeReverseTimer = new ElapsedTime();

    // Shooting timer
    private final ElapsedTime spinTimer = new ElapsedTime();

    // Shot counter — triggers a rehome every REHOME_EVERY_N_SHOTS shots
    private int shotCount = 0;

    // Sensor distances (updated every N loops from outside, or call updateSensors())
    private double f1Dist  = 9999, f2Dist  = 9999;
    private double br1Dist = 9999, br2Dist = 9999;
    private double bl1Dist = 9999, bl2Dist = 9999;
    private int loopCounter = 0;

    // endregion

    // region ===== CONSTRUCTOR =====

    public IntakeSpindexer(OpMode opMode) {
        this.opmode      = opMode;
        this.telemetry   = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

        DcMotorsEx = new ArrayList<>();
        Servos     = new ArrayList<>();

        // Motors
        for (MotorNames name : MotorNames.values()) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name.toString());
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            DcMotorsEx.add(motor);
        }

        // Servos
        for (ServoNames name : ServoNames.values()) {
            Servo servo = hardwareMap.get(Servo.class, name.toString());
            Servos.add(servo);
        }

        // Sensors
        frontSensor1    = hardwareMap.get(RevColorSensorV3.class, "f1");
        frontSensor2    = hardwareMap.get(RevColorSensorV3.class, "f2");
        backRightSensor1 = hardwareMap.get(RevColorSensorV3.class, "br1");
        backRightSensor2 = hardwareMap.get(RevColorSensorV3.class, "br2");
        backLeftSensor1  = hardwareMap.get(RevColorSensorV3.class, "bl1");
        backLeftSensor2  = hardwareMap.get(RevColorSensorV3.class, "bl2");

        // Spindexer home
        spindexerHomePosition   = getMotor(MotorNames.spindexer).getCurrentPosition();
        spindexerTargetPosition = spindexerHomePosition;

        spindexerPID = new PIDController(p, i, d);

        telemetry.addData("IntakeSpindexer", "Initialized");
    }

    // endregion

    // region ===== ACCESSORS =====

    public DcMotorEx getMotor(MotorNames name) {
        return DcMotorsEx.get(name.ordinal());
    }

    public Servo getServo(ServoNames name) {
        return Servos.get(name.ordinal());
    }

    // endregion

    // region ===== MAIN UPDATE =====

    /**
     * Call this every loop() iteration.
     * @param triggerShoot  true on the loop where the driver pressed "shoot"
     * @param triggerReady  true on the loop where the driver pressed "ready to shoot"
     */
    public void update(boolean triggerShoot, boolean triggerReady) {

        spindexerPID.setPID(p, i, d);
        loopCounter++;

        // 1. Poll sensors every 10 loops
        if (loopCounter % 10 == 0) {
            updateSensors();
        }

        // 2. Determine raw ball count
        boolean spindexerHomed = Math.abs(
                getMotor(MotorNames.spindexer).getCurrentPosition() - spindexerHomePosition
        ) < homeTolerance;

        boolean ballFront     = false;
        boolean ballBackLeft  = false;
        boolean ballBackRight = false;

        if (spindexerHomed) {
            ballFront     = (f1Dist < FRONT_THRESHOLD) || (f2Dist < FRONT_THRESHOLD);
            ballBackLeft  = (bl1Dist < BACK_THRESHOLD) || (bl2Dist < BACK_THRESHOLD);
            ballBackRight = (br1Dist < BACK_THRESHOLD) || (br2Dist < BACK_THRESHOLD);
        }

        int rawCount = 0;
        if (ballFront)     rawCount++;
        if (ballBackLeft)  rawCount++;
        if (ballBackRight) rawCount++;

        // 3. Ball count can only increase (never drop)
        if (rawCount > confirmedBallCount) {
            confirmedBallCount = rawCount;
        }

        // 4. Hood / block servo: close when 3 balls confirmed
        boolean full = (confirmedBallCount >= 3);
        getServo(ServoNames.spindexerBlock).setPosition(full ? spindexerBlockClosed : spindexerBlockOpen);

        // 5. When first filling to 3, trigger READY_TO_SHOOT rotation automatically
        if (full && !rotatedForThree) {
            rotatedForThree = true;
            triggerReady    = true; // treat as if driver pressed ready
        }

        // 6. Handle driver buttons → state transitions
        if (triggerShoot) {
            spinTimer.reset();
            spindexerState = SpindexerState.SHOOTING;
        } else if (triggerReady) {
            spindexerState    = SpindexerState.READY_TO_SHOOT;
            readyBackStepDone = false;
        }

        // 7. Spindexer state machine
        switch (spindexerState) {

            case INTAKING:
                spindexerTargetPosition = spindexerHomePosition;
                break;

            case READY_TO_SHOOT:
                if (!readyBackStepDone) {
                    // Advance +175 ticks toward shooter
                    spindexerTargetPosition = spindexerHomePosition + 175;

                    // Once within 2 ticks, do a small back-step to seat the ball
                    if (Math.abs(getMotor(MotorNames.spindexer).getCurrentPosition()
                            - spindexerTargetPosition) < 2) {
                        spindexerTargetPosition -= 35;
                        readyBackStepDone = true;
                    }
                }
                break;

            case SHOOTING:
                getMotor(MotorNames.spindexer).setPower(-0.7);
                if (spinTimer.seconds() >= 2.0) {
                    confirmedBallCount = 0;
                    rotatedForThree    = false;

                    // Increment shot count and decide whether to rehome
                    shotCount++;
                    if (shotCount >= REHOME_EVERY_N_SHOTS) {
                        shotCount      = 0;
                        spindexerState = SpindexerState.REHOMING;
                    } else {
                        // Go to the closest multiple of 384.5 ticks (one full spindexer revolution)
                        double enc     = getMotor(MotorNames.spindexer).getCurrentPosition();
                        long   nearest = Math.round(enc / 384.5);
                        spindexerTargetPosition = nearest * 384.5;
                        spindexerState = SpindexerState.INTAKING;
                    }
                }
                runIntake();
                updateTelemetry(rawCount);
                return;

            case REHOMING:
                rehomeSpindexer();
                runIntake();
                updateTelemetry(rawCount);
                return; // motor driven directly inside rehomeSpindexer, skip PID
        }

        // 8. PID drive for spindexer
        double currentPos = getMotor(MotorNames.spindexer).getCurrentPosition();
        double pid = spindexerPID.calculate(currentPos, spindexerTargetPosition);

        // Tighter cap during the small back-step to avoid overshooting
        if (spindexerState == SpindexerState.READY_TO_SHOOT && readyBackStepDone) {
            pid = Math.max(-0.2, Math.min(0.2, pid));
        } else {
            pid = Math.max(-0.45, Math.min(0.45, pid));
        }
        getMotor(MotorNames.spindexer).setPower(pid);

        // 9. Intake
        runIntake();

        // 10. Telemetry
        updateTelemetry(rawCount);
    }

    // endregion

    // region ===== INTAKE =====

    /** Runs the intake with stall detection and ball-count gating. */
    private void runIntake() {
        double intakeCurrent = getMotor(MotorNames.intake).getCurrent(CurrentUnit.AMPS);

        // Stall protection
        if (intakeCurrent > INTAKE_STALL_AMPS && !intakeReversing) {
            intakeReversing = true;
            intakeReverseTimer.reset();
        }

        if (intakeReversing) {
            intakePower = -0.5;
            if (intakeReverseTimer.seconds() >= INTAKE_REVERSE_SECS) {
                intakeReversing = false;
            }
        } else {
            if (confirmedBallCount >= 3) {
                intakePower = 0;
            } else if (spindexerState == SpindexerState.INTAKING) {
                intakePower = 1.0;
            }
        }

        getMotor(MotorNames.intake).setPower(intakePower);
    }

    /** Manual intake control (call instead of update() when desired). */
    public void moveIntake(IntakeState state) {
        switch (state) {
            case INTAKE: getMotor(MotorNames.intake).setPower(1.0);  break;
            case EXPEL:  getMotor(MotorNames.intake).setPower(-1.0); break;
            case IDLE:   getMotor(MotorNames.intake).setPower(0.0);  break;
        }
    }

    // endregion

    // region ===== SORTING =====

    /**
     * Computes how many cyclic rotations of the spindexer are needed to best
     * align the loaded balls with a target color order.
     *
     * @param targetSort 3-char string of 'G', 'P', or 'O' e.g. "GPP"
     * @return 0, 1, or 2 — the number of spindexer positions to rotate
     */
    public int getBestSortRotation(String targetSort) {
        char front     = detectPosition(frontSensor1,     frontSensor2,     FRONT_THRESHOLD);
        char backRight = detectPosition(backRightSensor1, backRightSensor2, BACK_THRESHOLD);
        char backLeft  = detectPosition(backLeftSensor1,  backLeftSensor2,  BACK_THRESHOLD);

        String current = "" + front + backRight + backLeft;

        String perm0 = current;
        String perm1 = current.substring(1) + current.charAt(0);
        String perm2 = current.substring(2) + current.substring(0, 2);

        int score0 = scoreMatch(perm0, targetSort);
        int score1 = scoreMatch(perm1, targetSort);
        int score2 = scoreMatch(perm2, targetSort);

        int best = 0;
        if (score1 > score0)              { best = 1; }
        if (score2 > Math.max(score0, score1)) { best = 2; }

        return best;
    }

    /** Detects whether a ball is present and its color at a sensor pair. */
    private char detectPosition(RevColorSensorV3 s1, RevColorSensorV3 s2, double threshold) {
        double d1 = s1.getDistance(DistanceUnit.MM);
        double d2 = s2.getDistance(DistanceUnit.MM);

        if (!(d1 <= threshold && d2 <= threshold)) {
            return 'O'; // no ball
        }

        RevColorSensorV3 chosen = (d1 <= d2) ? s1 : s2;
        return (chosen.green() >= GREEN_THRESHOLD) ? 'G' : 'P';
    }

    private int scoreMatch(String a, String b) {
        int score = 0;
        for (int idx = 0; idx < 3; idx++) {
            if (a.charAt(idx) == b.charAt(idx)) score++;
        }
        return score;
    }

    // endregion

    // region ===== HELPERS =====

    /**
     * Slowly rotates the spindexer until the back-right sensor reads the neon-blue
     * home marker, then resets spindexerHomePosition to that encoder value.
     * Called every loop while state == REHOMING — no blocking.
     */
    private void rehomeSpindexer() {
        // Read both back-right sensors fresh every loop during homing
        double dist1  = backRightSensor1.getDistance(DistanceUnit.MM);
        double dist2  = backRightSensor2.getDistance(DistanceUnit.MM);
        double blue1  = backRightSensor1.blue();
        double blue2  = backRightSensor2.blue();

        // Use the closer sensor's blue reading for the color check
        boolean closeEnough = (dist1 < REHOME_DISTANCE_GATE) || (dist2 < REHOME_DISTANCE_GATE);
        double  blueReading = (dist1 <= dist2) ? blue1 : blue2;
        boolean seesBlue    = closeEnough && (blueReading >= REHOME_BLUE_THRESHOLD);

        if (seesBlue) {
            // Found the home mark — latch position and return to intaking
            spindexerHomePosition   = getMotor(MotorNames.spindexer).getCurrentPosition();
            spindexerTargetPosition = spindexerHomePosition;
            spindexerState          = SpindexerState.INTAKING;
            telemetry.addData("Rehome", "Complete @ " + spindexerHomePosition);
        } else {
            // Keep creeping
            getMotor(MotorNames.spindexer).setPower(REHOME_POWER);
            telemetry.addData("Rehome", "Searching... blue=" + String.format("%.0f", blueReading));
        }
    }

    private void updateSensors() {
        f1Dist  = frontSensor1.getDistance(DistanceUnit.MM);
        f2Dist  = frontSensor2.getDistance(DistanceUnit.MM);
        br1Dist = backRightSensor1.getDistance(DistanceUnit.MM);
        br2Dist = backRightSensor2.getDistance(DistanceUnit.MM);
        bl1Dist = backLeftSensor1.getDistance(DistanceUnit.MM);
        bl2Dist = backLeftSensor2.getDistance(DistanceUnit.MM);
    }

    public int getConfirmedBallCount() { return confirmedBallCount; }

    public SpindexerState getSpindexerState() { return spindexerState; }

    private void updateTelemetry(int rawCount) {
        telemetry.addData("Spindexer Pos",    getMotor(MotorNames.spindexer).getCurrentPosition());
        telemetry.addData("Spindexer Target", spindexerTargetPosition);
        telemetry.addData("Spindexer State",  spindexerState);
        telemetry.addData("Shot Count",       shotCount + " / " + REHOME_EVERY_N_SHOTS);
        telemetry.addData("Ball Count (raw)", rawCount);
        telemetry.addData("Ball Count (confirmed)", confirmedBallCount);
        telemetry.addData("Intake Power",     intakePower);
        telemetry.addLine("=== DISTANCES ===");
        telemetry.addData("F1",  String.format("%.1f", f1Dist));
        telemetry.addData("F2",  String.format("%.1f", f2Dist));
        telemetry.addData("BR1", String.format("%.1f", br1Dist));
        telemetry.addData("BR2", String.format("%.1f", br2Dist));
        telemetry.addData("BL1", String.format("%.1f", bl1Dist));
        telemetry.addData("BL2", String.format("%.1f", bl2Dist));
    }

    // endregion
}