package org.firstinspires.ftc.teamcode.Robot.Decode.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
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
    public boolean isTelemetryEnabled = true;

    RevColorSensorV3 frontSensor1, frontSensor2;
    RevColorSensorV3 backRightSensor1, backRightSensor2;
    RevColorSensorV3 backLeftSensor1, backLeftSensor2;

    PIDController spindexerPID;

    // ---- TUNABLE CONSTANTS ----
    public static double p = 0.01, i = 0.0, d = 0.0005;
    public static double intakeTargetVelocity = 0.0; // ticks/sec
    public static double homeTolerance       = 15;
    public static double FRONT_THRESHOLD     = 45;   // mm
    public static double BACK_THRESHOLD      = 15;   // mm
    public static double GREEN_THRESHOLD     = 200;  // raw green value
    public static double spindexerBlockClosed = 0.2525; // hood closed (3 balls)
    public static double spindexerBlockOpen   = 0.105;  // hood open  (<3 balls)
    public static double INTAKE_STALL_AMPS   = 20.0;
    public static double INTAKE_REVERSE_SECS = 0.4;  // duration for both jam-expel and full-expel

    // ---- REHOMING ----
    public static int    REHOME_EVERY_N_SHOTS  = 5;
    public static double REHOME_POWER          = 0.1;
    public static double REHOME_BLUE_THRESHOLD = 200;
    public static double REHOME_DISTANCE_GATE  = 15;

    // endregion

    // region ===== ENUMS =====

    public enum BallColor { P, G }

    public enum IntakeState { INTAKE, EXPEL, IDLE }

    public enum SpindexerState { INTAKING, READY_TO_SHOOT, SHOOTING, REHOMING }

    public enum MotorNames { intake, spindexer }

    public enum ServoNames { spindexerBlock }

    // endregion

    // region ===== STATE =====

    // Ball tracking
    private int confirmedBallCount = 0;

    // Spindexer
    private double spindexerHomePosition;
    private double spindexerTargetPosition;
    private SpindexerState spindexerState = SpindexerState.INTAKING;
    private boolean readyBackStepDone     = false;
    private boolean rotatedForThree       = false;

    // Intake stall (jam) reversal
    private boolean intakeReversing = false;
    private final ElapsedTime intakeReverseTimer = new ElapsedTime();

    // Full-expel: when 3 balls loaded, expel briefly then stop
    private boolean fullExpelling = false;
    private boolean fullExpelDone = false;
    private final ElapsedTime fullExpelTimer = new ElapsedTime();

    // Shooting timer
    private final ElapsedTime spinTimer = new ElapsedTime();

    // Hood-close delay before spinning on shoot
    private boolean shootWaitingForHood = false;
    private final ElapsedTime shootHoodTimer = new ElapsedTime();

    // Shot counter
    private int shotCount = 0;

    // True while spindexer is travelling back to home after a shot or rehome
    private boolean returningHome = false;

    // Pre-computed target set once when triggerReady fires
    private double readyTargetPosition = 0;

    // Tracks intended block servo state — set explicitly, not derived from ball count alone
    private boolean blockClosed = false;

    // Sensor distances
    private double f1Dist  = 9999, f2Dist  = 9999;
    private double br1Dist = 9999, br2Dist = 9999;
    private double bl1Dist = 9999, bl2Dist = 9999;
    private int loopCounter = 0;

    double intakeVelocity = 0;
    private boolean manualExpelActive = false;

    // endregion

    // region ===== CONSTRUCTOR =====

    public IntakeSpindexer(OpMode opMode) {
        this.opmode      = opMode;
        this.telemetry   = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

        DcMotorsEx = new ArrayList<>();
        Servos     = new ArrayList<>();

        for (MotorNames name : MotorNames.values()) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name.toString());
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            DcMotorsEx.add(motor);
        }

        for (ServoNames name : ServoNames.values()) {
            Servo servo = hardwareMap.get(Servo.class, name.toString());
            Servos.add(servo);
        }

        frontSensor1     = hardwareMap.get(RevColorSensorV3.class, "f1");
        frontSensor2     = hardwareMap.get(RevColorSensorV3.class, "f2");
        backRightSensor1 = hardwareMap.get(RevColorSensorV3.class, "br1");
        backRightSensor2 = hardwareMap.get(RevColorSensorV3.class, "br2");
        backLeftSensor1  = hardwareMap.get(RevColorSensorV3.class, "bl1");
        backLeftSensor2  = hardwareMap.get(RevColorSensorV3.class, "bl2");

        spindexerHomePosition   = getMotor(MotorNames.spindexer).getCurrentPosition();
        spindexerTargetPosition = spindexerHomePosition;

        spindexerPID = new PIDController(p, i, d);
        spindexerPID.setPID(p, i, d);

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

    public void update(boolean triggerShoot, boolean triggerReady, boolean triggerExpel) {

        loopCounter++;

        if (loopCounter % 10 == 0) {
            updateSensors();
        }

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

        if (rawCount > confirmedBallCount) {
            confirmedBallCount = rawCount;
        }

        boolean full = (confirmedBallCount >= 3);

        // Auto-close when 3 balls detected for the first time
        if (full && !rotatedForThree) {
            rotatedForThree = true;
            blockClosed     = true;
            triggerReady    = true;
        }

        // While returning home: force block closed until we arrive
        if (returningHome) {
            blockClosed = true;
            if (spindexerHomed) {
                returningHome = false; // arrived — open block next loop
                blockClosed   = false;
            }
        }

        // Apply block servo position from flag (never overridden by ball count alone)
        getServo(ServoNames.spindexerBlock).setPosition(blockClosed ? spindexerBlockClosed : spindexerBlockOpen);

        if (triggerShoot && spindexerState == SpindexerState.READY_TO_SHOOT) {
            boolean hoodClosed = getServo(ServoNames.spindexerBlock).getPosition() >= spindexerBlockClosed - 0.01;
            if (hoodClosed) {
                // Hood already closed — start spinning immediately
                shootWaitingForHood = false;
                spinTimer.reset();
                spindexerState = SpindexerState.SHOOTING;
            } else {
                // Close the hood first, then wait before spinning
                blockClosed = true;
                shootWaitingForHood = true;
                shootHoodTimer.reset();
            }
        } else if (triggerReady) {
            blockClosed = true;
            // Compute target once here so it doesn't drift while moving
            double encNow = getMotor(MotorNames.spindexer).getCurrentPosition();
            if (Math.abs(encNow - spindexerHomePosition) >= 30) {
                long nextSlot = (long) Math.ceil(encNow / 384.5);
                readyTargetPosition = nextSlot * 384.5 + 190;
            } else {
                readyTargetPosition = spindexerHomePosition + 190;
            }
            spindexerState    = SpindexerState.READY_TO_SHOOT;
            readyBackStepDone = false;
        }

        // If we're waiting for the hood to close before shooting, poll the timer
        if (shootWaitingForHood && shootHoodTimer.seconds() >= 0.2) {
            shootWaitingForHood = false;
            spinTimer.reset();
            spindexerState = SpindexerState.SHOOTING;
        }

        addTelemetry("Manual Spindexer", manualExpelActive);

        switch (spindexerState) {

            case INTAKING:
                spindexerTargetPosition = spindexerHomePosition;
                runIntake();
                break;

            case READY_TO_SHOOT:
                if (!readyBackStepDone) {
                    spindexerTargetPosition = readyTargetPosition;
                    double enc = getMotor(MotorNames.spindexer).getCurrentPosition();
                    if (Math.abs(enc - spindexerTargetPosition) < 3) {
                        spindexerTargetPosition = readyTargetPosition - 50;
                        readyBackStepDone = true;
                    }
                }
                break;

            case SHOOTING:
                getMotor(MotorNames.spindexer).setPower(-0.7);
                if (spinTimer.seconds() >= 0.8) {
                    confirmedBallCount = 0;
                    rotatedForThree    = false;

                    shotCount++;
                    if (shotCount >= REHOME_EVERY_N_SHOTS) {
                        shotCount      = 0;
                        spindexerState = SpindexerState.REHOMING;
                    } else {
                        double enc     = getMotor(MotorNames.spindexer).getCurrentPosition();
                        long   nearest = (long) Math.ceil(enc / 384.5);
                        spindexerHomePosition = nearest * 384.5;
                        returningHome  = true;
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
                return;
        }

        double currentPos = getMotor(MotorNames.spindexer).getCurrentPosition();
        double pid = spindexerPID.calculate(currentPos, spindexerTargetPosition);

        if (spindexerState == SpindexerState.READY_TO_SHOOT && readyBackStepDone) {
            pid = Math.max(-0.1, Math.min(0.1, pid));
        } else {
            pid = Math.max(-0.45, Math.min(0.45, pid));
        }
        getMotor(MotorNames.spindexer).setPower(pid);

        runIntake();
        updateTelemetry(rawCount);
    }

    // endregion

    // region ===== INTAKE =====

    /**
     * Runs the intake with jam detection and ball-count gating.
     *
     * Priority (highest → lowest):
     *   1. Jam (overcurrent)  → expel at 0.5 power for INTAKE_REVERSE_SECS, then resume
     *   2. Full (≥3 balls)    → expel at 0.5 power for INTAKE_REVERSE_SECS, then stop
     *   3. Normal             → intake at full power (1.0)
     */
    private void runIntake() {
        // 0. Manual expel override — highest priority, bypasses all other logic
        if (manualExpelActive) {
            getMotor(MotorNames.intake).setPower(-1.0);
            return;
        } else {
            double intakeCurrent = getMotor(MotorNames.intake).getCurrent(CurrentUnit.AMPS);

            // 1. Jam: overcurrent → expel burst, highest priority
            if (intakeCurrent > INTAKE_STALL_AMPS && !intakeReversing) {
                intakeReversing = true;
                intakeReverseTimer.reset();
            }

            if (intakeReversing) {
                getMotor(MotorNames.intake).setPower(0.5);
                if (intakeReverseTimer.seconds() >= INTAKE_REVERSE_SECS) {
                    intakeReversing = false;
                }
                return; // jam handling overrides everything else
            }

            // 2. Returning home after shot/rehome → block stays down, slow-reverse intake
            if (returningHome) {
                getMotor(MotorNames.intake).setPower(-0.1);
                return;
            }

            // 3. Full: 3 balls loaded → expel briefly then stop permanently until count resets
            if (confirmedBallCount >= 3) {
                if (!fullExpelDone) {
                    if (!fullExpelling) {
                        fullExpelling = true;
                        fullExpelTimer.reset();
                    }
                    getMotor(MotorNames.intake).setPower(0.5);
                    if (fullExpelTimer.seconds() >= INTAKE_REVERSE_SECS) {
                        fullExpelling = false;
                        fullExpelDone = true;
                    }
                } else {
                    getMotor(MotorNames.intake).setPower(0.0);
                }
                return;
            }

            // Reset full-expel flags once ball count drops back below 3 (e.g. after shooting)
            fullExpelling = false;
            fullExpelDone = false;

            // 3. Normal: keep intaking
            if (spindexerState == SpindexerState.INTAKING) {
                getMotor(MotorNames.intake).setPower(1.0);
            } else {
                getMotor(MotorNames.intake).setPower(0.0);
            }
        }
    }

    public void moveIntake(IntakeState state) {
        switch (state) {
            case INTAKE: getMotor(MotorNames.intake).setPower(1.0);  break;
            case EXPEL:  getMotor(MotorNames.intake).setPower(-1.0); break;
            case IDLE:   getMotor(MotorNames.intake).setPower(0.0);  break;
        }
    }

    // endregion

    public void setIntakeVelocity(double velocity) {
        getMotor(MotorNames.intake).setVelocity(velocity);
    }

    // region ===== SORTING =====

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
        if (score1 > score0)                   { best = 1; }
        if (score2 > Math.max(score0, score1)) { best = 2; }

        return best;
    }

    private char detectPosition(RevColorSensorV3 s1, RevColorSensorV3 s2, double threshold) {
        double d1 = s1.getDistance(DistanceUnit.MM);
        double d2 = s2.getDistance(DistanceUnit.MM);

        if (!(d1 <= threshold && d2 <= threshold)) {
            return 'O';
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

    public void rehomeSpindexer() {
        double dist1  = backLeftSensor1.getDistance(DistanceUnit.MM);
        double dist2  = backLeftSensor2.getDistance(DistanceUnit.MM);
        double blue1  = backLeftSensor1.blue();
        double blue2  = backLeftSensor2.blue();

        boolean closeEnough = (dist1 < REHOME_DISTANCE_GATE) || (dist2 < REHOME_DISTANCE_GATE);
        double  blueReading = (dist1 <= dist2) ? blue1 : blue2;
        boolean seesBlue    = closeEnough && (blueReading >= REHOME_BLUE_THRESHOLD);

        if (seesBlue) {
            spindexerHomePosition   = getMotor(MotorNames.spindexer).getCurrentPosition();
            spindexerTargetPosition = spindexerHomePosition;
            returningHome           = true;
            spindexerState          = SpindexerState.INTAKING;
            telemetry.addData("Rehome", "Complete @ " + spindexerHomePosition);
        } else {
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
        telemetry.addData("Intake Reversing", intakeReversing);
        telemetry.addData("Full Expel Done",  fullExpelDone);
        telemetry.addLine("=== DISTANCES ===");
        telemetry.addData("F1",  String.format("%.1f", f1Dist));
        telemetry.addData("F2",  String.format("%.1f", f2Dist));
        telemetry.addData("BR1", String.format("%.1f", br1Dist));
        telemetry.addData("BR2", String.format("%.1f", br2Dist));
        telemetry.addData("BL1", String.format("%.1f", bl1Dist));
        telemetry.addData("BL2", String.format("%.1f", bl2Dist));
    }

    public void addTelemetry(String caption, Object value) {
        if (isTelemetryEnabled) {
            telemetry.addData(caption, value);
        }
    }

    // endregion
}