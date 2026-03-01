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
    public ArrayList<Servo>     Servos;

    public OpMode      opmode;
    public Telemetry   telemetry;
    public HardwareMap hardwareMap;

    RevColorSensorV3 frontSensor1,     frontSensor2;
    RevColorSensorV3 backRightSensor1, backRightSensor2;
    RevColorSensorV3 backLeftSensor1,  backLeftSensor2;

    PIDController spindexerPID;

    private boolean manualBlockOverride = false;

    // endregion

    // region ===== TUNABLE CONSTANTS =====

    public static double p = 0.01, i = 0.0, d = 0.0005;

    public static double FRONT_THRESHOLD = 45;   // mm
    public static double BACK_THRESHOLD  = 15;   // mm
    public static double GREEN_THRESHOLD = 200;  // raw green channel

    public static double BLOCK_CLOSED = 0.2525;
    public static double BLOCK_OPEN   = 0.105;

    public static double INTAKE_STALL_AMPS   = 8.75;
    public static double INTAKE_REVERSE_SECS = 0.4;

    // How far forward (ticks) to advance spindexer when ready to shoot
    public static double READY_OFFSET = 170;

    // endregion

    // region ===== ENUMS =====

    public enum SpindexerState { INTAKING, READY_TO_SHOOT, SHOOTING }

    public enum MotorNames { intake, spindexer }

    public enum ServoNames { spindexerBlock }

    // endregion

    // region ===== STATE =====

    private SpindexerState spindexerState  = SpindexerState.INTAKING;
    private double         spindexerHome   = 0;
    private double         spindexerTarget = 0;
    private boolean        blockClosed     = false;

    private int ballCount   = 0;
    private int loopCounter = 0;

    // Sensor distance cache (updated every 10 loops)
    private double f1  = 9999, f2  = 9999;
    private double br1 = 9999, br2 = 9999;
    private double bl1 = 9999, bl2 = 9999;

    // Jam reversal
    private boolean           jamReversing = false;
    private final ElapsedTime jamTimer     = new ElapsedTime();

    // Manual expel toggle
    private boolean manualExpel = false;

    // Shoot timer
    private final ElapsedTime shootTimer = new ElapsedTime();

    private boolean           fullExpelling = false;
    private boolean           fullExpelDone = false;
    private final ElapsedTime fullExpelTimer = new ElapsedTime();

    private boolean           readyExpelling = false;
    private final ElapsedTime readyExpelTimer = new ElapsedTime();
    boolean teleop;

    // endregion

    // region ===== CONSTRUCTOR =====

    public IntakeSpindexer(OpMode opMode, boolean teleop) {
        this.opmode      = opMode;
        this.telemetry   = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;
        this.teleop = teleop;

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
            Servos.add(hardwareMap.get(Servo.class, name.toString()));
        }

        frontSensor1     = hardwareMap.get(RevColorSensorV3.class, "f1");
        frontSensor2     = hardwareMap.get(RevColorSensorV3.class, "f2");
        backRightSensor1 = hardwareMap.get(RevColorSensorV3.class, "br1");
        backRightSensor2 = hardwareMap.get(RevColorSensorV3.class, "br2");
        backLeftSensor1  = hardwareMap.get(RevColorSensorV3.class, "bl1");
        backLeftSensor2  = hardwareMap.get(RevColorSensorV3.class, "bl2");

        spindexerHome   = getMotor(MotorNames.spindexer).getCurrentPosition();
        spindexerTarget = spindexerHome;
        spindexerPID    = new PIDController(p, i, d);

        telemetry.addData("IntakeSpindexer", "Ready");
    }

    // endregion

    // region ===== ACCESSORS =====

    public DcMotorEx      getMotor(MotorNames name)  { return DcMotorsEx.get(name.ordinal()); }
    public Servo          getServo(ServoNames name)  { return Servos.get(name.ordinal());     }
    public int            getBallCount()             { return ballCount;                       }
    public SpindexerState getSpindexerState()        { return spindexerState;                 }

    public void manualOverrideSpindexerBlock(boolean forceClose) {
        manualBlockOverride = forceClose;
    }

    public void setSpindexerBlockPosition() {
        getServo(ServoNames.spindexerBlock).setPosition((blockClosed || manualBlockOverride) ? BLOCK_CLOSED : BLOCK_OPEN);
    }

    public void update(boolean triggerReady, boolean triggerShoot) {

        spindexerPID.setPID(p, i, d);
        loopCounter++;

        // Poll sensors every 10 loops
        if(teleop) {
            if (loopCounter % 25 == 0) {
                updateSensors();
            }
        } else {
            if (loopCounter % 10 == 0) {
                updateSensors();
            }
        }

        // --- Ball count ---
        boolean seeFront = f1 < FRONT_THRESHOLD || f2 < FRONT_THRESHOLD;
        boolean seeBR    = br1 < BACK_THRESHOLD  || br2 < BACK_THRESHOLD;
        boolean seeBL    = bl1 < BACK_THRESHOLD  || bl2 < BACK_THRESHOLD;

        int raw = (seeFront ? 1 : 0) + (seeBR ? 1 : 0) + (seeBL ? 1 : 0);
        if (raw > ballCount) ballCount = raw; // count only goes up

        boolean full = ballCount >= 3;

        // --- Block servo: close if full, or if explicitly closed by a state transition ---
        if (full) blockClosed = true;
        setSpindexerBlockPosition();

        // --- State transitions ---
        if (triggerReady && spindexerState != SpindexerState.SHOOTING) {
            blockClosed     = true;
            readyExpelling   = true;
            readyExpelTimer.reset();
            spindexerHome   = getMotor(MotorNames.spindexer).getCurrentPosition();
            spindexerTarget = spindexerHome + READY_OFFSET;
            spindexerState  = SpindexerState.READY_TO_SHOOT;
        }

        if (triggerShoot) {
            shootTimer.reset();
            spindexerState = SpindexerState.SHOOTING;
        }

        // --- State machine ---
        switch (spindexerState) {

            case INTAKING:
                spindexerTarget = spindexerHome;
                if (!full) blockClosed = false;
                break;

            case READY_TO_SHOOT:
                // target already set on transition — just hold it
                break;

            case SHOOTING:
                getMotor(MotorNames.spindexer).setPower(-0.7);
                if (shootTimer.seconds() >= 0.8) {
                    ballCount       = 0;
                    blockClosed     = false;
                    fullExpelling = false;
                    fullExpelDone = false;
                    double currentPos = getMotor(MotorNames.spindexer).getCurrentPosition();
                    spindexerHome = Math.ceil(currentPos / 384.5) * 384.5;
                    spindexerTarget = spindexerHome;
                    spindexerState  = SpindexerState.INTAKING;
                    if(manualBlockOverride) {
                        manualBlockOverride = false;
                    }
                }
                runIntake();
                updateTelemetry(raw);
                return; // skip PID while shooting motor is driven directly
        }

        // PID drive spindexer
        double pos = getMotor(MotorNames.spindexer).getCurrentPosition();
        double pid = spindexerPID.calculate(pos, spindexerTarget);
        pid = Math.max(-0.45, Math.min(0.45, pid));
        getMotor(MotorNames.spindexer).setPower(pid);

        runIntake();
        updateTelemetry(raw);
    }

    // endregion

    public double getTotalCurrent() {
        return getMotor(MotorNames.intake).getCurrent(CurrentUnit.AMPS) + getMotor(MotorNames.spindexer).getCurrent(CurrentUnit.AMPS);
    }

    // region ===== INTAKE =====

    private void runIntake() {

        if (manualBlockOverride) {
            getMotor(MotorNames.intake).setPower(-0.6);
            return;
        }

        if (readyExpelling) {
            getMotor(MotorNames.intake).setPower(-0.5);
            if (readyExpelTimer.seconds() >= INTAKE_REVERSE_SECS) readyExpelling = false;
            return;
        }

        double current = getMotor(MotorNames.intake).getCurrent(CurrentUnit.AMPS);

        // 1. Jam: overcurrent → expel burst
        if (current > INTAKE_STALL_AMPS && !jamReversing) {
            jamReversing = true;
            jamTimer.reset();
        }
        if (jamReversing) {
            getMotor(MotorNames.intake).setPower(-0.5);
            if (jamTimer.seconds() >= INTAKE_REVERSE_SECS) jamReversing = false;
            return;
        }

        if (ballCount >= 3) {
            if (!fullExpelDone) {
                if (!fullExpelling) {
                    fullExpelling = true;
                    fullExpelTimer.reset();
                }
                getMotor(MotorNames.intake).setPower(-0.5);
                if (fullExpelTimer.seconds() >= INTAKE_REVERSE_SECS) {
                    fullExpelling = false;
                    fullExpelDone = true;
                }
            } else {
                getMotor(MotorNames.intake).setPower(0.0);
            }
            return;
        }

        // 3. Normal intaking only while INTAKING state
        if (spindexerState == SpindexerState.INTAKING && Math.abs(getMotor(MotorNames.spindexer).getCurrentPosition() - spindexerHome) < 15) {
            getMotor(MotorNames.intake).setPower(1.0);
        } else {
            getMotor(MotorNames.intake).setPower(0.0);
        }
    }

    // endregion

    // region ===== HELPERS =====

    private void updateSensors() {
        f1  = frontSensor1.getDistance(DistanceUnit.MM);
        f2  = frontSensor2.getDistance(DistanceUnit.MM);
        br1 = backRightSensor1.getDistance(DistanceUnit.MM);
        br2 = backRightSensor2.getDistance(DistanceUnit.MM);
        bl1 = backLeftSensor1.getDistance(DistanceUnit.MM);
        bl2 = backLeftSensor2.getDistance(DistanceUnit.MM);
    }

    private void updateTelemetry(int raw) {
        telemetry.addData("State",            spindexerState);
        telemetry.addData("Ball Count",       ballCount + "  (raw: " + raw + ")");
        telemetry.addData("Block Closed",     blockClosed);
        telemetry.addData("Manual Expel",     manualExpel);
        telemetry.addData("Spindexer Pos",    getMotor(MotorNames.spindexer).getCurrentPosition());
        telemetry.addData("Manual Override", manualBlockOverride);
        telemetry.addData("Spindexer Target", spindexerTarget);
        telemetry.addLine("=== DISTANCES ===");
        telemetry.addData("F",  String.format("%.1f / %.1f", f1, f2));
        telemetry.addData("BR", String.format("%.1f / %.1f", br1, br2));
        telemetry.addData("BL", String.format("%.1f / %.1f", bl1, bl2));
    }

    // endregion
}