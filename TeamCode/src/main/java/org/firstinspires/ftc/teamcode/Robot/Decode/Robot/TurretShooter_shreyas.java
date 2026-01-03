package org.firstinspires.ftc.teamcode.Robot.Decode.Robot;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;

import java.util.ArrayList;
import java.util.List;

public class TurretShooter_shreyas {
//region SETUP
    public ArrayList<DcMotorEx> DcMotorsEx;
    public ArrayList<Servo> Servos;

    public OpMode opmode;
    public Telemetry telemetry;
    public HardwareMap hardwareMap;

    public boolean isTelemetryEnabled = true;
    IntakeSpindexer_shreyas intakeSpindexer;

    PIDController turretPID;
    Limelight3A limelight;
    //endregion
//region ENUMS
    public enum MotorNames {
        leftShooter,
        rightShooter,
        turret
    }

    public enum ServoNames {
        hood
    }
    public enum ShootCase {
        PPG,
        PGP,
        GPP,
        GG,
        INVALID
    }
    public enum intakedOrder {
        PP,
        PG,
        GP,
        GG
    }
    private boolean isShooting = false;
    //endregion
    //region CONSTRUCTOR
    public TurretShooter_shreyas(OpMode opMode, Alliance alliance, IntakeSpindexer_shreyas intakeSpindexer){
        this.opmode = opMode;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;
        this.intakeSpindexer = intakeSpindexer;

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

        getMotor(MotorNames.leftShooter).setDirection(DcMotorEx.Direction.REVERSE);
        getMotor(MotorNames.rightShooter).setDirection(DcMotorEx.Direction.FORWARD);

        turretPID = new PIDController(0.0175, 0, 0.5);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        if (alliance == Alliance.RED) {
            limelight.pipelineSwitch(0);
        } else if (alliance == Alliance.BLUE) {
            limelight.pipelineSwitch(1);
        }
        limelight.start();
        addTelemetry("TurretShooter", "Ready");
    }
    //endregion

    public DcMotorEx getMotor(MotorNames name) {
        return DcMotorsEx.get(name.ordinal());
    }

    public Servo getServo(ServoNames name) {
        return Servos.get(name.ordinal());
    }

    public void setShooterVelocity(double velocity) {
        getMotor(MotorNames.leftShooter).setVelocity(velocity);
        getMotor(MotorNames.rightShooter).setVelocity(velocity);
    }
    public void shoot(double velocity, double hoodPosition){
        getServo(ServoNames.hood).setPosition(hoodPosition);
        getMotor(MotorNames.leftShooter).setVelocity(velocity);
        getMotor(MotorNames.rightShooter).setVelocity(velocity);
    }
    public void stopShooter() {
        getServo(ServoNames.hood).setPosition(0);
        getMotor(MotorNames.leftShooter).setVelocity(0);
        getMotor(MotorNames.rightShooter).setVelocity(0);
    }

    public void updateTurretTracking() {
        LLResult result = limelight.getLatestResult();
        if (!result.isValid()) {
            getMotor(MotorNames.turret).setPower(0);
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            double target = getMotor(MotorNames.turret).getCurrentPosition()
                    - (537.7 * fiducial.getTargetYDegrees()) / 360.0;

            double power = turretPID.calculate(
                    getMotor(MotorNames.turret).getCurrentPosition(),
                    target
            );

            getMotor(MotorNames.turret).setPower(power);
        }
    }
    public void addTelemetry(String caption, Object value) {
        if (isTelemetryEnabled) {
            telemetry.addData(caption, value);
        }
    }

    public void stop() {
        limelight.stop();
        stopShooter();
        getMotor(MotorNames.turret).setPower(0);
    }

    // Stuff i added below

    public intakedOrder getIntakeOrder (ArrayList<IntakeSpindexer_shreyas.BallColor> balls) {
        if (balls == null || balls.size() < 2) {
            return intakedOrder.GG;
        }
        IntakeSpindexer_shreyas.BallColor first  = balls.get(0);
        IntakeSpindexer_shreyas.BallColor second = balls.get(1);

        if (first == null || second == null) {
            return intakedOrder.GG;  // default behavior
        }

        if (first == IntakeSpindexer_shreyas.BallColor.P && second == IntakeSpindexer_shreyas.BallColor.P) {
            return intakedOrder.PP;
        }

        if (first == IntakeSpindexer_shreyas.BallColor.P && second == IntakeSpindexer_shreyas.BallColor.G) {
            return intakedOrder.PG;
        }

        if (first == IntakeSpindexer_shreyas.BallColor.G && second == IntakeSpindexer_shreyas.BallColor.P) {
            return intakedOrder.GP;
        }
        return intakedOrder.GG;
    }
    public void shootIndexed(ShootCase shootCase) {
        if (isShooting) return;
        if (intakeSpindexer == null || intakeSpindexer.ballColors.size() < 2) return;

        ArrayList<IntakeSpindexer_shreyas.BallColor> firstTwo = new ArrayList<>(
                intakeSpindexer.ballColors.subList(0, 2)
        );

        intakedOrder intakeOrder = getIntakeOrder(firstTwo);
        new Thread(() -> {
            isShooting = true;
            try {
                switch(shootCase) {
                    case PPG:
                        //region PPG cases
                        if (intakeOrder == intakedOrder.PP){
                            Thread.sleep(1000);
                            shootCW(); // first shoot
                            Thread.sleep(1000);
                            shootCW(); // second shoot
                            Thread.sleep(1000);
                            shootCW();
                            Thread.sleep(1000);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin1).setPosition(0.0);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin2).setPosition(0.0);
                            intakeSpindexer.ballColors.clear();
                        } else if (intakeOrder == intakedOrder.PG) {
                            intakeSpindexer.rotateSpindexer120(-1, false);
                            Thread.sleep(1000);
                            shootCW(); // first shoot
                            Thread.sleep(1000);
                            shootCW(); // second shoot
                            Thread.sleep(1000);
                            shootCW();
                            Thread.sleep(1000);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin1).setPosition(0.0);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin2).setPosition(0.0);
                            intakeSpindexer.ballColors.clear();
                        } else if (intakeOrder == intakedOrder.GP) {
                            Thread.sleep(1000);
                            shootCCW(); // first shoot
                            Thread.sleep(1000);
                            shootCCW(); // second shoot
                            Thread.sleep(1000);
                            shootCW(); // second shoot
                            Thread.sleep(1000);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin1).setPosition(0.0);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin2).setPosition(0.0);
                            intakeSpindexer.ballColors.clear();
                        }
                        break;
                    //endregion
                    case PGP:
                        //region PGP cases
                        if (intakeOrder == intakedOrder.PP){
                            Thread.sleep(1000);
                            shootCCW(); // first shoot
                            Thread.sleep(500);
                            shootCCW(); // second shoot
                            Thread.sleep(500);
                            shootCCW();
                            Thread.sleep(100);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin1).setPosition(0.0);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin2).setPosition(0.0);
                            intakeSpindexer.ballColors.clear();
                        } else if (intakeOrder == intakedOrder.PG) {
                            intakeSpindexer.rotateSpindexer120(-1, false);
                            Thread.sleep(100);
                            shootCCW(); // first shoot
                            Thread.sleep(500);
                            shootCCW(); // second shoot
                            Thread.sleep(500);
                            shootCCW();
                            Thread.sleep(100);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin1).setPosition(0.0);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin2).setPosition(0.0);
                            intakeSpindexer.ballColors.clear();
                        } else if (intakeOrder == intakedOrder.GP) {
                            Thread.sleep(1000);
                            shootCW(); // first shoot
                            Thread.sleep(500);
                            shootCW(); // second shoot
                            Thread.sleep(500);
                            shootCW();
                            Thread.sleep(100);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin1).setPosition(0.0);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin2).setPosition(0.0);
                            intakeSpindexer.ballColors.clear();
                        }
                        break;
                    //endregion
                    case GPP:
                        //region GPP cases
                        if (intakeOrder == intakedOrder.PP){
                            intakeSpindexer.rotateSpindexer120(1, false);
                            Thread.sleep(100);
                            shootCW(); // first shoot
                            Thread.sleep(500);
                            shootCW(); // second shoot
                            Thread.sleep(500);
                            shootCW();
                            Thread.sleep(100);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin1).setPosition(0.0);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin2).setPosition(0.0);
                            intakeSpindexer.ballColors.clear();
                        } else if (intakeOrder == intakedOrder.PG) {
                            Thread.sleep(1000);
                            shootCW(); // first shoot
                            Thread.sleep(500);
                            shootCW(); // second shoot
                            Thread.sleep(500);
                            shootCW();
                            Thread.sleep(100);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin1).setPosition(0.0);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin2).setPosition(0.0);
                            intakeSpindexer.ballColors.clear();
                        } else if (intakeOrder == intakedOrder.GP) {
                            intakeSpindexer.rotateSpindexer120(-1, false);
                            Thread.sleep(100);
                            shootCW(); // first shoot
                            Thread.sleep(500);
                            shootCW(); // second shoot
                            Thread.sleep(500);
                            shootCW();
                            Thread.sleep(100);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin1).setPosition(0.0);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin2).setPosition(0.0);
                            intakeSpindexer.ballColors.clear();
                        }
                        break;
                    //endregion
            /* TODO: add this after if my current logic works
            case GG:
                break; */
                }
            } catch (Exception e) {
                telemetry.addLine("ERROR ---- RUNTIME EXCEPTION");
            } finally {
                isShooting = false;
            }
        }).start();
    }
    //region Helper Shoot Functions
    public void shootCCW() {
        try {
            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.kicker).setPosition(0.4);
            Thread.sleep(500);
            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.kicker).setPosition(0.225);
            Thread.sleep(500);
            intakeSpindexer.rotateSpindexer120(1, false);
            intakeSpindexer.ballsLoaded--;
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    public void shootCW() {
        try {
            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.kicker).setPosition(0.4);
            Thread.sleep(500);
            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.kicker).setPosition(0.225);
            Thread.sleep(500);
            intakeSpindexer.rotateSpindexer120(-1, false);
            intakeSpindexer.ballsLoaded--;
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    //endregion
    // TODO: double check the state machine to find any logic which may hit the limit 1.0 on the servo
    // TODO: if the 1.0 limit is reached, try fixing it in the statemachine as best as possible
}
