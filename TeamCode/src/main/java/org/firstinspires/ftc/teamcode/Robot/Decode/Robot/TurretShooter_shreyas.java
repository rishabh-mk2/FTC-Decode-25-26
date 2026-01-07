package org.firstinspires.ftc.teamcode.Robot.Decode.Robot;

import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
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
    private Limelight3A limelight;
    public PIDController turretPID;
    public LLResult result;
    private static final double TICKS_PER_REV = 537.7;
    public int id;

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
    public boolean isShooting = false;
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

        // TUNABLE: Start with these values and tune using the method described
        turretPID = new PIDController(0.02, 0.0, 0.5);
        turretPID.setPID(0.02, 0.0, 0.5);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

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

    public void trackAprilTag() {

        result = limelight.getLatestResult();

        if (!result.isValid()) {
            telemetry.addData("info", "no april tag being detected");
            getMotor(MotorNames.turret).setPower(0);
            return;
        }
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) {
            getMotor(MotorNames.turret).setPower(0);
            return;
        }

        LLResultTypes.FiducialResult fiducial = fiducials.get(0);

        id = fiducial.getFiducialId();
        double currentTargetDeg = fiducial.getTargetYDegrees();
        double turretPos = getMotor(MotorNames.turret).getCurrentPosition();

        double targetTicks = turretPos - (TICKS_PER_REV * currentTargetDeg) / 360.0;

        double pid = turretPID.calculate(turretPos, targetTicks);

        getMotor(MotorNames.turret).setPower(pid);

        telemetry.addData("Fiducial ID", id);
        telemetry.addData("Current Target Degrees", currentTargetDeg);
        telemetry.addData("Current Pos", turretPos);
        telemetry.addData("PID Power", pid);
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

    // Shooting methods below

    public intakedOrder getIntakeOrder (ArrayList<IntakeSpindexer_shreyas.BallColor> balls) {
        if (balls == null || balls.size() < 2) {
            return intakedOrder.GG;
        }
        IntakeSpindexer_shreyas.BallColor first  = balls.get(0);
        IntakeSpindexer_shreyas.BallColor second = balls.get(1);

        if (first == null || second == null) {
            return intakedOrder.GG;
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

    public void shootIndexed(ShootCase shootCase, int velocity, double hoodPose) {
        if (isShooting) return;
        if (intakeSpindexer == null || intakeSpindexer.ballColors.size() < 2) return;

        ArrayList<IntakeSpindexer_shreyas.BallColor> firstTwo = new ArrayList<>(
                intakeSpindexer.ballColors.subList(0, 2)
        );

        intakedOrder intakeOrder = getIntakeOrder(firstTwo);
        getMotor(MotorNames.leftShooter).setVelocity(velocity);
        getMotor(MotorNames.rightShooter).setVelocity(velocity);
        getServo(ServoNames.hood).setPosition(hoodPose);
        new Thread(() -> {
            isShooting = true;
            try {
                switch(shootCase) {
                    case PPG:
                        //region PPG cases
                        if (intakeOrder == intakedOrder.PP){
                            //Thread.sleep(1000);
                            shootCCW();
                            Thread.sleep(1000);
                            intakeSpindexer.rotateSpindexer120(1);
                            Thread.sleep(1000);
                            shootCW();
                            Thread.sleep(2000);
                            shootCW();
                            Thread.sleep(1000);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin1).setPosition(0.0);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin2).setPosition(0.0);
                            intakeSpindexer.ballColors.clear();
                        } else if (intakeOrder == intakedOrder.PG) {
                            intakeSpindexer.rotateSpindexer120(1);
                            Thread.sleep(1000);
                            shootCCW();
                            Thread.sleep(1000);
                            shootCCW();
                            Thread.sleep(2000);
                            shootCW();
                            Thread.sleep(1000);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin1).setPosition(0.0);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin2).setPosition(0.0);
                            intakeSpindexer.ballColors.clear();
                        } else if (intakeOrder == intakedOrder.GP) {
                            //Thread.sleep(200);
                            shootCCW();
                            Thread.sleep(1000);
                            shootCCW();
                            Thread.sleep(2000);
                            shootCW();
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
                            shootCCW();
                            Thread.sleep(1000);
                            shootCCW();
                            Thread.sleep(1000);
                            shootCW();
                            Thread.sleep(100);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin1).setPosition(0.0);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin2).setPosition(0.0);
                            intakeSpindexer.ballColors.clear();
                        } else if (intakeOrder == intakedOrder.PG) {
                            intakeSpindexer.rotateSpindexer120(1);
                            Thread.sleep(1000);
                            shootCW(); //shootCCW();
                            //intakeSpindexer.rotateSpindexer120(1, false);
                            Thread.sleep(1000);
                            shootCW();
                            Thread.sleep(1000);
                            shootCW();
                            Thread.sleep(100);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin1).setPosition(0.0);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin2).setPosition(0.0);
                            intakeSpindexer.ballColors.clear();
                        } else if (intakeOrder == intakedOrder.GP) {
                            Thread.sleep(1000);
                            shootCCW();
                            intakeSpindexer.rotateSpindexer120(1);
                            Thread.sleep(1000);
                            shootCW();
                            Thread.sleep(1000);
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
                            intakeSpindexer.rotateSpindexer120(1);
                            Thread.sleep(100);
                            shootCW();
                            Thread.sleep(500);
                            shootCW();
                            Thread.sleep(500);
                            shootCW();
                            Thread.sleep(100);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin1).setPosition(0.0);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin2).setPosition(0.0);
                            intakeSpindexer.ballColors.clear();
                        } else if (intakeOrder == intakedOrder.PG) {
                            Thread.sleep(1000);
                            shootCW();
                            Thread.sleep(500);
                            shootCW();
                            Thread.sleep(500);
                            shootCW();
                            Thread.sleep(100);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin1).setPosition(0.0);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin2).setPosition(0.0);
                            intakeSpindexer.ballColors.clear();
                        } else if (intakeOrder == intakedOrder.GP) {
                            intakeSpindexer.rotateSpindexer120(-1);
                            Thread.sleep(100);
                            shootCW();
                            Thread.sleep(500);
                            shootCW();
                            Thread.sleep(500);
                            shootCW();
                            Thread.sleep(100);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin1).setPosition(0.0);
                            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin2).setPosition(0.0);
                            intakeSpindexer.ballColors.clear();
                        }
                        break;
                    //endregion
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
            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.kicker).setPosition(0.35);
            Thread.sleep(200);
            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.kicker).setPosition(0.225);
            Thread.sleep(200);
            intakeSpindexer.rotateSpindexer120(1);
            intakeSpindexer.ballsLoaded--;
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void shootCW() {
        try {
            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.kicker).setPosition(0.35);
            Thread.sleep(200);
            intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.kicker).setPosition(0.225);
            Thread.sleep(200);
            intakeSpindexer.rotateSpindexer120(-1);
            intakeSpindexer.ballsLoaded--;
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void simpleShootSequence(int velocity, double hoodPose, int firstWait, int secondThirdWait,
                                    IntakeSpindexer_shreyas intakeSpindexer) {
        new Thread(() -> {
            try {
                getMotor(TurretShooter_shreyas.MotorNames.leftShooter).setVelocity(velocity);
                getMotor(TurretShooter_shreyas.MotorNames.rightShooter).setVelocity(velocity);
                getServo(TurretShooter_shreyas.ServoNames.hood).setPosition(hoodPose);
                Thread.sleep(firstWait);
                shootCCW();
                getMotor(MotorNames.leftShooter).setVelocity(velocity + 200);
                getMotor(MotorNames.rightShooter).setVelocity(velocity + 200);
                getServo(ServoNames.hood).setPosition(hoodPose - 0.10);

                Thread.sleep(secondThirdWait);
                shootCCW();
                getServo(ServoNames.hood).setPosition(hoodPose - 0.1);
                getMotor(MotorNames.leftShooter).setVelocity(velocity + 150);
                getMotor(MotorNames.rightShooter).setVelocity(velocity + 150);

                Thread.sleep(secondThirdWait);
                shootCCW();
                intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin1).setPosition(0);
                intakeSpindexer.getServo(IntakeSpindexer_shreyas.ServoNames.spin2).setPosition(0);
                getServo(ServoNames.hood).setPosition(hoodPose);
                Thread.sleep(200);
            } catch (Exception e) {
                throw new RuntimeException(e);
            } finally {
                intakeSpindexer.ballColors.clear();
            }
        }).start();
    }
    //endregion
}