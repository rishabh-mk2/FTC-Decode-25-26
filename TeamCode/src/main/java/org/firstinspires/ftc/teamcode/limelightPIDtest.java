package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.controller.PIDController;

import java.util.List;

@TeleOp(name = "LL PID Test", group = "TeleOp")
public class limelightPIDtest extends LinearOpMode {
    public Limelight3A limelight;

    PIDController pidController;
    double p, i, d;

    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        DcMotor turret = this.hardwareMap.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        telemetry.setAutoClear(true);

        limelight.start();

        telemetry.addData(">", "Robot ready!!!.  Press Play!!!!!");
        boolean AisPressed = false;
        boolean BisPressed = false;
        boolean XisPressed = false;
        boolean YisPressed = false;
        boolean dPadUpIsPressed = false;
        boolean dPadDownIsPressed = false;
        waitForStart();


        p = 0.009;
        i = 0;
        d = 0.0015;
        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            // KNOWN WORKING VALUES: p = 0.0091;  i = 0;  d = 0.0015

            pidController = new PIDController(p, i, d);
            pidController.setPID(p, i, d);
            if(gamepad1.a && !AisPressed){
                AisPressed = true;
            }
            if (!gamepad1.a && AisPressed) {
                p += 0.0001;
                AisPressed = false;
            } if(gamepad1.b && !BisPressed){
                BisPressed = true;
            } if(!gamepad1.b && BisPressed){
                p -= 0.0001;
                BisPressed = false;
            }



            if(gamepad1.x && !XisPressed){
                XisPressed = true;
            }
            if (!gamepad1.x && XisPressed) {
                d += 0.0001;
                XisPressed = false;
            } if(gamepad1.y && !YisPressed){
                YisPressed = true;
            } if(!gamepad1.y && YisPressed){
                d -= 0.0001;
                YisPressed = false;
            }
            telemetry.addData("kP val:", p);
            telemetry.addData("kI val:", i);
            telemetry.addData("kD val:", d);


            LLResult result = limelight.getLatestResult();
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            // RANDOM DATA
            /*
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();

            telemetry.addData("LL Latency", captureLatency + targetingLatency);
            telemetry.addData("Parse Latency", parseLatency);
            telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
             */

            // APRIL TAG TRACKING

            if (result.isValid()){
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    int id = fiducial.getFiducialId();
                    double currentTargetDeg = fiducial.getTargetXDegrees();
                    double turretPos = turret.getCurrentPosition();
                    double pid = pidController.calculate(turretPos, turretPos + 537.7*currentTargetDeg/360.0); // NOTE FOR SELF: Adding/Subtracting the TICK VAL. of the DEG VAL. from the current motor pos. basically doing -> currentMotorTickVal +- (motorTickPerRevolution*givenDegVal/totalDegInCircle)
                    turret.setPower(pid);
                    telemetry.addData("Fiducial " + id, " is " + currentTargetDeg + " degrees");
                }
            } else {
                telemetry.addData("info", "no april tag being detected");
                turret.setPower(0);
            }


            telemetry.update();
        }
        limelight.stop();
    }
}

// EXAMPLE PID WITH MOTOR
/*
    public void moveOuttakeSlides(int target, int offset, boolean down) {
        int slidesPos = getOuttakeSlidesPosition();
        double pid = pidController.calculate(slidesPos, -target + offset);
        double slidesPower = pid + f;

        if(down) {
            getDcMotorEx(IntakeOuttake.MotorNames.leftOuttake).setPower(-slidesPower * 0.1);
            getDcMotorEx(IntakeOuttake.MotorNames.rightOuttake).setPower(slidesPower * 0.1);
        } else {
            getDcMotorEx(IntakeOuttake.MotorNames.leftOuttake).setPower(-slidesPower);
            getDcMotorEx(IntakeOuttake.MotorNames.rightOuttake).setPower(slidesPower);
        }
    }
 */