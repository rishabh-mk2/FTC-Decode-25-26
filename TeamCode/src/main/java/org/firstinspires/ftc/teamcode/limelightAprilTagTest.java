package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

@TeleOp(name = "Limelight AprilTag Test", group = "TeleOp")
public class limelightAprilTagTest extends LinearOpMode {

    public Limelight3A limelight;

    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        DcMotor motor = this.hardwareMap.get(DcMotor.class, "motor");

        limelight.start();

        telemetry.addData(">", "Robot ready!!!.  Press Play!!!!!");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();

            /*telemetry.addData("LL Latency", captureLatency + targetingLatency);
            telemetry.addData("Parse Latency", parseLatency);
            telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
             */
            telemetry.addData("tx", result.getTx());
            telemetry.addData("ty", result.getTy());
            //telemetry.addData("txnc", result.getTxNC());
            //telemetry.addData("tync", result.getTyNC());

            // access april tag results
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId(); // The ID number of the fiducial
                double x = fiducial.getTargetXDegrees(); // Where it is (left-right)
                double y = fiducial.getTargetYDegrees(); // Where it is (up-down)
                String side;
                if (x < 0) {side = "left";} else {side = "right";}
                telemetry.addData("Fiducial " + id, " is " + x + "degrees " + side);
                if (x < -1){
                    motor.setPower(0.1);
                } else if (x > 1) {
                    motor.setPower(-0.1);
                } else if ((-1 <= x && x <=1)) {
                    motor.setPower(0);
                }
            }
            telemetry.update();
        }
        limelight.stop();
    }
}
