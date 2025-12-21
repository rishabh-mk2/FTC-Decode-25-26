package org.firstinspires.ftc.teamcode.testOpmodes;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.List;

@TeleOp(name = "LL PID Test", group = "TeleOp")
public class limelightPIDtest extends LinearOpMode {
    private Limelight3A limelight;
    private PIDController pidController;
    private DcMotorEx turret;
    private PinpointLocalizer pinpointLocalizer;
    private IMU imu;
    public LLResult result;
    double p, i, d;
    public int id;


    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        turret = this.hardwareMap.get(DcMotorEx.class, "turret");

        limelight.pipelineSwitch(0);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                RevHubOrientationOnRobot.UsbFacingDirection.UP);
//        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        telemetry.setMsTransmissionInterval(11);
        telemetry.setAutoClear(true);

        limelight.start();

        telemetry.addData(">", "Robot ready!!!.  Press Play!!!!!");
        boolean AisPressed = false;
        boolean BisPressed = false;
        boolean XisPressed = false;
        boolean YisPressed = false;
        boolean dPadUpIsPressed = false;
        boolean dPadDownIsPressed = false;

        p = 0.02;
        i = 0.0;
        d = 0.5;
        pidController = new PIDController(p, i, d);
        pidController.setPID(p, i, d);

        waitForStart();


        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            // KNOWN WORKING VALUES: p = 0.0091;  i = 0;  d = 0.0015


            /* if(gamepad1.a && !AisPressed){
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
            } */

            //TODO: Add the distance to the april tag stuff
//            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

            result = limelight.getLatestResult();
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            // APRIL TAG TRACKING
            if (result.isValid()){
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
//                    Pose3D botpose = result.getBotpose_MT2();
                    id = fiducial.getFiducialId();
                    double currentTargetDeg = fiducial.getTargetYDegrees();
                    double turretPos = turret.getCurrentPosition();
                    double pid = pidController.calculate(turretPos, turretPos - (537.7*currentTargetDeg)/360.0); // NOTE FOR SELF: Adding/Subtracting the TICK VAL. of the DEG VAL. from the current motor pos. basically doing -> currentMotorTickVal +- (motorTickPerRevolution*givenDegVal/totalDegInCircle)
                    turret.setPower(pid);
                    telemetry.addData("Fiducial " + id, " is " + currentTargetDeg + " degrees");
                    telemetry.addData("Target X", result.getTx());
                    telemetry.addData("Target Area", result.getTa());
                    telemetry.addData("Current Pos", turretPos);
                    telemetry.addData("PID Power", pid);
                    telemetry.addData("Current Target Degrees", currentTargetDeg);
//                    telemetry.addData("Botpose", botpose.toString());
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