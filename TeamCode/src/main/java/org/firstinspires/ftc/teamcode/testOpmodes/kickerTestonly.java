package org.firstinspires.ftc.teamcode.testOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Kick+shoot", group = "TeleOp")
//@Disabled
public class kickerTestonly extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    boolean AisPressed = false;
    boolean XisPressed = false;
    boolean BisPressed = false;
    boolean YisPressed= false;

    boolean kickerGoingUp = false;
    boolean kickerGoingDown = false;
    @Override
    public void runOpMode() throws InterruptedException {

        Servo spin1 = this.hardwareMap.get(Servo.class, "spin1");
        Servo spin2 = this.hardwareMap.get(Servo.class, "spin2");
        Servo hood = this.hardwareMap.get(Servo.class, "hood");
        DcMotorEx intake = this.hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx turret = this.hardwareMap.get(DcMotorEx.class,"turret");
        Servo kicker = this.hardwareMap.get(Servo.class, "kicker");
        Servo kicker2 = this.hardwareMap.get(Servo.class, "kicker2");
        DcMotorEx outtake1 = this.hardwareMap.get(DcMotorEx.class, "leftShooter");
        DcMotorEx outtake2 = this.hardwareMap.get(DcMotorEx.class, "rightShooter");

        outtake1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtake1.setDirection(DcMotorEx.Direction.FORWARD);
        outtake2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtake2.setDirection(DcMotorEx.Direction.REVERSE);

        outtake1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        outtake2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        kicker.setPosition(0.225);
        hood.setPosition(0);
        kicker2.setPosition(0.225);
        turret.setPower(0);
       // spin1.setPosition(0.0);
        //spin2.setPosition(0.0);

        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {
            turret.setPower(0);
            if(gamepad1.right_bumper) {
                outtake1.setVelocity(-2100);
                outtake2.setVelocity(-2100);
            } else {
                outtake1.setVelocity(0);
                outtake2.setVelocity(0);
            }

           if(gamepad1.a && !AisPressed){
                AisPressed = true;
            }
            if (!gamepad1.a && AisPressed) {
                hood.setPosition(hood.getPosition() + 0.05);
                AisPressed = false;
            }

            if(gamepad1.y && !YisPressed){
                YisPressed = true;
            }
            if (!gamepad1.y && YisPressed) {
                hood.setPosition(hood.getPosition() - 0.05);
                YisPressed = false;
            }

            if(gamepad1.b && !BisPressed) {
                BisPressed = true;
            }
            if(!gamepad1.b && BisPressed) {
                kicker.setPosition(0.39);
                kicker2.setPosition(0.39);
                BisPressed = false;
                time = runtime.milliseconds();
                kickerGoingUp = false;
            }
            if(gamepad1.x && !XisPressed) {
                XisPressed = true;
            }
            if(!gamepad1.x && XisPressed) {
                kicker.setPosition(0.225);
                kicker2.setPosition(0.225);
                XisPressed = false;
            }

            telemetry.addData("hood position:",hood.getPosition());
            telemetry.update();
        }
    }

}