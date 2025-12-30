package org.firstinspires.ftc.teamcode.testOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Kick+shoot", group = "TeleOp")
//@Disabled
public class kickerTestonly extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    static final double SERVO_POW = 0.7;
    static final double DEADZONE = 5;
    double lastAngle;
    boolean AisPressed = false;
    boolean XisPressed = false;
    boolean BisPressed = false;
    boolean YisPressed= false;
    boolean dpadupisPressed= false;
    boolean dpadleftisPressed= false;
    boolean dpadrightisPressed= false;
    boolean dpaddownisPressed= false;

    @Override
    public void runOpMode() throws InterruptedException {

        Servo spin1 = this.hardwareMap.get(Servo.class, "spin1");
        Servo spin2 = this.hardwareMap.get(Servo.class, "spin2");
        Servo hood = this.hardwareMap.get(Servo.class, "hood");
        DcMotorEx intake = this.hardwareMap.get(DcMotorEx.class, "intakeMotor");
        AnalogInput analog = this.hardwareMap.get(AnalogInput.class, "analog");
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
        hood.setPosition(0.7995);
        kicker2.setPosition(0.225);
        turret.setPower(0);
/*        double currV = analog.getVoltage();
        double move = 1.54-currV;
        if (move== Math.abs(move)) moveClockwise(spin1,spin2,analog,move*720/3.3);
        if (move != Math.abs(move)) moveClockwise(spin1,spin2,analog,move*720/3.3);
        waitForStart();*/
        spin1.setPosition(0.08);
        spin2.setPosition(0.08);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            turret.setPower(0);
            if(gamepad1.right_bumper) {
                outtake1.setVelocity(-1900);
                outtake2.setVelocity(-1900);
            } else {
                outtake1.setVelocity(0);
                outtake2.setVelocity(0);
            }
            if(gamepad1.left_bumper) {
                intake.setPower(0.7);
            } else {
                intake.setPower(0);
            }
           if(gamepad1.dpad_up && !dpadupisPressed){
                dpadupisPressed = true;
            }
            if (!gamepad1.dpad_up && dpadupisPressed) {
                hood.setPosition(hood.getPosition() + 0.05);
                dpadupisPressed = false;
            }
            if(gamepad1.dpad_down && !dpaddownisPressed){
                dpaddownisPressed = true;
            }
            if (!gamepad1.dpad_down && dpaddownisPressed) {
                hood.setPosition(hood.getPosition() - 0.05);
                dpaddownisPressed = false;
            }

            if(gamepad1.a && !AisPressed){
                AisPressed = true;
            }
            if (!gamepad1.a && AisPressed) {
                spin1.setPosition(spin1.getPosition() + 0.194);
                spin2.setPosition(spin2.getPosition() + 0.194);
                AisPressed = false;
            }

            if(gamepad1.dpad_left && !dpadleftisPressed){
                dpadleftisPressed = true;
            }
            if (!gamepad1.dpad_left && dpadleftisPressed) {
                spin1.setPosition(spin1.getPosition() - 0.194);
                spin2.setPosition(spin2.getPosition() - 0.194);
                dpadleftisPressed = false;
            }

            if(gamepad1.dpad_right && !dpadrightisPressed){
                dpadrightisPressed = true;
            }
            if (!gamepad1.dpad_right && dpadrightisPressed) {
                spin1.setPosition(spin1.getPosition() - 0.096);
                spin2.setPosition(spin2.getPosition() - 0.096);
                dpadrightisPressed = false;
            }
            if(gamepad1.b && !BisPressed){
                BisPressed = true;
            }
            if (!gamepad1.b && BisPressed) {
                spin1.setPosition(spin1.getPosition() + 0.096);
                spin2.setPosition(spin2.getPosition() + 0.096);
                BisPressed = false;
            }

            if(gamepad1.x && !XisPressed) {
                XisPressed = true;
            }
            if(!gamepad1.x && XisPressed) {
                kicker.setPosition(0.39);
                kicker2.setPosition(0.39);
                sleep(1000);
                kicker.setPosition(0.225);
                kicker2.setPosition(0.225);
                XisPressed = false;
            }
            if(gamepad1.y && !YisPressed) {
                YisPressed = true;
            }
            if(!gamepad1.y && YisPressed) {
                kicker.setPosition(0.225);
                kicker2.setPosition(0.225);
                YisPressed = false;
            }
            lastAngle = analog.getVoltage() * (360.0/3.3);

            telemetry.addData("hood position:",hood.getPosition());
            telemetry.addData("analog:",analog.getVoltage());

            telemetry.update();
        }
    }
    void moveClockwise(CRServo spin1, CRServo spin2, AnalogInput analog, double MOVE_DEG) {
        double movedAngle = 0;

        while (movedAngle < MOVE_DEG - DEADZONE) {
            double currentAngle = analog.getVoltage() * (720.0/3.3);
            double delta = currentAngle - lastAngle; // how much servo has moved

            if (delta < -180) delta += 360; // handling the wrap around (EX. CURRENT is 20 & LAST is 350 --> didn't acc move -330(CCW) but +30(CW))
            if (delta > 180) delta -= 360; // idk but chatgpt says i should have this

            if (delta > 0) movedAngle += delta; // changing how much servo moved

            spin1.setPower(SERVO_POW);
            spin2.setPower(SERVO_POW);

            lastAngle = currentAngle; // resetting how much the servo moved since last iteration for NEXT interation
        }

        spin1.setPower(0);
        spin2.setPower(0);
    }
    void moveCounterClockwise(CRServo spin1, CRServo spin2, AnalogInput analog, double MOVE_DEG) {
        double movedAngle = 0;

        while (movedAngle < MOVE_DEG - DEADZONE) {
            double currentAngle = analog.getVoltage() * (720.0/3.3);
            double delta = currentAngle - lastAngle;

            if (delta < -180) delta += 360; // idk but chatgpt says i should have this
            if (delta > 180) delta -= 360; // handling the wrap around (EX. CURRENT is 350 & LAST is 20 --> didn't acc move 330(CW) but only -30(CCW))

            if (delta < 0) movedAngle -= delta; // changing how much servo moved

            spin1.setPower(-SERVO_POW);
            spin2.setPower(-SERVO_POW);
            lastAngle = currentAngle; // resetting how much the servo moved since last iteration for NEXT interation
        }

        spin1.setPower(0);
        spin2.setPower(0);
    }

}