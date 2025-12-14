package org.firstinspires.ftc.teamcode.testOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Kicker Test", group = "TeleOp")
//@Disabled
public class kickerTest extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    static final double SERVO_POW = 0.35;
    static final double DEADZONE = 20; // delta from target we can accept
    double lastAngle;
    boolean AisPressed = false;
    boolean XisPressed = false;
    boolean BisPressed = false;

    boolean kickerGoingUp = false;
    boolean kickerGoingDown = false;
    @Override
    public void runOpMode() throws InterruptedException {

        CRServo spin1 = this.hardwareMap.get(CRServo.class, "spinServo1");
        CRServo spin2 = this.hardwareMap.get(CRServo.class, "spinServo2");
        AnalogInput analog = this.hardwareMap.get(AnalogInput.class, "analog");
        DcMotorEx intake = this.hardwareMap.get(DcMotorEx.class, "intakeMotor");

        Servo kicker = this.hardwareMap.get(Servo.class, "kicker");
        DcMotorEx outtake1 = this.hardwareMap.get(DcMotorEx.class, "outtake1");
        DcMotorEx outtake2 = this.hardwareMap.get(DcMotorEx.class, "outtake2");

        outtake1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtake1.setDirection(DcMotorEx.Direction.FORWARD);
        outtake2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtake2.setDirection(DcMotorEx.Direction.REVERSE);

        outtake1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        outtake2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        kicker.setPosition(0.225);

        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {
            intake.setPower(0.5); // keep running intake

            if(gamepad1.right_bumper) {
                outtake1.setVelocity(-2250);
                outtake2.setVelocity(-2250);
            } else {
                outtake1.setVelocity(0);
                outtake2.setVelocity(0);
            }

            if(gamepad1.a && !AisPressed){
                AisPressed = true;
            }
            if (!gamepad1.a && AisPressed) {
                moveClockwise(spin1, spin2, analog, 60);
                AisPressed = false;
            }

            if(gamepad1.x && !XisPressed) {
                XisPressed = true;
            }
            if(!gamepad1.x && XisPressed) {
                moveClockwise(spin1, spin2, analog, 30);
                XisPressed = false;
            }

            if(gamepad1.b && !BisPressed) {
                BisPressed = true;
            }
            if(!gamepad1.b && BisPressed) {
                kicker.setPosition(0.4);
                BisPressed = false;
                time = runtime.milliseconds();
                kickerGoingUp = false;
            }
            if(kickerGoingUp && runtime.milliseconds() - time > 2000) {
                kicker.setPosition(0.225);
                kickerGoingUp = false;
                kickerGoingDown = true;
                time = runtime.milliseconds();
            }
            if(kickerGoingDown && runtime.milliseconds() - time > 2000) {
                kickerGoingDown = false;
                time = runtime.milliseconds();
//                moveClockwise(spin1, spin2, analog, 60);
            }

            lastAngle = analog.getVoltage() * (360.0/3.3); // the angle the overall servo is at after the movements

            telemetry.addData("voltage", analog.getVoltage());
            telemetry.update();
        }
    }
    void moveClockwise(CRServo spin1, CRServo spin2, AnalogInput analog, double MOVE_DEG) {
        double movedAngle = 0;

        while (movedAngle < MOVE_DEG - DEADZONE) {
            double currentAngle = analog.getVoltage() * (360.0/3.3);
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
            double currentAngle = analog.getVoltage() * (360.0/3.3);
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
