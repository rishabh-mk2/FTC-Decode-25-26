package org.firstinspires.ftc.teamcode.testOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Spindexer Test", group = "TeleOp")
public class spindexerTest extends LinearOpMode {
    static final double SERVO_POW = 0.35;
    static final double MOVE_DEG = 120;
    static final double DEADZONE = 2; // delta from target we can accept
    double lastAngle;
    boolean AisPressed = false;
    boolean BisPressed = false;

    @Override
    public void runOpMode() {

        CRServo spin1 = hardwareMap.get(CRServo.class, "spinServo1");
        CRServo spin2 = hardwareMap.get(CRServo.class, "spinServo2");
        AnalogInput analog = hardwareMap.get(AnalogInput.class, "analog");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        waitForStart();

        lastAngle = analog.getVoltage() * 109.09; // degree before starting (applying anything)

        while (opModeIsActive()) {

            intake.setPower(0.5); // keep running intake

            if(gamepad1.a && !AisPressed){
                AisPressed = true;
            }
            if (!gamepad1.a && AisPressed) {
                moveClockwise(spin1, spin2, analog);
                AisPressed = false;
            }

            if(gamepad1.b && !BisPressed){
                BisPressed = true;
            }
            if(!gamepad1.b && BisPressed){
                moveCounterClockwise(spin1, spin2, analog);
                BisPressed = false;
            }

            lastAngle = analog.getVoltage() * 109.09; // the angle the overall servo is at after the movements
        }
    }
    void moveClockwise(CRServo spin1, CRServo spin2, AnalogInput analog) {
        double movedAngle = 0;

        while (movedAngle < MOVE_DEG - DEADZONE && opModeIsActive()) {
            double currentAngle = analog.getVoltage() * 109.09;
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

    void moveCounterClockwise(CRServo spin1, CRServo spin2, AnalogInput analog) {
        double movedAngle = 0;

        while (movedAngle < MOVE_DEG - DEADZONE && opModeIsActive()) {
            double currentAngle = analog.getVoltage() * 109.09;
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