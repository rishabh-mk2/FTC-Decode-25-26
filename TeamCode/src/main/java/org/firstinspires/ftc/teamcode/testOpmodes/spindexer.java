package org.firstinspires.ftc.teamcode.testOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Spindexer Proper Test", group = "TeleOp")
//@Disabled
public class spindexer extends LinearOpMode {
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

        Servo spin1 = this.hardwareMap.get(Servo.class, "spin1");
        Servo spin2 = this.hardwareMap.get(Servo.class, "spin2");
        DcMotorEx intake = this.hardwareMap.get(DcMotorEx.class, "intakeMotor");

        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {
            intake.setPower(0.5); // keep running intake

            spin1.setPosition(0.5);
            spin2.setPosition(0.5);

        }
    }

}
