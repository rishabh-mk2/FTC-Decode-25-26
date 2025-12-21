package org.firstinspires.ftc.teamcode.testOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    DcMotorEx leftShooter;
    DcMotorEx rightShooter;
    @Override
    public void runOpMode() throws InterruptedException {

        Servo spin1 = this.hardwareMap.get(Servo.class, "spin1");
        Servo spin2 = this.hardwareMap.get(Servo.class, "spin2");
        DcMotorEx intake = this.hardwareMap.get(DcMotorEx.class, "intake");
        leftShooter = this.hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = this.hardwareMap.get(DcMotorEx.class, "rightShooter");
        Servo kicker = this.hardwareMap.get(Servo.class, "kicker");

        leftShooter.setDirection(DcMotorEx.Direction.FORWARD);
        rightShooter.setDirection(DcMotorEx.Direction.REVERSE);
        leftShooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightShooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {
            intake.setPower(0.7); // keep running intake

            if(gamepad1.a) {
                spin1.setPosition(0.06);
                spin2.setPosition(0.06);
            }

            if (gamepad1.b && !BisPressed) {
                BisPressed = true;
            }

            if (!gamepad1.b && BisPressed) {
                spin1.setPosition(spin1.getPosition() + 0.169);
                spin2.setPosition(spin2.getPosition() + 0.169);
                BisPressed = false;
            }



        }
    }

}
