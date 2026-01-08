package org.firstinspires.ftc.teamcode.Robot.Decode.TeleOp;

import static org.firstinspires.ftc.teamcode.Robot.Decode.Robot.IntakeSpindexer.ballsLoaded;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.Decode.Alliance;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.IntakeSpindexer;
import org.firstinspires.ftc.teamcode.Robot.Decode.Robot.TurretShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "TeleOpFinal", group = "TeleOp")
@Disabled
public class Teleop extends OpMode {

    Follower follower;
    TurretShooter turretShooter;
    IntakeSpindexer intakeSpindexer;

    boolean shootPressed = false;
    private final ElapsedTime runtime = new ElapsedTime();
    int ballsToShoot = IntakeSpindexer.ballsLoaded;
    double shootStartTime = 0;
    boolean intake=false;

    boolean XisPressed = false;
    boolean YisPressed= false;
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Constants.createFollower(hardwareMap).getPose());
        runtime.reset();

        turretShooter = new TurretShooter(this, Alliance.RED); // change alliance if needed
        intakeSpindexer = new IntakeSpindexer(this);
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        // --- DRIVE ---
        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x*0.4,
                true
        );

        // --- SUBSYSTEM UPDATES ---
        turretShooter.updateTurretTracking();
        intakeSpindexer.update();

        if(gamepad1.x && !XisPressed) {
            XisPressed = true;
            intake = true;
        }
        if(!gamepad1.x && XisPressed && intake) {
            intakeSpindexer.setIntakeState(IntakeSpindexer.IntakeState.INTAKING);
            if (intakeSpindexer.getColorSensor1().getDistance(DistanceUnit.MM) < 20 || intakeSpindexer.getColorSensor2().getDistance(DistanceUnit.MM) < 20) {
                    intakeSpindexer.rotateSpindexer120();
                    ballsToShoot+=1;
                    intake = false;
            }
            XisPressed = false;
        }

        if(gamepad1.y && !YisPressed) {
            YisPressed = true;
            shootStartTime = runtime.milliseconds();
        }
        if(!gamepad1.y && YisPressed) {
            double elapsed = runtime.milliseconds() - shootStartTime;
            if(intakeSpindexer.getServo(IntakeSpindexer.ServoNames.spin1).getPosition() < 0.65){
                intakeSpindexer.rotateSpindexer60();
            }
            turretShooter.shoot(1700, 0.95);
            switch(ballsLoaded) {
                case 3:
                    if (elapsed > 800) {
                        intakeSpindexer.getServo(IntakeSpindexer.ServoNames.kicker).setPosition(0.4);
                    }
                    if (elapsed > 1200) {
                        turretShooter.stopShooter();
                        intakeSpindexer.getServo(IntakeSpindexer.ServoNames.kicker).setPosition(0.225);
                        intakeSpindexer.consumeBall();
                    }
            }
        }
        //telemetry.addData("distance1",intakeSpindexer.getColorSensor1().getDistance(DistanceUnit.MM));
        //telemetry.addData("distance2",intakeSpindexer.getColorSensor2().getDistance(DistanceUnit.MM));
        //telemetry.addData("position",intakeSpindexer.getServo(IntakeSpindexer.ServoNames.spin2).getPosition());
        telemetry.addData("balls to shoot",ballsLoaded);
        telemetry.update();

    }

    @Override
    public void stop() {
        turretShooter.stop();
    }
}
