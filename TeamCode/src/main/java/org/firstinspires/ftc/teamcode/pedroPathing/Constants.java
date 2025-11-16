package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            //TODO: update robot mass (in kg) (used to compensate centripetal force)
            .mass(5);

    // TODO: Add the PID tuners after decided which PID tuner we will use for the robot
    public static MecanumConstants driveConstants = new MecanumConstants()
            //TODO: change the direction of motors as needed
            .maxPower(1)
            .rightFrontMotorName("rF")
            .rightRearMotorName("rR")
            .leftRearMotorName("lR")
            .leftFrontMotorName("lF")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
            /*
            * TODO: for automatic tuners, follow the steps on Pedropathing tutorial
            * TODO: .xVelocity(velocity), .yVelocity(velocity) in MecanumConstants
            * TODO: .forwardZeroPowerAcceleration(acceleration), .lateralZeroPowerAcceleration(acceleration) in FollowerConstants
            * */

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(5.5122)
            .strafePodX(-7.435)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD) //TODO: find out whether we changed encoder resolution (if so, replace line with .customEncoderResolution)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    // TODO: run LocalizationTest under Tuning.java --> move robot forward (if x is not increasing, forwardEncoderDirection should be reverse)
    // TODO: move robot left (if y is not increasing, strafeEncoderDirection should be reversed)

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
