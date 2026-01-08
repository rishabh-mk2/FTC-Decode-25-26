package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.43053)
//            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.025, 0.025))
            .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0, 0.025, 0.02))
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.15, 0, 0.01, 0.023))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.05, 0, 0.0035, 0.023))
            .forwardZeroPowerAcceleration(-56.65633997326078)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.008,0,0.0001,0.6,0.007))
            .lateralZeroPowerAcceleration(-81.37651332233594)
            .centripetalScaling(0.0005);
    /*
     * TODO: for automatic tuners, follow the steps on Pedropathing tutorial
     * TODO: .forwardZeroPowerAcceleration(acceleration), .lateralZeroPowerAcceleration(acceleration) in FollowerConstants
     * */


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rF")
            .rightRearMotorName("rR")
            .leftRearMotorName("lR")
            .leftFrontMotorName("lF")
            .leftFrontMotorDirection(DcMotorEx.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorEx.Direction.FORWARD)
//            .xVelocity(72.85834028589444)
//            .yVelocity(58.96647583968998);
            .xVelocity(72.01752814345473)
            .yVelocity(55.84054973932702);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(5.52244094)
            .strafePodX(-7.14988189)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
