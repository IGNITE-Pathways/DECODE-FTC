package org.firstinspires.ftc.teamcode.auto;

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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()

            .mass(17)
            .forwardZeroPowerAcceleration(-41.92219776577201)
            .lateralZeroPowerAcceleration(-72.95656401089053)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.055, 0, 0.0001, 0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(2.5, 0, 0.06, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.08,0,0.001,0.6,0.02))
            .centripetalScaling(0.0005);

// breaking start is 1.0
// breaking strength is 1.2


    // forward velocity 69.1272885780635

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.2, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(69.1272885780635)
            .yVelocity(49.48530013917939)
            .rightFrontMotorName("rightfrontmotor")
            .rightRearMotorName("rightbackmotor")
            .leftRearMotorName("leftbackmotor")
            .leftFrontMotorName("leftfrontmotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.75)
            .strafePodX(-7.08661)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("imu")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
}