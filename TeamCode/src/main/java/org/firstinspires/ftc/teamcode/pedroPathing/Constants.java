package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.47789) // Mass in kilograms.
            .forwardZeroPowerAcceleration(-26.2898) //-29.16455519
            .lateralZeroPowerAcceleration(-69.454)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.06,0,0.01,0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(0.8,0,0.03,0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01,0,0.0003,0.6,0.01));

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            .85,
            1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("driveRF")
            .rightRearMotorName("driveRR")
            .leftRearMotorName("driveLR")
            .leftFrontMotorName("driveLF")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(89.26448984221211) //87.82768477792816
            .yVelocity(68.45032795583168); //69.22337521530513

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2.75)
            .strafePodX(-5.25)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static Follower follower = null;

    public static Follower createFollower(HardwareMap hardwareMap) {
        Pose startingPose = new Pose();

        if (Constants.follower != null) {
            startingPose = Constants.follower.getPose();
        }

        follower = new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();

        follower.setPose(startingPose);
        follower.update();

        return follower;
    }
}