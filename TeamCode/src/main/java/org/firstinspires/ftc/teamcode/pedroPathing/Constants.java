package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.Common;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12)
            .forwardZeroPowerAcceleration(-30.08)
            .lateralZeroPowerAcceleration(-71.16)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.003, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.05, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.015, 0, 0.00003, 0.6, 0));
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(81.14)
            .yVelocity(64.14);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(Common.PINPOINT_X_OFFSET_MM/25.4)
            .strafePodX(Common.PINPOINT_Y_OFFSET_MM/25.4)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(Common.PINPOINT_POD_TYPE)
            .forwardEncoderDirection(Common.PINPOINT_X_DIRECTION)
            .strafeEncoderDirection(Common.PINPOINT_Y_DIRECTION);

    public static PathConstraints pathConstraints = new PathConstraints(0.8, 0, 0.5, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

}