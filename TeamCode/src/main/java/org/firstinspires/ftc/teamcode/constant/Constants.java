package org.firstinspires.ftc.teamcode.constant;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;

import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.swerve.SwerveDrive;


public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14)
            .forwardZeroPowerAcceleration(-42)
            .lateralZeroPowerAcceleration(-42)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.02,
                    0,
                    0,
                    0.02
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    1.0,
                    0,
                    0,
                    0
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.008,
                    0,
                    0,
                    0.6,
                    0
            ))
            .centripetalScaling(0.0003);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(6.2)
            .strafePodX(0.01)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    /**
     These are the PathConstraints in order:
     tValueConstraint, velocityConstraint, translationalConstraint, headingConstraint, timeoutConstraint,
     brakingStrength, BEZIER_CURVE_SEARCH_LIMIT, brakingStart

     The BEZIER_CURVE_SEARCH_LIMIT should typically be left at 10 and shouldn't be changed.
     */


    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.009,
            50,
            0.9,
            10,
            1.1
    );

}