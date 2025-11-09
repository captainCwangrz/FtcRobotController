package org.firstinspires.ftc.team28770_SYSNG;

import org.firstinspires.ftc.common.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Team 28770 specific drivetrain constants.
 *
 * These are the ONLY place you touch robot-specific
 * wheel radius, gear ratio, track width, etc.
 *
 */

// Todo: MAKE SURE THESE ARE GROUND TRUTHS
public class Team28770Constants
{
    private Team28770Constants() {}
    // Drive motor names
    public static final String MOTOR_FL = "fl";
    public static final String MOTOR_FR = "fr";
    public static final String MOTOR_BL = "bl";
    public static final String MOTOR_BR = "br";

    // Linear dimensions in meters
    public static final double WHEEL_RADIUS = 0.0;
    public static final double WHEEL_BASE   = 0.0; // front-back distance
    public static final double TRACK_WIDTH  = 0.0; // left-right distance

    // Encoder/motor config
    public static final double TICKS_PER_REV = 0.0;
    public static final double GEAR_RATIO    = 0.0;       // wheel rev / motor rev

    // Derived: meters per wheel revolution
    public static final double WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * WHEEL_RADIUS;

    // Max wheel speed estimates (for normalization / motion constraints)
    // You can refine these later with tests.
    public static final double MAX_WHEEL_RPS = 0.0; // example
    public static final double MAX_WHEEL_MPS = MAX_WHEEL_RPS * WHEEL_CIRCUMFERENCE;

    // Pinpoint config
    public static final String PINPOINT_NAME = "pinpoint";
    public static final GoBildaPinpointDriver.GoBildaOdometryPods PINPOINT_POD_TYPE = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static final double PINPOINT_X_OFFSET = 0.0; // mm
    public static final double PINPOINT_Y_OFFSET = 0.0; // mm
    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_X_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_Y_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    public static final DistanceUnit PINPOINT_DIST_UNIT = DistanceUnit.METER;
}
