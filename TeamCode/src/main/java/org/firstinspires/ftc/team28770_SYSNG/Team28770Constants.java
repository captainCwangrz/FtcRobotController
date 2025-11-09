package org.firstinspires.ftc.team28770_SYSNG;

import org.firstinspires.ftc.common.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Team 28770 specific drivetrain constants.
 *
 * These are the ONLY place you touch robot-specific
 * wheel radius, gear ratio, track width, etc. All linear quantities are in mm,
 * velocities in mm/s, and angles in radians.
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

    // Linear dimensions in millimeters
    public static final double WHEEL_RADIUS_MM = 0.0; // TODO: measure
    public static final double WHEEL_BASE_MM   = 0.0; // front-back distance, TODO: measure
    public static final double TRACK_WIDTH_MM  = 0.0; // left-right distance, TODO: measure

    // Encoder/motor config
    public static final double TICKS_PER_REV = 0.0;
    public static final double GEAR_RATIO    = 0.0;       // wheel rev / motor rev

    // Derived: millimeters per wheel revolution
    public static final double WHEEL_CIRCUMFERENCE_MM = 2.0 * Math.PI * WHEEL_RADIUS_MM;

    // Max wheel speed estimates (for normalization / motion constraints)
    // You can refine these later with tests.
    public static final double MAX_WHEEL_RPS = 0.0; // TODO: measure
    public static final double MAX_WHEEL_MM_PER_SEC = MAX_WHEEL_RPS * WHEEL_CIRCUMFERENCE_MM;

    // Pinpoint config
    public static final String PINPOINT_NAME = "pinpoint";
    public static final GoBildaPinpointDriver.GoBildaOdometryPods PINPOINT_POD_TYPE = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static final double PINPOINT_X_OFFSET = 0.0; // mm
    public static final double PINPOINT_Y_OFFSET = 0.0; // mm
    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_X_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_Y_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    public static final DistanceUnit PINPOINT_DIST_UNIT = DistanceUnit.MM;
}
