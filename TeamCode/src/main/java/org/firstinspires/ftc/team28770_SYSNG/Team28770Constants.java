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
    public static final double WHEEL_RADIUS_MM = 50.0; // TODO: measure actual
    public static final double WHEEL_BASE_MM   = 237.42; // front-back distance, TODO: measure
    public static final double TRACK_WIDTH_MM  = 310.0; // left-right distance, TODO: measure

    // Encoder/motor config
    public static final double TICKS_PER_REV = 537.7; // TODO: confirm motor
    public static final double GEAR_RATIO    = 1.0;       // wheel rev / motor rev, TODO: confirm

    // Derived: millimeters per wheel revolution
    public static final double WHEEL_CIRCUMFERENCE_MM = 2.0 * Math.PI * WHEEL_RADIUS_MM;

    // Max wheel speed estimates (for normalization / motion constraints)
    // You can refine these later with tests.
    public static final double MAX_WHEEL_SPEED_MM_PER_S = 1500.0; // TODO: tune
    public static final double MAX_WHEEL_RPS = MAX_WHEEL_SPEED_MM_PER_S / WHEEL_CIRCUMFERENCE_MM;

    // TeleOp input shaping configuration (stick space -> mm/s, rad/s)
    public static final double TELEOP_DEADBAND = 0.05;               // TODO: tune
    public static final double TELEOP_EXPO_TRANSLATION = 0.25;       // TODO: tune
    public static final double TELEOP_EXPO_ROTATION = 0.3;           // TODO: tune
    public static final double TELEOP_MAX_VEL_MM_PER_S = 1200.0;     // TODO: tune
    public static final double TELEOP_MAX_ANG_VEL_RAD_PER_S = 4.0;   // TODO: tune
    public static final double TELEOP_LINEAR_SLEW_RATE = 3000.0;     // mm/s^2, TODO: tune (symmetric legacy)
    public static final double TELEOP_ANGULAR_SLEW_RATE = 8.0;       // rad/s^2, TODO: tune (symmetric legacy)
    public static final double TELEOP_LINEAR_ACCEL_SLEW = 3000.0;    // mm/s^2, TODO: tune (increasing command)
    public static final double TELEOP_LINEAR_DECEL_SLEW = 3000.0;    // mm/s^2, TODO: tune (decreasing command)
    public static final double TELEOP_ANG_ACCEL_SLEW = 8.0;          // rad/s^2, TODO: tune (increasing command)
    public static final double TELEOP_ANG_DECEL_SLEW = 8.0;          // rad/s^2, TODO: tune (decreasing command)

    // Heading regulation (TeleOp hold) gains
    public static final double HEADING_KP = 4.0;    // TODO: tune
    public static final double HEADING_KI = 0.0;    // TODO: tune
    public static final double HEADING_KD = 0.4;    // TODO: tune
    public static final double HEADING_KS = 0.15;    // TODO: tune
    public static final double HEADING_ERR_DEADBAND_RAD = Math.toRadians(0.5); // TODO: tune

    // Autonomous motion limits
    public static final double AUTO_MAX_VEL_MM_PER_S = 1000.0;       // TODO: tune
    public static final double AUTO_MAX_ACCEL_MM_PER_S2 = 1200.0;    // TODO: tune
    public static final double AUTO_MAX_ANG_VEL_RAD_PER_S = 4.0;     // TODO: tune
    public static final double AUTO_MAX_ANG_ACCEL_RAD_PER_S2 = 6.0;  // TODO: tune

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
