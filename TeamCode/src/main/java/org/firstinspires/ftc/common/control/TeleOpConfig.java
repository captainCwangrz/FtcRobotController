package org.firstinspires.ftc.common.control;

/**
 * Configuration bundle for TeleOp input shaping.
 *
 * <p>All units obey the control stack ground rules:</p>
 * <ul>
 *     <li>{@code deadband}: unitless fraction of stick travel.</li>
 *     <li>{@code expoTranslation}: unitless, 0 = linear, 1 = strong curve.</li>
 *     <li>{@code expoRotation}: unitless, 0 = linear, 1 = strong curve.</li>
 *     <li>{@code maxVelMmPerS}: maximum linear velocity command, mm/s.</li>
 *     <li>{@code maxAngVelRadPerS}: maximum angular velocity command, rad/s.</li>
 * </ul>
 */
public record TeleOpConfig(
        double deadband,
        double expoTranslation,
        double expoRotation,
        double maxVelMmPerS,
        double maxAngVelRadPerS
) {}
