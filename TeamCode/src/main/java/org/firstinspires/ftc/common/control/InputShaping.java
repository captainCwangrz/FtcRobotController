package org.firstinspires.ftc.common.control;

/**
 * Utility methods for joystick shaping prior to scaling into physical units.
 */
public final class InputShaping
{
    private InputShaping() {}

    /**
     * Apply a symmetric deadband around zero while preserving range outside of the deadband.
     *
     * @param x        raw input in the range [-1, 1].
     * @param deadband fraction of the stick travel to ignore around zero.
     * @return input scaled back to [-1, 1] after deadband compensation.
     */
    public static double applyDeadband(double x, double deadband)
    {
        double abs = Math.abs(x);
        double clampedDeadband = Math.max(0.0, Math.min(Math.abs(deadband), 1.0));
        if (clampedDeadband >= 1.0)
        {
            return 0.0;
        }
        if (abs <= clampedDeadband)
        {
            return 0.0;
        }
        double scaled = (abs - clampedDeadband) / (1.0 - clampedDeadband);
        return Math.copySign(scaled, x);
    }

    /**
     * Apply exponential shaping to highlight fine control near the origin.
     *
     * @param x    input after deadband compensation, assumed to be in [-1, 1].
     * @param expo mixing parameter in [0, 1] where 0 is linear, 1 is a cubic curve.
     * @return input shaped with exponential response.
     */
    public static double applyExpo(double x, double expo)
    {
        double clampedExpo = Math.max(0.0, Math.min(expo, 1.0));
        double cubic = x * x * x;
        return (1.0 - clampedExpo) * x + clampedExpo * cubic;
    }

    /**
     * Scale a unitless input into physical units while clamping to the specified maximum.
     *
     * @param x      unitless command, typically in [-1, 1].
     * @param maxAbs magnitude to scale by, absolute value used for clamping.
     * @return scaled command in the same sign as the input.
     */
    public static double scale(double x, double maxAbs)
    {
        double limit = Math.abs(maxAbs);
        if (limit <= 0.0)
        {
            return 0.0;
        }
        double scaled = x * limit;
        return Math.max(-limit, Math.min(limit, scaled));
    }
}
