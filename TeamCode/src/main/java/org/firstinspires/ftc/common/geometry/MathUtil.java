package org.firstinspires.ftc.common.geometry;

public final class MathUtil
{
    private MathUtil() {}

    public static double wrapAngle(double angle)
    {
        double twoPi = 2.0 * Math.PI;
        angle = angle % twoPi;
        if (angle <= -Math.PI)
        {
            angle += twoPi;
        }
        else if (angle > Math.PI)
        {
            angle -= twoPi;
        }
        return angle;
    }

    public static double clamp(double value, double min, double max)
    {
        return Math.max(min, Math.min(value, max));
    }

    public static boolean epsilonEquals(double a, double b, double eps)
    {
        return Math.abs(a - b) <= eps;
    }
}
