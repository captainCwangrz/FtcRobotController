package org.firstinspires.ftc.common.geometry;

/**
 * A 2D pose expressed in the <b>field frame</b> unless otherwise noted.
 *
 * <p>Units:</p>
 * <ul>
 *     <li>{@code x}: millimetres, +x forward from the origin.</li>
 *     <li>{@code y}: millimetres, +y to the left from the origin.</li>
 *     <li>{@code heading}: radians, counter-clockwise positive.</li>
 * </ul>
 */
public class Pose2d
{
    public final double x; // mm in field frame
    public final double y; // mm in field frame
    public final double heading; // radians, CCW+, wrapped

    public Pose2d(double x, double y, double heading)
    {
        this.x = x;
        this.y = y;
        this.heading = MathUtil.wrapAngle(heading);
    }

    public Vector2d vec()
    {
        return new Vector2d(x, y);
    }

    public Pose2d plus(Vector2d translation)
    {
        return new Pose2d(x + translation.x, y + translation.y, heading);
    }

    public Pose2d withHeading(double newHeading)
    {
        return new Pose2d(x, y, newHeading);
    }

    @Override
    public String toString()
    {
        return String.format("Pose2d(%.4f, %.4f, %.4f rad)", x, y, heading);
    }
}
