package org.firstinspires.ftc.common.geometry;

public class Pose2d
{
    public final double x; // m
    public final double y; // m
    public final double heading; // radians, CCW+, wrapped

    public Pose2d(double x, double y, double heading)
    {
        this.x = x;
        this.y = y;
        this.heading = heading;
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
