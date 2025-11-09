package org.firstinspires.ftc.common.geometry;

import java.util.Vector;

public class Vector2d
{
    public final double x;
    public final double y;

    public Vector2d(double x, double y)
    {
        this.x = x;
        this.y = y;
    }

    public Vector2d plus(Vector2d other)
    {
        return new Vector2d(this.x + other.x, this.y + other.y);
    }

    public Vector2d minus(Vector2d other)
    {
        return new Vector2d(this.x - other.x, this.y - other.y);
    }

    public Vector2d times(double s)
    {
        return new Vector2d(this.x * s, this.y * s);
    }

    public double dot(Vector2d other)
    {
        return this.x * other.x + this.y * other.y;
    }

    public double norm()
    {
        return Math.hypot(x, y);
    }

    public Vector2d normalized()
    {
        double n = norm();
        return (n >= 1e-9) ? new Vector2d(x / n, y / n) : new Vector2d(0.0, 0.0);
    }

    public Vector2d rotate(double angle)
    {
        double c = Math.cos(angle);
        double s = Math.sin(angle);
        return new Vector2d(
                c * x - s * y,
                s * x + c * y
        );
    }

    @Override
    public String toString()
    {
        return String.format("Vector2d(%.4f, %。4f)", x, y);
    }
}
