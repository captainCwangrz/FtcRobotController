package org.firstinspires.ftc.common.control;

/**
 * Limits the rate of change of a signal to keep commands smooth.
 * Works for translational (mm/s) and angular (rad/s) inputs.
 */
public class SlewRateLimiter
{
    private final double rateLimit;

    private double previousValue;
    private double previousTimeSeconds;
    private boolean initialized;

    public SlewRateLimiter(double rateLimit)
    {
        this.rateLimit = Math.abs(rateLimit);
    }

    public double calculate(double input, double timeSeconds)
    {
        if (!initialized)
        {
            previousValue = input;
            previousTimeSeconds = timeSeconds;
            initialized = true;
            return input;
        }

        double deltaTime = timeSeconds - previousTimeSeconds;
        if (deltaTime <= 0.0)
        {
            return previousValue;
        }

        double maxDelta = rateLimit * deltaTime;
        double delta = input - previousValue;
        if (delta > maxDelta)
        {
            delta = maxDelta;
        }
        else if (delta < -maxDelta)
        {
            delta = -maxDelta;
        }

        previousValue += delta;
        previousTimeSeconds = timeSeconds;
        return previousValue;
    }

    public void reset(double value, double timeSeconds)
    {
        previousValue = value;
        previousTimeSeconds = timeSeconds;
        initialized = true;
    }

    public double getRateLimit()
    {
        return rateLimit;
    }

    public double getLastValue()
    {
        return previousValue;
    }
}
