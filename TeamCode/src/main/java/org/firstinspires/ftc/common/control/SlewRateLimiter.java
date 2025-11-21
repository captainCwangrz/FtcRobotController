package org.firstinspires.ftc.common.control;

/**
 * Limits the rate of change of a signal to keep commands smooth.
 * Works for translational (mm/s) and angular (rad/s) inputs.
 */
public class SlewRateLimiter
{
    private final double accelLimit;
    private final double decelLimit;

    private double previousValue;
    private double previousTimeSeconds;
    private boolean initialized;

    /**
     * Symmetric slew limiter (legacy behavior).
     */
    public SlewRateLimiter(double rateLimit)
    {
        this(rateLimit, rateLimit);
    }

    /**
     * Asymmetric slew limiter.
     *
     * @param accelLimit maximum positive change rate (units per second) when the command is increasing
     * @param decelLimit maximum negative change rate (units per second) when the command is decreasing
     */
    public SlewRateLimiter(double accelLimit, double decelLimit)
    {
        this.accelLimit = Math.abs(accelLimit);
        this.decelLimit = Math.abs(decelLimit);
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

        double delta = input - previousValue;

        double limit = delta >= 0.0 ? accelLimit : decelLimit;
        double maxDelta = limit * deltaTime;

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

    /**
     * @return positive rate limit when ramping up (units/s)
     */
    public double getAccelLimit()
    {
        return accelLimit;
    }

    /**
     * @return positive rate limit when ramping down (units/s)
     */
    public double getDecelLimit()
    {
        return decelLimit;
    }

    public double getLastValue()
    {
        return previousValue;
    }
}
