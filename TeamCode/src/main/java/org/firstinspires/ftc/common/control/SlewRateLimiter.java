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

    /**
     * Calculates the rate-limited output based on the input and current time.
     * Handles direction reversals by decoupling the braking phase (to zero)
     * from the acceleration phase (away from zero).
     */
    public double calculate(double input, double timeSeconds)
    {
        if (!initialized)
        {
            previousValue = input;
            previousTimeSeconds = timeSeconds;
            initialized = true;
            return input;
        }

        double dt = timeSeconds - previousTimeSeconds;
        previousTimeSeconds = timeSeconds;

        // Guard against invalid time steps
        if (dt < 1e-6)
        {
            if (Math.abs(previousValue) < 1e-9)
            {
                previousValue = 0.0;
            }
            return previousValue;
        }

        // 1. Handle Reversal (Crossing Zero)
        boolean isReversal = (previousValue != 0) && (Math.signum(input) != Math.signum(previousValue));

        // Tracks whether we still have time to apply normal accel/decel after braking
        boolean applyNormalLogic = true;

        if (isReversal)
        {
            double timeToZero = Math.abs(previousValue) / decelLimit;

            if (dt <= timeToZero)
            {
                // Case A: We can't reach zero in this step. Just brake.
                previousValue -= Math.signum(previousValue) * decelLimit * dt;
                applyNormalLogic = false; // all dt spent braking
            }
            else
            {
                // Case B: reach zero and have time left; spend timeToZero braking
                previousValue = 0.0;
                dt -= timeToZero;
            }
        }

        // 2. Normal Logic (Same direction or starting from zero)
        if (applyNormalLogic)
        {
            boolean isAccelerating = Math.abs(input) > Math.abs(previousValue);
            double rateLimit = isAccelerating ? accelLimit : decelLimit;

            double maxChange = rateLimit * dt;
            double desiredChange = input - previousValue;
            double clampedChange = Math.max(-maxChange, Math.min(maxChange, desiredChange));

            previousValue += clampedChange;
        }

        // 3. Global Precision Clamp
        if (Math.abs(previousValue) < 1e-9)
        {
            previousValue = 0.0;
        }

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
