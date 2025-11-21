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
    public double calculate(double input, double timeSeconds) {
        if (!initialized) {
            previousValue = input;
            previousTimeSeconds = timeSeconds;
            initialized = true;
            return input;
        }

        double dt = timeSeconds - previousTimeSeconds;
        previousTimeSeconds = timeSeconds;

        // Guard against invalid time steps
        if (dt < 1e-6) return previousValue;

        // 1. Handle Reversal (Crossing Zero)
        // We are reversing if signs are different AND we aren't already at zero.
        boolean isReversal = (previousValue != 0) && (Math.signum(input) != Math.signum(previousValue));

        if (isReversal) {
            // Calculate time required to brake to zero at decelLimit
            double timeToZero = Math.abs(previousValue) / decelLimit;

            if (dt <= timeToZero) {
                // Case A: We can't reach zero in this step. Just brake.
                // Apply decelLimit in the direction of zero (opposite to current velocity)
                previousValue -= Math.signum(previousValue) * decelLimit * dt;
                return previousValue;
            } else {
                // Case B: We reach zero and have time left.
                // 1. Snap to zero (effectively spending 'timeToZero' seconds)
                previousValue = 0.0;
                // 2. Reduce dt by the time we spent braking
                dt -= timeToZero;
                // 3. Fall through to Normal Logic to handle the remaining acceleration
            }
        }

        // 2. Normal Logic (Same direction or starting from zero)
        // At this point, previousValue and input have the same sign (or prev is 0).
        // We are "accelerating" if moving away from 0, "decelerating" if moving towards 0.
        boolean isAccelerating = Math.abs(input) > Math.abs(previousValue);
        double rateLimit = isAccelerating ? accelLimit : decelLimit;

        // Calculate max allowed change for the (remaining) time step
        double maxChange = rateLimit * dt;
        double desiredChange = input - previousValue;

        // Clamp the desired change
        double clampedChange = Math.max(-maxChange, Math.min(maxChange, desiredChange));
        previousValue += clampedChange;

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
