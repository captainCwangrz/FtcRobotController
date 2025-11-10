package org.firstinspires.ftc.common.control;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class SlewRateLimiterTest
{
    private static final double EPS = 1e-9;

    @Test
    public void firstCalculationInitializesState()
    {
        SlewRateLimiter limiter = new SlewRateLimiter(5.0);
        double output = limiter.calculate(3.0, 1.0);

        assertEquals(3.0, output, EPS);
        assertEquals(3.0, limiter.getLastValue(), EPS);
    }

    @Test
    public void rateIsLimitedByMaxDelta()
    {
        SlewRateLimiter limiter = new SlewRateLimiter(2.0);
        limiter.calculate(0.0, 0.0);

        double output = limiter.calculate(10.0, 1.0);
        assertEquals(2.0, output, EPS);

        double output2 = limiter.calculate(-10.0, 2.0);
        assertEquals(0.0, output2, EPS);
    }

    @Test
    public void resetOverridesPreviousState()
    {
        SlewRateLimiter limiter = new SlewRateLimiter(4.0);
        limiter.calculate(0.0, 0.0);
        limiter.reset(5.0, 5.0);

        double output = limiter.calculate(10.0, 5.5);
        assertEquals(6.0, output, EPS);
    }
}
