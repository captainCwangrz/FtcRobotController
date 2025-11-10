package org.firstinspires.ftc.common.geometry;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class MathUtilTest
{
    private static final double EPS = 1e-9;

    @Test
    public void wrapAngleReturnsSameForInRange()
    {
        assertEquals(0.5, MathUtil.wrapAngle(0.5), EPS);
        assertEquals(-Math.PI + 0.25, MathUtil.wrapAngle(-Math.PI + 0.25), EPS);
        assertEquals(Math.PI, MathUtil.wrapAngle(Math.PI), EPS);
    }

    @Test
    public void wrapAngleWrapsMultipleTurns()
    {
        assertEquals(Math.PI, MathUtil.wrapAngle(3.0 * Math.PI), EPS);
        assertEquals(Math.PI / 2.0, MathUtil.wrapAngle(5.0 * Math.PI / 2.0), EPS);
        assertEquals(Math.PI, MathUtil.wrapAngle(-3.0 * Math.PI), EPS);
    }

    @Test
    public void wrapAnglePeriodicity()
    {
        double angle = -4.2;
        assertEquals(MathUtil.wrapAngle(angle), MathUtil.wrapAngle(angle + 4.0 * Math.PI), EPS);
    }
}
