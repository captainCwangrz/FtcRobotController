package org.firstinspires.ftc.common.control;

import static org.junit.Assert.assertEquals;

import org.junit.Before;
import org.junit.Test;

public class PIDControllerTest
{
    private static final double EPS = 1e-9;

    private PIDController controller;

    @Before
    public void setup()
    {
        controller = new PIDController(0.0, 0.0, 0.0);
    }

    @Test
    public void proportionalResponseMatchesGain()
    {
        controller.setPID(1.5, 0.0, 0.0);
        controller.setSetpoint(100.0);

        double output = controller.calculate(92.0, 0.02);
        assertEquals(12.0, output, EPS);
        assertEquals(8.0, controller.getPositionError(), EPS);
    }

    @Test
    public void integralAccumulationDependsOnDt()
    {
        controller.setPID(0.0, 0.5, 0.0);
        controller.setSetpoint(1.0);

        double first = controller.calculate(0.0, 0.1);
        double second = controller.calculate(0.0, 0.1);

        assertEquals(0.05, first, EPS);
        assertEquals(0.10, second, EPS);
    }

    @Test
    public void derivativeUsesMeasurementSlope()
    {
        controller.setPID(0.0, 0.0, 2.0);
        controller.setSetpoint(0.0);

        double first = controller.calculate(0.0, 0.02);
        double second = controller.calculate(0.2, 0.02);

        assertEquals(0.0, first, EPS);
        assertEquals(-20.0, second, 1e-6);
    }

    @Test
    public void continuousInputWrapsAround()
    {
        controller.setPID(1.0, 0.0, 0.0);
        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setSetpoint(Math.PI - 0.1);

        double output = controller.calculate(-Math.PI + 0.1, 0.02);
        assertEquals(0.2, output, 1e-6);
    }

    @Test
    public void resetClearsAccumulatedState()
    {
        controller.setPID(0.5, 0.5, 0.0);
        controller.setSetpoint(2.0);

        controller.calculate(0.0, 0.1);
        controller.calculate(0.0, 0.1);
        controller.reset(0.0);
        double output = controller.calculate(1.0, 0.1);

        assertEquals(0.5, output, EPS);
        assertEquals(1.0, controller.getPositionError(), EPS);
        assertEquals(0.0, controller.getVelocityError(), EPS);
    }
}
