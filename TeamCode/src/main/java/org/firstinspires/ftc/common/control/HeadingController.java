package org.firstinspires.ftc.common.control;

/**
 * Wrapper around {@link PIDController} configured for heading regulation in radians.
 */
public class HeadingController
{
    private final PIDController pid;

    public HeadingController(double kP, double kI, double kD)
    {
        this.pid = new PIDController(kP, kI, kD);
        this.pid.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Update the target heading.
     *
     * @param headingRad desired heading in radians.
     */
    public void setTarget(double headingRad)
    {
        pid.setSetpoint(headingRad);
    }

    /**
     * Compute the commanded angular velocity in rad/s based on the current heading measurement.
     *
     * @param currentHeading current robot heading in radians.
     * @param dt             loop period in seconds.
     * @return commanded angular velocity in rad/s.
     */
    public double update(double currentHeading, double dt)
    {
        return pid.calculate(currentHeading, dt);
    }

    public double getTarget()
    {
        return pid.getSetpoint();
    }

    public void reset(double currentHeading)
    {
        pid.reset(currentHeading);
    }
}
