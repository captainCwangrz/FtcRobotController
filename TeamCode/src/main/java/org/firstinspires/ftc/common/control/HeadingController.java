package org.firstinspires.ftc.common.control;

/**
 * Wrapper around {@link PIDController} configured for heading regulation in radians.
 */
public class HeadingController
{
    private final PIDController pid;
    private final double kS; // rad/s
    private final double deadband; // rad

    public HeadingController(double kP, double kI, double kD, double kS, double deadband)
    {
        this.pid = new PIDController(kP, kI, kD);
        this.pid.enableContinuousInput(-Math.PI, Math.PI);
        this.kS = Math.abs(kS);
        this.deadband = Math.abs(deadband)
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
        double pidOutput = pid.calculate(currentHeading, dt);
        double error = pid.getPositionError();

        if (Math.abs(error) > deadband)
        {
            return pidOutput + Math.signum(pidOutput) * kS;
        }
        else
        {
            return 0.0;
        }
    }

    public double getTarget()
    {
        return pid.getSetpoint();
    }

    public double getError()
    {
        return pid.getPositionError();
    }

    public void reset(double currentHeading)
    {
        pid.reset(currentHeading);
    }
}
