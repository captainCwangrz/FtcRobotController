package org.firstinspires.ftc.common.control;

/**
 * Basic PID controller that works with setpoints and measurements expressed in mm or rad.
 * Gains are doubles and outputs whatever units the caller expects.
 */
public class PIDController
{
    private double kP;
    private double kI;
    private double kD;

    private double setpoint;
    private double positionError;
    private double velocityError;
    private double totalError;

    private boolean continuous;
    private double minimumInput;
    private double maximumInput;

    private double previousMeasurement;
    private boolean haveMeasurement;

    public PIDController(double kP, double kI, double kD)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setPID(double kP, double kI, double kD)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double getKP()
    {
        return kP;
    }

    public double getKI()
    {
        return kI;
    }

    public double getKD()
    {
        return kD;
    }

    public void setSetpoint(double setpoint)
    {
        if (continuous)
        {
            this.setpoint = placeInRange(setpoint);
        }
        else
        {
            this.setpoint = setpoint;
        }
    }

    public double getSetpoint()
    {
        return setpoint;
    }

    public double getPositionError()
    {
        return positionError;
    }

    public double getVelocityError()
    {
        return velocityError;
    }

    public void enableContinuousInput(double minimumInput, double maximumInput)
    {
        this.continuous = true;
        this.minimumInput = minimumInput;
        this.maximumInput = maximumInput;
        this.setpoint = placeInRange(setpoint);
    }

    public void disableContinuousInput()
    {
        this.continuous = false;
    }

    public double calculate(double measurement, double dtSeconds)
    {
        if (!haveMeasurement)
        {
            previousMeasurement = measurement;
            haveMeasurement = true;
        }

        double error = setpoint - measurement;
        error = getContinuousError(error);

        if (dtSeconds > 0.0)
        {
            totalError += error * dtSeconds;
        }

        double derivative = 0.0;
        if (haveMeasurement && dtSeconds > 0.0)
        {
            double measurementDelta = measurement - previousMeasurement;
            measurementDelta = getContinuousError(measurementDelta);
            derivative = -measurementDelta / dtSeconds;
        }

        positionError = error;
        velocityError = derivative;

        previousMeasurement = measurement;
        haveMeasurement = true;

        return kP * positionError + kI * totalError + kD * velocityError;
    }

    public void reset(double measurement)
    {
        totalError = 0.0;
        positionError = 0.0;
        velocityError = 0.0;
        previousMeasurement = measurement;
        haveMeasurement = false;
    }

    private double getContinuousError(double error)
    {
        if (!continuous)
        {
            return error;
        }

        double range = maximumInput - minimumInput;
        if (range <= 0.0)
        {
            return error;
        }

        error %= range;
        if (error > range / 2.0)
        {
            error -= range;
        }
        else if (error < -range / 2.0)
        {
            error += range;
        }
        return error;
    }

    private double placeInRange(double value)
    {
        double range = maximumInput - minimumInput;
        if (range <= 0.0)
        {
            return value;
        }

        double wrapped = (value - minimumInput) % range;
        if (wrapped < 0.0)
        {
            wrapped += range;
        }
        return wrapped + minimumInput;
    }
}
