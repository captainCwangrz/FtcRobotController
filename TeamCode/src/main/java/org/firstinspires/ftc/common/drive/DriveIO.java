package org.firstinspires.ftc.common.drive;

/**
 * Hardware abstraction for a 4-wheel mecanum drivetrain.
 *
 * All methods use physical units (mm, mm/s, rad/s), not encoder ticks.
 * Team-specific implementations are responsible for converting between
 * physical units and motors/encoders at the hardware boundary.
 */
public interface DriveIO
{
    /**
     * Command wheel linear speeds in mm/s along each wheel's drive direction.
     *
     * Wheel order:
     *  - fl: front-left
     *  - fr: front-right
     *  - bl: back-left
     *  - br: back-right
     */
    void setWheelSpeeds(WheelSpeeds speeds);

    /**
     * Optional: directly command normalized wheel powers [-1, 1].
     *
     * Implementations may override; default throws to highlight unsupported use.
     */
    default void setWheelPowers(WheelSpeeds powers)
    {
        throw new UnsupportedOperationException("Wheel power control not implemented");
    }

    /**
     * (Optional) Return cumulative wheel travel in mm for each wheel.
     * Default throws to highlight unsupported use.
     */
    default WheelSpeeds getWheelPositions()
    {
        throw new UnsupportedOperationException("Wheel position reporting not implemented");
    }

    /**
     * (Optional but recommended)
     * Return the current measured wheel speeds in mm/s.
     * Implement using encoders if available; default throws to highlight unsupported use.
     */
    default WheelSpeeds getWheelVelocities()
    {
        throw new UnsupportedOperationException("Wheel velocity reporting not implemented");
    }
}
