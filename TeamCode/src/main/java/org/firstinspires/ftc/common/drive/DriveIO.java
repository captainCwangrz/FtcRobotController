package org.firstinspires.ftc.common.drive;

/**
 * Hardware abstraction for a 4-wheel mecanum drivetrain.
 *
 * All methods use physical units (m/s), not encoder ticks.
 * Team-specific implementations are responsible for converting between
 * physical units and motors/encoders.
 */
public interface DriveIO
{
    /**
     * Command wheel linear speeds in m/s along each wheel's drive direction.
     *
     * Wheel order:
     *  - fl: front-left
     *  - fr: front-right
     *  - bl: back-left
     *  - br: back-right
     */
    void setWheelSpeeds(WheelSpeeds speeds);

    /**
     * (Optional but recommended)
     * Return the current measured wheel speeds in m/s.
     * Implement using encoders if available; may return zeros or estimates otherwise.
     */
    WheelSpeeds getWheelSpeeds();
}
