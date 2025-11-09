package org.firstinspires.ftc.common.drive;

/**
 * Chassis velocities expressed in the ROBOT frame.
 *
 * Units:
 *  - vx: mm/s, +x is forward.
 *  - vy: mm/s, +y is left.
 *  - omega: rad/s, CCW positive.
 *
 * This is the only shape of "drive command" that the kinematics layer accepts.
 */
public class ChassisSpeeds
{
    public final double vx; // mm/s, forward
    public final double vy; // mm/s, left
    public final double omega; // rad/s, CCW+

    public ChassisSpeeds(double vx, double vy, double omega)
    {
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
    }

    public static ChassisSpeeds fromRobotRelative(double vx, double vy, double omega)
    {
        return new ChassisSpeeds(vx, vy, omega);
    }

    @Override
    public String toString()
    {
        return String.format("ChassisSpeeds(vx=%.4f, vy=%.4f, omega=%.4f)", vx, vy, omega);
    }
}
