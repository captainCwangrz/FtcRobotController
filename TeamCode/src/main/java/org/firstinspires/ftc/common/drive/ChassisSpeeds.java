package org.firstinspires.ftc.common.drive;

/**
 * Immutable chassis velocity container expressed in the <strong>robot frame</strong>.
 *
 * <p>Units:
 * <ul>
 *     <li>{@code vx}: mm/s, +x forward</li>
 *     <li>{@code vy}: mm/s, +y left</li>
 *     <li>{@code omega}: rad/s, CCW+</li>
 * </ul>
 *
 * <p>This is the canonical "drive command" shape understood by the kinematics layer.</p>
 */
public final class ChassisSpeeds
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
