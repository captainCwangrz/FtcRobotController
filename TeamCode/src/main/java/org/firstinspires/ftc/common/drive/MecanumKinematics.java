package org.firstinspires.ftc.common.drive;

/**
 * Kinematics mapping for a standard rectangular 4-wheel mecanum drivetrain.
 *
 * <p>Geometry definitions:</p>
 * <ul>
 *     <li>{@code wheelBaseMm}: distance between the front and rear wheel centers (mm)</li>
 *     <li>{@code trackWidthMm}: distance between the left and right wheel centers (mm)</li>
 *     <li>Wheel order: {@code fl}, {@code fr}, {@code bl}, {@code br}</li>
 * </ul>
 *
 * <p>All inputs/outputs follow the project convention:</p>
 * <ul>
 *     <li>Chassis speeds {@link ChassisSpeeds}: robot frame, {@code vx}/{@code vy} in mm/s, {@code omega} in rad/s</li>
 *     <li>Wheel speeds {@link WheelSpeeds}: individual wheel linear speeds in mm/s</li>
 * </ul>
 */
public class MecanumKinematics
{
    // Combined lever arm used in standard mecanum kinematics.
    private final double k;

    /**
     * @param wheelBaseMm  distance between front and back wheel centers (mm)
     * @param trackWidthMm distance between left and right wheel centers (mm)
     */
    public MecanumKinematics(double wheelBaseMm, double trackWidthMm)
    {
        this.k = Math.sqrt((wheelBaseMm * wheelBaseMm) / 2 + (trackWidthMm * trackWidthMm) / 2);
    }

    /**
     * Forward kinematics: convert wheel speeds (mm/s) to robot-frame chassis speeds (mm/s, rad/s).
     *
     * <p>Uses the standard mecanum equations:</p>
     * <pre>
     * vx    = (fl + fr + bl + br) / 4
     * vy    = (-fl + fr + bl - br) / 4
     * omega = (-fl + fr - bl + br) / (4 k)
     * </pre>
     */
    public ChassisSpeeds toChassisSpeeds(WheelSpeeds wheelSpeeds)
    {
        double fl = wheelSpeeds.fl;
        double fr = wheelSpeeds.fr;
        double bl = wheelSpeeds.bl;
        double br = wheelSpeeds.br;

        double vx = (fl + fr + bl + br) / 4.0;
        double vy = (-fl + fr + bl - br) / 4.0;
        double omega = (-fl + fr - bl + br) / (4.0 * k);

        return new ChassisSpeeds(vx, vy, omega);
    }

    /**
     * Inverse kinematics: convert robot-frame chassis speeds (mm/s, rad/s) to wheel speeds (mm/s).
     *
     * <p>Uses the standard mecanum equations:</p>
     * <pre>
     * fl = vx - vy - omega k
     * fr = vx + vy + omega k
     * bl = vx + vy - omega k
     * br = vx - vy + omega k
     * </pre>
     */
    public WheelSpeeds toWheelSpeeds(ChassisSpeeds speeds)
    {
        double vx = speeds.vx;
        double vy = speeds.vy;
        double omega = speeds.omega;

        double fl = vx - vy - omega * k;
        double fr = vx + vy + omega * k;
        double bl = vx + vy - omega * k;
        double br = vx - vy + omega * k;

        return new WheelSpeeds(fl, fr, bl, br);
    }
}
