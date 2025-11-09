package org.firstinspires.ftc.common.drive;

/**
 * Kinematics for a standard 4-wheel mecanum drivetrain.
 *
 * Assumptions:
 *  - Coordinate convention from common.geometry.CoordinateConvention:
 *      +x: forward, +y: left, omega: CCW+.
 *  - Wheel layout is rectangular:
 *      front-left, front-right, back-left, back-right.
 *  - wheelBase: distance between front and back wheel centers (meters).
 *  - trackWidth: distance between left and right wheel centers (meters).
 *
 * This class converts between:
 *  - ChassisSpeeds (vx, vy, omega) in ROBOT frame
 *  - WheelSpeeds (fl, fr, bl, br) linear speeds (m/s).
 */
public class MecanumKinematics
{
    // Half distances from robot center to wheel rows/columns
    private final double halfWheelBase;
    private final double halfTrackWidth;
    // Combined lever arm used in standard mecanum kinematics
    private final double k;

    /**
     * @param wheelBase  distance between front and back wheel centers (meters)
     * @param trackWidth distance between left and right wheel centers (meters)
     */
    public MecanumKinematics(double wheelBase, double trackWidth)
    {
        this.halfWheelBase = wheelBase / 2.0;
        this.halfTrackWidth = trackWidth / 2.0;
        this.k = halfWheelBase + halfTrackWidth;
    }

    /**
     * Forward kinematics:
     * Given wheel linear speeds (m/s), compute chassis velocities in ROBOT frame.
     *
     * @param wheelSpeeds wheel speeds (fl, fr, bl, br), m/s
     * @return chassis speeds (vx, vy, omega), vx/vy in m/s, omega in rad/s
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
     * Inverse kinematics:
     * Given desired chassis velocities in ROBOT frame, compute wheel linear speeds.
     *
     * @param speeds chassis speeds in ROBOT frame (vx, vy, omega)
     * @return wheel linear speeds (fl, fr, bl, br), m/s
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
