package org.firstinspires.ftc.common.drive;

/**
 * Kinematics for a standard 4-wheel mecanum drivetrain.
 *
 * Assumptions:
 *  - Coordinate convention from common.geometry.CoordinateConvention:
 *      +x: forward, +y: left, omega: CCW+.
 *  - Wheel layout is rectangular using wheel order (fl, fr, bl, br).
 *  - wheelBaseMm: distance between front and back wheel centers (mm).
 *  - trackWidthMm: distance between left and right wheel centers (mm).
 *
 * This class converts between:
 *  - ChassisSpeeds (vx, vy, omega) in ROBOT frame
 *  - WheelSpeeds (fl, fr, bl, br) linear speeds (mm/s).
 */
public class MecanumKinematics
{
    // Half distances from robot center to wheel rows/columns
    private final double halfWheelBase;
    private final double halfTrackWidth;
    // Combined lever arm used in standard mecanum kinematics
    private final double k;

    /**
     * @param wheelBaseMm  distance between front and back wheel centers (mm)
     * @param trackWidthMm distance between left and right wheel centers (mm)
     */
    public MecanumKinematics(double wheelBaseMm, double trackWidthMm)
    {
        this.halfWheelBase = wheelBaseMm / 2.0;
        this.halfTrackWidth = trackWidthMm / 2.0;
        this.k = halfWheelBase + halfTrackWidth;
    }

    /**
     * Forward kinematics:
     * Given wheel linear speeds (mm/s), compute chassis velocities in ROBOT frame.
     *
     * @param wheelSpeeds wheel speeds (fl, fr, bl, br), mm/s
     * @return chassis speeds (vx, vy, omega), vx/vy in mm/s, omega in rad/s
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
     * @return wheel linear speeds (fl, fr, bl, br), mm/s
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
