package org.firstinspires.ftc.common.drive;

/**
 * Linear wheel speeds for a 4-wheel mecanum drivetrain.
 *
 * <p>Units: all fields are linear wheel speeds along each wheel's rolling direction (mm/s).</p>
 *
 * <p>Wheel order convention:
 * <ul>
 *     <li>{@code fl}: front-left</li>
 *     <li>{@code fr}: front-right</li>
 *     <li>{@code bl}: back-left</li>
 *     <li>{@code br}: back-right</li>
 * </ul>
 *
 * <p>This is a pure math container; it does not reference hardware identifiers.</p>
 */
public class WheelSpeeds
{
    public double fl;
    public double fr;
    public double bl;
    public double br;

    public WheelSpeeds(double fl, double fr, double bl, double br)
    {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
    }

    /**
     * Returns a new {@link WheelSpeeds} scaled by {@code s}.
     */
    public WheelSpeeds times(double s)
    {
        return new WheelSpeeds(fl * s, fr * s, bl * s, br * s);
    }

    /**
     * Mutates this instance so that the maximum absolute wheel speed is {@code maxMagnitude} mm/s.
     *
     * <p>If all wheels are already within the limit the values are unchanged.</p>
     */
    public void normalize(double maxMagnitude)
    {
        double max = Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))
        );
        if (max > maxMagnitude && max > 1e-9)
        {
            double scale = maxMagnitude / max;
            fl *= scale;
            fr *= scale;
            bl *= scale;
            br *= scale;
        }
    }

    /**
     * Returns the largest absolute wheel speed (mm/s).
     */
    public double maxAbs()
    {
        return Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))
        );
    }

    @Override
    public String toString()
    {
        return String.format("WheelSpeeds(fl=%.4f, fr=%.4f, bl=%.4f, br=%.4f)", fl, fr, bl, br);
    }
}
